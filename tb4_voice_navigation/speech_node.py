"""Node ROS điều khiển microphone; việc thu âm/giải mã chạy ở worker riêng."""
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .audio_capture import recognize_microphone
from .commands import RegionCatalog, is_stop_command
from .voice_model import default_voice_model_path


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        model_path = self.declare_parameter('model_path', '').value
        share = get_package_share_directory('tb4_voice_navigation')
        regions_file = self.declare_parameter('regions_file', share+'/maps/regions.yaml').value
        catalog = RegionCatalog(regions_file)
        # Lấy từ vựng từ đúng bản đồ đang dùng, không viết riêng một tên phòng.
        # Đây là gợi ý cho ASR; bộ phân tích lệnh vẫn kiểm tra tên vùng bình thường.
        self.room_vocabulary = ', '.join(region['name'] for region in catalog.regions.values())
        self.use_hotwords = self.declare_parameter('use_hotwords', False).value
        self.declare_parameter('microphone_index', -1)
        self.command_max_age = float(self.declare_parameter('command_max_age', 10.0).value)
        if not math.isfinite(self.command_max_age) or not 1 <= self.command_max_age <= 30:
            raise ValueError('command_max_age phải nằm trong khoảng 1 đến 30 giây')
        self.sample_rate = self.declare_parameter('sample_rate', 16000).value
        self.audio_gain = float(self.declare_parameter('audio_gain', 1.8).value)
        start_listening = self.declare_parameter('start_listening', True).value
        if not math.isfinite(self.audio_gain) or not 0.25 <= self.audio_gain <= 4:
            raise ValueError('audio_gain phải nằm trong khoảng 0.25 đến 4.0')
        if not isinstance(self.sample_rate, int) or self.sample_rate <= 0:
            raise ValueError('sample_rate phải là số nguyên dương')
        self.model_path = Path(model_path).expanduser() if model_path else default_voice_model_path()
        if not all((self.model_path/name).is_file() for name in ('model.bin', 'config.json', 'tokenizer.json')):
            raise FileNotFoundError(f'Thiếu model: {self.model_path}. Chạy download_voice_model trước.')
        self.text_pub = self.create_publisher(String, '/voice_navigation/speech', 10)
        self.partial_pub = self.create_publisher(String, '/voice_navigation/partial', 10)
        self.error_pub = self.create_publisher(String, '/voice_navigation/audio_error', 10)
        self.rejected_pub = self.create_publisher(String, '/voice_navigation/recognition_error', 10)
        self.command_pub = self.create_publisher(String, '/voice_navigation/command', 10)
        self.mic_pub = self.create_publisher(String, '/voice_navigation/microphone_state', 10)
        self.create_service(SetBool, '/voice_navigation/set_listening', self.on_set_listening)
        self.context_audio = multiprocessing.get_context('spawn')
        self.worker = None
        self.messages = None
        self.shutdown_audio = None
        self.enabled = False
        self.worker_ready = False
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.device_name = ''
        self.mic_detail = 'Microphone đang tắt'
        self.partial_stop_sent = False
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.03, self.listen, clock=steady_clock)
        self.create_timer(1.0, self.publish_mic_state, clock=steady_clock)
        if start_listening:
            self.set_listening(True)
        self.publish_mic_state()

    def publish_mic_state(self):
        data = {'enabled': self.enabled, 'detail': self.mic_detail, 'device': self.device_name,
                'ready': self.worker_ready,
                'device_index': self.get_parameter('microphone_index').value}
        self.mic_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'microphone_index':
                if self.enabled:
                    return SetParametersResult(successful=False, reason='Tắt mic trước khi đổi thiết bị')
                if type(parameter.value) is not int or parameter.value < -1:
                    return SetParametersResult(successful=False, reason='Chỉ số mic không hợp lệ')
        return SetParametersResult(successful=True)

    def on_set_listening(self, request, response):
        response.success = self.set_listening(request.data)
        response.message = self.mic_detail
        return response

    def set_listening(self, enabled):
        if enabled == self.enabled:
            self.publish_mic_state()
            return True
        if not enabled:
            self.stop_capture()
            self.mic_detail = 'Microphone đang tắt'
            self.publish_mic_state()
            return True
        try:
            self.messages = self.context_audio.Queue(maxsize=64)
            self.shutdown_audio = self.context_audio.Event()
            self.worker = self.context_audio.Process(target=recognize_microphone,
                args=(str(self.model_path), self.get_parameter('microphone_index').value, self.sample_rate, self.audio_gain,
                      self.messages, self.shutdown_audio, os.getpid(), self.room_vocabulary,
                      self.use_hotwords), daemon=True)
            self.worker.start()
            self.enabled = True
            self.worker_ready = False
            self.last_worker = time.monotonic()
            self.start_deadline = self.last_worker+45
            self.mic_detail = 'Đang tải '+self.model_path.name+' và mở microphone…'
            self.get_logger().info('Model: '+str(self.model_path)+
                                   '; gợi ý từ: '+str(self.use_hotwords))
            self.publish_mic_state()
            return True
        except Exception as error:
            self.stop_capture()
            self.mic_detail = 'Không mở được microphone: '+str(error)
            self.publish_mic_state()
            return False

    def stop_capture(self):
        self.enabled = False
        self.worker_ready = False
        if self.shutdown_audio is not None:
            self.shutdown_audio.set()
        if self.worker is not None and self.worker.pid is not None:
            self.worker.join(timeout=1)
            if self.worker.is_alive():
                self.worker.terminate()
                self.worker.join(timeout=1)
        if self.messages is not None:
            self.messages.close()
            self.messages.cancel_join_thread()
        self.worker = None
        self.messages = None
        self.shutdown_audio = None
        self.partial_stop_sent = False
        self.partial_pub.publish(String(data=''))

    def audio_error(self, reason, stop=False):
        self.error_pub.publish(String(data=reason))
        self.partial_pub.publish(String(data=''))
        self.partial_stop_sent = False
        self.get_logger().warning(reason)
        if stop:
            self.stop_capture()
        self.mic_detail = reason
        self.publish_mic_state()

    def listen(self):
        if not self.enabled:
            return
        # Callback này chỉ chuyển các gói văn bản nhỏ, không chạy Whisper tại đây.
        for _ in range(16):
            try:
                kind, data = self.messages.get_nowait()
            except queue.Empty:
                break
            self.last_worker = time.monotonic()
            if kind == 'ready':
                self.worker_ready = True
                self.device_name = data['device']
                self.mic_detail = 'Đang nghe tiếng Việt'
                self.publish_mic_state()
            elif kind == 'fatal':
                self.audio_error(data, stop=True)
                return
            elif kind == 'warning':
                self.audio_error(data)
            elif kind == 'state':
                self.mic_detail = data
                self.publish_mic_state()
            elif kind == 'rejected':
                self.rejected_pub.publish(String(data=data))
            elif kind == 'recovered':
                self.mic_detail = 'Đã sẵn sàng — hãy nói lại cả câu'
                self.publish_mic_state()
            elif kind == 'final':
                if not -0.25 <= time.time()-float(data['observed_at']) <= self.command_max_age:
                    self.rejected_pub.publish(String(data='Nhận dạng quá lâu; hãy nói lại câu ngắn hơn.'))
                    continue
                self.text_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
                self.get_logger().info('Đã nghe: '+data['text'])
                self.partial_stop_sent = False
                self.partial_pub.publish(String(data=''))
            elif kind == 'partial':
                if time.time()-float(data['observed_at']) > 2:
                    continue
                text = data['text']
                self.partial_pub.publish(String(data=text))
                if not text:
                    self.partial_stop_sent = False
                elif is_stop_command(text):
                    if not self.partial_stop_sent:
                        self.command_pub.publish(String(data='Dừng lại'))
                        self.partial_stop_sent = True
        now = time.monotonic()
        if not self.worker.is_alive():
            self.audio_error('Tiến trình nhận dạng đã dừng; bật lại microphone để thử lại.', stop=True)
        elif not self.worker_ready and now > self.start_deadline:
            self.audio_error('Khởi tạo nhận dạng quá lâu; kiểm tra model và microphone.', stop=True)
        elif self.worker_ready and now-self.last_worker > 30:
            self.audio_error('Bộ nhận dạng không phản hồi; hãy bật lại microphone.', stop=True)

    def close(self):
        self.error_pub.publish(String(data='Node giọng nói kết thúc'))
        self.stop_capture()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
