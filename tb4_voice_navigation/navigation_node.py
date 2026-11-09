"""Nhận văn bản, kiểm tra vùng rồi gửi từng NavigateToPose cho Nav2.

Callback bất đồng bộ giúp vẫn nhận được Dừng lại khi Nav2 đang chạy.
Node không phát cmd_vel: Nav2 chịu trách nhiệm lập đường và tránh vật cản.
"""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .commands import CommandError, RegionCatalog, parse_command
from .map_tools import OccupancyMap


class VoiceNavigationNode(Node):
    def __init__(self):
        super().__init__('voice_navigation_node')
        share = get_package_share_directory('tb4_voice_navigation')
        path = self.declare_parameter('regions_file', share+'/maps/regions.yaml').value
        self.command_max_age = float(self.declare_parameter('command_max_age', 10.0).value)
        if not math.isfinite(self.command_max_age) or not 1 <= self.command_max_age <= 30:
            raise ValueError('command_max_age phải nằm trong khoảng 1 đến 30 giây')
        self.timeout = float(self.declare_parameter('goal_timeout', 180.0).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('goal_timeout phải dương')
        self.catalog = RegionCatalog(path)
        # Kiểm tra lại YAML để không gửi nhầm một điểm đích đã bị sửa vào tường.
        grid = OccupancyMap(self.catalog.path.parent / self.catalog.data['map_yaml'])
        reachable = grid.reachable(self.catalog.data['start'], self.catalog.data['robot_clearance'])
        for region in self.catalog.regions.values():
            goal = region['goal']
            col, row = grid.world_to_pixel(goal['x'], goal['y'])
            if not grid.contains(col, row) or not reachable[row, col]:
                raise ValueError(f"Đích {region['id']} không an toàn trên bản đồ")
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/voice_navigation/status', 10)
        self.create_subscription(String, '/voice_navigation/command', self.on_command, 10)
        self.create_subscription(String, '/voice_navigation/speech', self.on_speech, 10)
        self.create_subscription(String, '/voice_navigation/audio_error', self.on_audio_error, 10)
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/voice_navigation/regions', marker_qos)
        self.route = []
        self.index = 0
        self.busy = False
        # handle là thuộc tính nội bộ của rclpy.Node; dùng tên riêng cho goal Nav2.
        self.goal_handle = None
        self.cancel_requested = False
        self.last_cancel_at = 0.0
        self.cancel_sent = False
        self.started = 0.0
        self.current_state = 'idle'
        self.current_detail = 'Sẵn sàng nhận lệnh'
        # Timeout dùng đồng hồ thật: pause Gazebo không làm callback bị treo.
        self.create_timer(0.2, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.publish_markers)
        # Gửi nhịp trạng thái kể cả khi đứng yên để GUI nhận biết mất kết nối.
        self.create_timer(1.0, self.publish_status,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status('idle', 'Sẵn sàng. Ví dụ: Đi đến khu vực A')

    def status(self, state, detail):
        self.current_state, self.current_detail = state, detail
        self.publish_status()
        self.get_logger().info(detail)

    def publish_status(self):
        value = {'state': self.current_state, 'detail': self.current_detail,
                 'route': self.route, 'index': self.index, 'busy': self.busy,
                 'nav2_ready': self.client.server_is_ready()}
        self.status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def on_speech(self, message):
        try:
            value = json.loads(message.data)
            age = time.time() - float(value['observed_at'])
            if not -0.25 <= age <= self.command_max_age:
                raise ValueError('Kết quả giọng nói đã cũ')
            if value.get('final') is not True:
                raise ValueError('Chỉ nhận câu đã chốt')
            text = value['text']
            if not isinstance(text, str):
                raise ValueError('Văn bản không hợp lệ')
            # Câu đã thu trước lúc dừng có thể vừa giải mã xong. Không cho
            # kết quả đang chờ đó khởi động lại robot sau thao tác dừng.
            started_at = float(value.get('started_at', value['observed_at']))
            if not math.isfinite(started_at) or started_at <= self.last_cancel_at:
                self.get_logger().info('Bỏ câu đã thu trước yêu cầu dừng')
                return
            self.process_text(text)
        except (ValueError, KeyError, TypeError) as error:
            self.status('input_error', str(error))

    def on_command(self, message):
        # Topic thử bằng bàn phím: dùng để tách lỗi ASR khỏi lỗi Nav2.
        self.process_text(message.data)

    def process_text(self, text):
        try:
            command = parse_command(text, self.catalog)
        except CommandError as error:
            self.status('command_error', str(error))
            return
        if command['action'] == 'stop':
            self.request_cancel('Người dùng yêu cầu dừng')
            return
        if self.busy:
            self.status('busy', 'Robot đang chạy; hãy dừng trước khi gửi tuyến mới')
            return
        if not self.client.server_is_ready():
            self.status('unavailable', 'Nav2 chưa sẵn sàng; kiểm tra navigation.launch.py')
            return
        self.route = command['route']
        self.index = 0
        self.busy = True
        self.cancel_requested = False
        self.send_next()

    def send_next(self):
        if self.index >= len(self.route):
            self.busy = False
            self.status('succeeded', 'Đã đến đủ các khu vực theo thứ tự')
            return
        region_id = self.route[self.index]
        target = self.catalog.regions[region_id]['goal']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(target['x']), float(target['y'])
        pose.pose.orientation.z = math.sin(target['yaw']/2)
        pose.pose.orientation.w = math.cos(target['yaw']/2)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_handle = None
        self.cancel_sent = False
        self.started = time.monotonic()
        self.status('navigating', f'Đang đi đến {region_id} ({self.index+1}/{len(self.route)})')
        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self.goal_response)
        except Exception as error:
            self.finish_error(f'Không gửi được goal: {error}')

    def goal_response(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.finish_error('Nav2 từ chối điểm đến')
                return
            self.goal_handle.get_result_async().add_done_callback(self.goal_result)
            # STOP có thể đến trong lúc đang chờ Nav2 chấp nhận goal.
            if self.cancel_requested:
                self.send_cancel()
        except Exception as error:
            self.finish_error(f'Lỗi nhận phản hồi goal: {error}')

    def goal_result(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.finish_error(f'Không nhận được kết quả: {error}')
            return
        self.goal_handle = None
        if self.cancel_requested:
            self.busy = False
            self.status('canceled', 'Tuyến đã dừng; cần lệnh mới để chạy tiếp')
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            # Đây là chỗ bảo đảm A thành công rồi mới gửi B.
            self.index += 1
            self.send_next()
        else:
            self.finish_error(f'Chặng thất bại, status={result.status}; không chạy chặng sau')

    def finish_error(self, detail):
        self.busy = False
        self.goal_handle = None
        self.status('failed', detail)

    def request_cancel(self, reason):
        self.last_cancel_at = time.time()
        if not self.busy:
            self.status('idle', 'Robot không có tuyến đang chạy')
            return
        self.cancel_requested = True
        self.status('canceling', reason)
        if self.goal_handle is not None:
            self.send_cancel()

    def send_cancel(self):
        if self.cancel_sent:
            return
        self.cancel_sent = True
        self.goal_handle.cancel_goal_async().add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.status('cancel_error', 'Nav2 chưa xác nhận hủy; đang chờ kết quả goal')
        except Exception as error:
            self.status('cancel_error', f'Lỗi yêu cầu hủy: {error}')

    def on_audio_error(self, message):
        if self.busy:
            self.request_cancel('Lỗi âm thanh: '+message.data)

    def check_timeout(self):
        if self.busy and not self.cancel_requested and time.monotonic()-self.started > self.timeout:
            self.request_cancel('Quá thời gian cho một chặng')

    def publish_markers(self):
        array = MarkerArray()
        for index, region in enumerate(self.catalog.regions.values()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'region_names'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(region['goal']['x'])
            marker.pose.position.y = float(region['goal']['y'])
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.7, 0.1, 1.0
            marker.text = region['id']
            array.markers.append(marker)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceNavigationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            # Chờ hủy ngắn trước khi đóng node nếu ROS vẫn còn hoạt động.
            if node.busy and rclpy.ok():
                node.request_cancel('Đóng chương trình')
                deadline = time.monotonic()+3.0
                while node.busy and rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
