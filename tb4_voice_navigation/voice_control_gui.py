"""Cửa sổ nhận lệnh và theo dõi điều hướng; ROS xử lý trong nhịp timer của Qt."""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool
from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QFrame, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QProgressBar, QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget,
)

from .commands import RegionCatalog
from .map_tools import OccupancyMap


STYLE = '''
QMainWindow, QWidget#page { background: #eef3f8; color: #172c42; }
QWidget { font-family: "DejaVu Sans"; font-size: 13px; }
QFrame#header { background: #12283d; border-radius: 16px; }
QLabel#brand { color: #5fe0cd; font-size: 13px; font-weight: bold; }
QLabel#title { color: white; font-size: 23px; font-weight: bold; }
QLabel#subtitle { color: #b7c8d9; }
QFrame#card { background: white; border: 1px solid #dce5ee; border-radius: 14px; }
QLabel#section { color: #172c42; font-size: 16px; font-weight: bold; }
QLabel#muted { color: #63798d; }
QLabel#transcript { color: #173c58; font-size: 19px; font-weight: bold;
    background: #edf7f6; border-radius: 10px; padding: 16px; }
QLabel#state { color: #176f64; font-size: 18px; font-weight: bold; }
QLabel#badge { color: #d0e1ee; background: #263f55; border-radius: 12px; padding: 8px 12px; }
QPushButton { background: #eef3f8; color: #24445f; border: 1px solid #d5e0eb;
    border-radius: 9px; padding: 11px; font-weight: bold; }
QPushButton:hover { background: #dfeaf4; }
QPushButton:disabled { color: #8b9bad; background: #f0f3f7; border-color: #e2e8ef; }
QPushButton#mic, QPushButton#send { background: #087f73; color: white; border: none; }
QPushButton#mic:hover, QPushButton#send:hover { background: #07695f; }
QPushButton#mic:disabled, QPushButton#send:disabled { background: #9cbeb9; color: white; }
QPushButton#stop { background: #fff0f0; color: #be303c; border: 1px solid #efbec3; }
QPushButton#stop:hover { background: #ffe0e3; }
QLineEdit { background: #f7f9fc; border: 1px solid #cddae5; border-radius: 9px;
    padding: 12px; color: #1a344d; }
QLineEdit:focus { border: 1px solid #089b89; }
QProgressBar { background: #eaf0f5; border: none; border-radius: 6px; height: 18px;
    text-align: center; color: #173d45; }
QProgressBar::chunk { background: #72d7c2; border-radius: 6px; }
QSplitter::handle { background: transparent; width: 12px; }
'''
COLORS = ['#f5b95e', '#75c995', '#79b3e2', '#d597c3', '#d7cc6d']


def label(text='', name='', wrap=False):
    widget = QLabel(text)
    widget.setTextFormat(Qt.PlainText)
    widget.setWordWrap(wrap)
    if name:
        widget.setObjectName(name)
    return widget


def card():
    frame = QFrame()
    frame.setObjectName('card')
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(18, 16, 18, 16)
    layout.setSpacing(12)
    return frame, layout


class MapView(QWidget):
    """Vẽ map, vùng, đường Nav2 và pose AMCL; không tự gửi goal từ ảnh."""
    def __init__(self, catalog, grid):
        super().__init__()
        self.catalog, self.grid = catalog, grid
        self.pose = None
        self.pose_time = 0.0
        self.path = []
        self.active_region = None
        self.setMinimumHeight(160)
        # Tô màu từ mã vùng, chỉ tô lên ô trống đã được phân vùng.
        pixels = cv2.cvtColor(grid.image, cv2.COLOR_GRAY2RGB)
        labels = cv2.imread(str(grid.path.parent/'region_labels.png'), cv2.IMREAD_GRAYSCALE)
        if labels is not None and labels.shape == grid.image.shape:
            for i, region in enumerate(catalog.regions.values()):
                color = QColor(COLORS[i % len(COLORS)])
                mask = (labels == region['label']) & grid.free
                rgb = np.array([color.red(), color.green(), color.blue()])
                pixels[mask] = (0.55*pixels[mask] + 0.45*rgb).astype('uint8')
        h, w = pixels.shape[:2]
        self.image = QImage(pixels.data, w, h, pixels.strides[0], QImage.Format_RGB888).copy()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#f3f6fa'))
        scale = min((self.width()-36)/self.grid.width, (self.height()-36)/self.grid.height)
        dx = (self.width()-self.grid.width*scale)/2
        dy = (self.height()-self.grid.height*scale)/2
        painter.save()
        painter.translate(dx, dy)
        painter.scale(scale, scale)
        painter.drawImage(0, 0, self.image)
        painter.restore()

        def screen(x, y):
            col, row = self.grid.world_to_pixel(x, y)
            return QPointF(dx+(col+0.5)*scale, dy+(row+0.5)*scale)

        if self.path:
            line = QPainterPath()
            line.moveTo(screen(*self.path[0]))
            for x, y in self.path[1:]:
                line.lineTo(screen(x, y))
            painter.setPen(QPen(QColor('#008dce'), 2.5))
            painter.drawPath(line)
        for i, region in enumerate(self.catalog.regions.values()):
            goal = region['goal']
            point = screen(goal['x'], goal['y'])
            active = region['id'] == self.active_region
            painter.setPen(QPen(QColor('#0b6e65') if active else QColor('white'), 3))
            painter.setBrush(QColor(COLORS[i % len(COLORS)]))
            painter.drawEllipse(point, 14 if active else 11, 14 if active else 11)
            painter.setPen(QColor('#19354a'))
            painter.setFont(QFont('DejaVu Sans', 10, QFont.Bold))
            painter.drawText(int(point.x()-5), int(point.y()+5), region['id'])
        if self.pose is not None:
            x, y, yaw = self.pose
            point = screen(x, y)
            painter.save()
            if time.monotonic()-self.pose_time > 5:
                painter.setOpacity(0.4)
            painter.translate(point)
            painter.rotate(-math.degrees(yaw-self.grid.origin[2]))
            painter.setPen(QPen(QColor('white'), 2))
            painter.setBrush(QColor('#12334d'))
            painter.drawPolygon(QPolygonF([QPointF(14, 0), QPointF(-10, -9),
                                            QPointF(-5, 0), QPointF(-10, 9)]))
            painter.restore()
        painter.end()


class VoiceControlWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        share = get_package_share_directory('tb4_voice_navigation')
        regions_file = node.declare_parameter('regions_file', share+'/maps/regions.yaml').value
        self.catalog = RegionCatalog(regions_file)
        self.grid = OccupancyMap(self.catalog.path.parent/self.catalog.data['map_yaml'])
        self.nav_seen = self.mic_seen = 0.0
        self.nav_ready = self.busy = self.mic_enabled = False
        self.mic_future = None
        self.device_future = None
        self.confirmed_device = -1
        self.setWindowTitle('TurtleBot 4 • Điều hướng bằng giọng nói')
        self.setMinimumSize(1000, 650)
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1320, screen.width()-40), min(860, screen.height()-65))
        self.setStyleSheet(STYLE)
        self.build_ui()

        # Qt và callback ROS chạy cùng luồng, không sửa widget từ luồng khác.
        self.command_pub = node.create_publisher(String, '/voice_navigation/command', 10)
        self.device_client = node.create_client(SetParameters, '/speech_node/set_parameters')
        self.mic_choice.currentIndexChanged.connect(self.change_microphone)
        self.mic_client = node.create_client(SetBool, '/voice_navigation/set_listening')
        node.create_subscription(String, '/voice_navigation/status', self.on_status, 10)
        node.create_subscription(String, '/voice_navigation/microphone_state', self.on_mic, 10)
        node.create_subscription(String, '/voice_navigation/speech', self.on_speech, 10)
        node.create_subscription(String, '/voice_navigation/partial', self.on_partial, 10)
        node.create_subscription(String, '/voice_navigation/audio_error', self.on_audio_error, 10)
        node.create_subscription(String, '/voice_navigation/recognition_error', self.on_audio_error, 10)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose,
                                 qos_profile_sensor_data)
        node.create_subscription(NavPath, '/plan', self.on_path, qos_profile_sensor_data)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.poll_ros)
        self.ros_timer.start(25)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_connections)
        self.ui_timer.start(500)
        self.refresh_connections()

    def build_ui(self):
        page = QWidget()
        page.setObjectName('page')
        self.setCentralWidget(page)
        root = QVBoxLayout(page)
        root.setContentsMargins(20, 18, 20, 12)
        root.setSpacing(16)
        header = QFrame()
        header.setObjectName('header')
        bar = QHBoxLayout(header)
        bar.setContentsMargins(22, 18, 22, 18)
        titles = QVBoxLayout()
        titles.addWidget(label('TURTLEBOT 4  /  VOICE CONTROL', 'brand'))
        titles.addWidget(label('Bạn nói đích đến. Robot thực hiện.', 'title'))
        titles.addWidget(label('Điều khiển bằng tiếng Việt · Theo dõi hành trình trực tiếp', 'subtitle'))
        bar.addLayout(titles, 1)
        self.connection = label('Đang kết nối', 'badge')
        bar.addWidget(self.connection)
        root.addWidget(header)

        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        left_box = QVBoxLayout(left)
        left_box.setContentsMargins(0, 0, 0, 0)
        left_box.setSpacing(12)
        map_frame, map_box = card()
        top = QHBoxLayout()
        top.addWidget(label('Bản đồ khu vực', 'section'))
        top.addStretch()
        self.pose_label = label('Chưa nhận vị trí robot', 'muted')
        top.addWidget(self.pose_label)
        map_box.addLayout(top)
        self.map_view = MapView(self.catalog, self.grid)
        map_box.addWidget(self.map_view, 1)
        map_box.addWidget(label('Mũi tên: robot  ·  Đường xanh: kế hoạch Nav2  ·  A–E: điểm đến', 'muted', True))
        left_box.addWidget(map_frame, 1)
        rooms_frame, rooms_box = card()
        rooms_box.addWidget(label('Chọn nhanh điểm đến', 'section'))
        buttons = QHBoxLayout()
        self.room_buttons = []
        for region in self.catalog.regions.values():
            button = QPushButton(region['id']+'\n'+region['name'])
            button.setToolTip('Gửi lệnh: Đi đến '+region['name'])
            button.clicked.connect(lambda checked=False, name=region['name']:
                                   self.send_command('Đi đến '+name))
            buttons.addWidget(button)
            self.room_buttons.append(button)
        rooms_box.addLayout(buttons)
        left_box.addWidget(rooms_frame)
        split.addWidget(left)

        right = QWidget()
        right.setMinimumWidth(350)
        right_box = QVBoxLayout(right)
        right_box.setContentsMargins(0, 0, 0, 0)
        right_box.setSpacing(12)
        voice_frame, voice_box = card()
        voice_box.addWidget(label('Trợ lý giọng nói', 'section'))
        self.mic_detail = label('Đang kết nối microphone…', 'muted', True)
        voice_box.addWidget(self.mic_detail)
        self.mic_device = label('Thiết bị thu: đang chờ kết nối', 'muted', True)
        voice_box.addWidget(self.mic_device)
        self.mic_choice = QComboBox()
        self.mic_choice.addItem('Microphone mặc định của hệ thống', -1)
        # Chỉ liệt kê thiết bị, chưa mở stream thu âm khi tạo cửa sổ.
        try:
            import sounddevice as sd
            for index, device in enumerate(sd.query_devices()):
                if device['max_input_channels'] > 0:
                    self.mic_choice.addItem(f"{index}: {device['name']}", index)
        except Exception as error:
            self.mic_device.setText('Không liệt kê được mic: '+str(error))
        voice_box.addWidget(self.mic_choice)
        self.mic_button = QPushButton('●  Bật microphone')
        self.mic_button.setObjectName('mic')
        self.mic_button.setMinimumHeight(49)
        self.mic_button.clicked.connect(self.toggle_microphone)
        voice_box.addWidget(self.mic_button)
        self.transcript = label('“Đi đến phòng khách”', 'transcript', True)
        self.transcript.setMinimumHeight(78)
        voice_box.addWidget(self.transcript)
        self.partial = label('Câu vừa nghe sẽ xuất hiện ở đây.', 'muted', True)
        voice_box.addWidget(self.partial)
        self.input = QLineEdit()
        self.input.setPlaceholderText('Hoặc nhập: Đi từ A đến B')
        self.input.returnPressed.connect(self.send_typed)
        voice_box.addWidget(self.input)
        self.send_button = QPushButton('Gửi yêu cầu')
        self.send_button.setObjectName('send')
        self.send_button.clicked.connect(self.send_typed)
        voice_box.addWidget(self.send_button)
        right_box.addWidget(voice_frame)

        trip_frame, trip_box = card()
        trip_box.addWidget(label('Hành trình', 'section'))
        self.state_label = label('Đang chờ kết nối', 'state')
        self.detail_label = label('Mở mô phỏng và Nav2 để bắt đầu.', 'muted', True)
        self.route_label = label('Chưa có tuyến đường', '', True)
        trip_box.addWidget(self.state_label)
        trip_box.addWidget(self.detail_label)
        trip_box.addWidget(self.route_label)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat('%v/%m điểm đã đến')
        trip_box.addWidget(self.progress)
        right_box.addWidget(trip_frame)

        right_box.addStretch(1)
        # Nội dung cuộn được trên màn hình laptop; nút dừng luôn nằm ngoài vùng cuộn.
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(right)
        scroll.setMinimumWidth(380)
        split.addWidget(scroll)
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 1)
        split.setSizes([790, 410])
        root.addWidget(split, 1)
        footer = QHBoxLayout()
        footer.addWidget(label('Nói rõ một câu và chờ hoàn tất. Tắt mic không hủy hành trình.',
                               'muted', True), 1)
        self.stop_button = QPushButton('■  DỪNG ROBOT')
        self.stop_button.setObjectName('stop')
        self.stop_button.setMinimumSize(190, 43)
        self.stop_button.clicked.connect(self.stop_robot)
        footer.addWidget(self.stop_button)
        root.addLayout(footer)

    def poll_ros(self):
        if not rclpy.ok():
            self.close()
            return
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0)

    def refresh_connections(self):
        now = time.monotonic()
        connected = now-self.nav_seen < 4
        ready = connected and self.nav_ready
        if not connected:
            self.connection.setText('○  Chờ bộ điều khiển')
        elif not ready:
            self.connection.setText('○  Chờ Nav2 sẵn sàng')
        else:
            self.connection.setText('●  Nav2 đã kết nối')
        can_send = ready and not self.busy
        self.send_button.setEnabled(can_send)
        for button in self.room_buttons:
            button.setEnabled(can_send)
        self.stop_button.setEnabled(connected)
        mic_connected = now-self.mic_seen < 4
        self.mic_button.setEnabled(mic_connected and self.mic_future is None and self.device_future is None)
        self.mic_choice.setEnabled(mic_connected and not self.mic_enabled and
                                   self.mic_future is None and self.device_future is None)
        if self.device_future is not None and now > self.device_deadline:
            self.device_future = None
            self.mic_detail.setText('Đổi mic chưa có phản hồi; chờ đồng bộ thiết bị.')
        if not mic_connected:
            self.mic_detail.setText('Chưa có kết nối nhận giọng nói.')
            self.mic_button.setText('●  Chờ microphone')
        if self.mic_future is not None and now > self.mic_deadline:
            self.mic_future = None
            self.mic_detail.setText('Chưa nhận được phản hồi bật/tắt mic.')
        if not connected:
            self.state_label.setText('Mất kết nối điều hướng' if self.nav_seen else 'Đang chờ kết nối')
            self.detail_label.setText('Chưa nhận được trạng thái của bộ điều khiển.')
        if self.map_view.pose is not None and now-self.map_view.pose_time > 5:
            self.pose_label.setText('Vị trí robot chưa cập nhật')
        self.map_view.update()

    def send_command(self, text):
        text = text.strip()
        if not text:
            return
        # Không xếp hàng ngầm các lệnh mới khi robot đang thực hiện tuyến cũ.
        if time.monotonic()-self.nav_seen >= 4 or not self.nav_ready or self.busy:
            self.detail_label.setText('Chờ kết nối Nav2 và hành trình hiện tại kết thúc.')
            return
        self.command_pub.publish(String(data=text))
        self.transcript.setText(text)
        self.detail_label.setText('Đã gửi yêu cầu, đang chờ phản hồi.')

    def send_typed(self):
        self.send_command(self.input.text())

    def stop_robot(self):
        # GUI yêu cầu hủy goal, kết quả thật do node điều hướng gửi về.
        self.command_pub.publish(String(data='Dừng lại'))
        self.detail_label.setText('Đang chờ bộ điều khiển xác nhận hủy tuyến.')

    def change_microphone(self, index):
        if self.mic_enabled or self.device_future is not None:
            return
        if not self.device_client.service_is_ready():
            self.mic_detail.setText('Chưa kết nối dịch vụ chọn microphone.')
            self.select_device(self.confirmed_device)
            return
        device = self.mic_choice.itemData(index)
        request = SetParameters.Request()
        request.parameters = [Parameter('microphone_index', Parameter.Type.INTEGER, device).to_parameter_msg()]
        self.device_future = self.device_client.call_async(request)
        self.device_deadline = time.monotonic()+6
        self.device_future.add_done_callback(self.device_response)
        self.mic_choice.setEnabled(False)
        self.mic_button.setEnabled(False)

    def select_device(self, device):
        index = self.mic_choice.findData(device)
        if index < 0:
            self.mic_choice.addItem('Thiết bị '+str(device), device)
            index = self.mic_choice.count()-1
        self.mic_choice.blockSignals(True)
        self.mic_choice.setCurrentIndex(index)
        self.mic_choice.blockSignals(False)

    def device_response(self, future):
        if future is not self.device_future:
            return
        self.device_future = None
        try:
            results = future.result().results
            if not results or not results[0].successful:
                reason = results[0].reason if results else 'Không có phản hồi'
                raise RuntimeError(reason)
            self.confirmed_device = self.mic_choice.currentData()
            self.mic_device.setText('Đã chọn: '+self.mic_choice.currentText())
        except Exception as error:
            self.select_device(self.confirmed_device)
            self.mic_detail.setText('Không đổi được microphone: '+str(error))

    def toggle_microphone(self):
        if self.mic_future is not None or not self.mic_client.service_is_ready():
            self.mic_detail.setText('Dịch vụ bật/tắt mic chưa sẵn sàng.')
            return
        request = SetBool.Request()
        request.data = not self.mic_enabled
        self.mic_future = self.mic_client.call_async(request)
        self.mic_deadline = time.monotonic()+6
        self.mic_future.add_done_callback(self.mic_response)
        self.mic_button.setEnabled(False)

    def mic_response(self, future):
        if future is not self.mic_future:
            return
        self.mic_future = None
        try:
            response = future.result()
            self.mic_detail.setText(response.message)
        except Exception as error:
            self.mic_detail.setText('Lỗi microphone: '+str(error))
        self.refresh_connections()

    def on_mic(self, message):
        try:
            data = json.loads(message.data)
            self.mic_enabled = bool(data['enabled'])
            self.mic_seen = time.monotonic()
            if self.device_future is None:
                self.confirmed_device = int(data.get('device_index', -1))
                self.select_device(self.confirmed_device)
            self.mic_detail.setText(data['detail'])
            if data.get('device'):
                self.mic_device.setText('Thiết bị thu: '+data['device'])
            self.mic_button.setText('●  Tắt microphone' if self.mic_enabled else '●  Bật microphone')
        except (ValueError, KeyError, TypeError):
            return

    def on_partial(self, message):
        self.partial.setText('Đang nghe: '+message.data if message.data else 'Chờ câu lệnh tiếp theo…')

    def on_speech(self, message):
        try:
            data = json.loads(message.data)
            self.transcript.setText(data['text'])
        except (ValueError, KeyError, TypeError):
            return

    def on_audio_error(self, message):
        self.mic_detail.setText(message.data)
        self.transcript.setText('Chưa nhận đủ câu. Hãy nói lại.')
        self.partial.setText('Chờ âm thanh ổn định…')

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            state, detail = data['state'], data['detail']
            self.nav_seen = time.monotonic()
            self.nav_ready = bool(data.get('nav2_ready', False))
            self.busy = bool(data.get('busy', False))
            names = {'idle': 'Sẵn sàng', 'navigating': 'Đang di chuyển', 'succeeded': 'Đã đến nơi',
                     'canceling': 'Đang yêu cầu dừng', 'canceled': 'Đã dừng hành trình',
                     'command_error': 'Không hiểu điểm đến', 'input_error': 'Lệnh không hợp lệ',
                     'unavailable': 'Nav2 chưa sẵn sàng', 'failed': 'Điều hướng thất bại',
                     'busy': 'Đang thực hiện hành trình', 'cancel_error': 'Chưa xác nhận dừng'}
            self.state_label.setText(names.get(state, state))
            self.detail_label.setText(detail)
            route = data.get('route', [])
            index = max(0, min(int(data.get('index', 0)), len(route)))
            display = [r+' · '+self.catalog.regions[r]['name'] for r in route if r in self.catalog.regions]
            self.route_label.setText(' → '.join(display) if display else 'Chưa có tuyến đường')
            self.progress.setRange(0, max(1, len(route)))
            self.progress.setValue(index)
            self.map_view.active_region = route[index] if self.busy and index < len(route) else None
            if not self.busy:
                self.map_view.path = []
            self.refresh_connections()
        except (ValueError, KeyError, TypeError):
            return

    def on_pose(self, message):
        if message.header.frame_id != 'map':
            return
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.map_view.pose = (pose.position.x, pose.position.y, yaw)
        self.map_view.pose_time = time.monotonic()
        self.pose_label.setText(f'Robot: {pose.position.x:.1f}, {pose.position.y:.1f} m')
        self.map_view.update()

    def on_path(self, message):
        if message.header.frame_id == 'map' and self.busy:
            self.map_view.path = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            self.map_view.update()

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.ui_timer.stop()
        if rclpy.ok():
            self.command_pub.publish(String(data='Dừng lại'))
            if self.mic_client.service_is_ready():
                self.mic_client.call_async(SetBool.Request(data=False))
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    # Chỉ bỏ giá trị do bản OpenCV pip tự thêm, giữ cấu hình Qt của người dùng.
    for key in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        if '/cv2/' in os.environ.get(key, ''):
            os.environ.pop(key, None)
    app = QApplication([])
    node = Node('voice_control_gui')
    try:
        window = VoiceControlWindow(node)
        window.show()
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
