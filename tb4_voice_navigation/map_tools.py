"""Đọc Occupancy Grid và đổi tọa độ. Module này chạy được khi chưa mở ROS."""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class OccupancyMap:
    def __init__(self, yaml_path):
        self.path = Path(yaml_path).resolve()
        data = yaml.safe_load(self.path.read_text())
        if data.get('mode', 'trinary') != 'trinary':
            raise ValueError('Bài này hỗ trợ map mode trinary')
        self.resolution = float(data['resolution'])
        self.origin = tuple(float(v) for v in data['origin'])
        if (not math.isfinite(self.resolution) or self.resolution <= 0
                or len(self.origin) != 3 or not all(map(math.isfinite, self.origin))):
            raise ValueError('resolution/origin không hợp lệ')
        self.image = cv2.imread(str(self.path.parent / data['image']), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError('Không đọc được ảnh bản đồ')
        self.height, self.width = self.image.shape
        # ROS map_server: ảnh trắng là trống khi negate=0.
        probability = self.image.astype(float) / 255.0
        if not data.get('negate', 0):
            probability = 1.0 - probability
        self.free = probability < float(data['free_thresh'])
        # Đệm biên bằng vật cản, tránh chọn điểm sát ngoài ảnh.
        padded = np.pad(self.free.astype('uint8'), 1)
        self.clearance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        self.clearance *= self.resolution

    def pixel_to_world(self, col, row):
        # Pixel có gốc trên trái; map có gốc dưới trái. Dùng tâm ô (+0.5).
        local_x = (col + 0.5) * self.resolution
        local_y = (self.height - row - 0.5) * self.resolution
        ox, oy, yaw = self.origin
        return (ox + math.cos(yaw)*local_x - math.sin(yaw)*local_y,
                oy + math.sin(yaw)*local_x + math.cos(yaw)*local_y)

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        local_x = math.cos(yaw)*dx + math.sin(yaw)*dy
        local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
        return (math.floor(local_x / self.resolution),
                self.height - 1 - math.floor(local_y / self.resolution))

    def contains(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def reachable(self, start, margin):
        # Chỉ giữ ô cách vật cản >= margin. Kết nối 4 tránh đi chéo qua góc tường.
        safe = (self.free & (self.clearance >= margin)).astype('uint8')
        col, row = self.world_to_pixel(*start)
        if not self.contains(col, row) or not safe[row, col]:
            raise ValueError('Vị trí xuất phát không nằm trong vùng an toàn')
        _, components = cv2.connectedComponents(safe, connectivity=4)
        return components == components[row, col]
