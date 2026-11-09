"""Vẽ vùng thủ công, chọn đích an toàn, xuất ảnh và YAML để Nav2 sử dụng."""
import argparse
import unicodedata
from pathlib import Path

import cv2
import numpy as np
import yaml

from .map_tools import OccupancyMap


def segment(map_path, definitions_path, output):
    grid = OccupancyMap(map_path)
    definitions = yaml.safe_load(Path(definitions_path).read_text())
    margin = float(definitions['robot_clearance'])
    if not np.isfinite(margin) or margin <= 0:
        raise ValueError('robot_clearance phải dương')
    reachable = grid.reachable(definitions['start'], margin)
    labels = np.zeros(grid.image.shape, dtype='uint8')
    preview = cv2.cvtColor(grid.image, cv2.COLOR_GRAY2BGR)
    colors = [(70, 170, 245), (120, 210, 100), (220, 150, 100),
              (180, 130, 220), (100, 210, 220)]
    regions = []
    for number, region in enumerate(definitions['regions'], start=1):
        polygon = np.array([grid.world_to_pixel(*point) for point in region['polygon']], dtype='int32')
        mask = np.zeros_like(labels)
        cv2.fillPoly(mask, [polygon], 1)
        # Biên chung thuộc vùng được vẽ trước. Không ghi nhãn lên tường/unknown.
        inside = mask.astype(bool) & grid.free & (labels == 0)
        labels[inside] = number
        candidates = np.argwhere(inside & reachable)
        if not len(candidates):
            raise ValueError(f"Vùng {region['id']} không có điểm đến an toàn nối với start")
        preferred_x, preferred_y, yaw = region['preferred_goal']
        col, row = grid.world_to_pixel(preferred_x, preferred_y)
        # Nếu điểm mong muốn không an toàn, lấy ô hợp lệ gần nó nhất.
        distance = (candidates[:, 0]-row)**2 + (candidates[:, 1]-col)**2
        chosen_row, chosen_col = candidates[np.argmin(distance)]
        x, y = grid.pixel_to_world(int(chosen_col), int(chosen_row))
        regions.append({
            'id': region['id'], 'name': region['name'], 'aliases': region['aliases'],
            'label': number, 'polygon': region['polygon'],
            'goal': {'x': x, 'y': y, 'yaw': float(yaw)},
            'clearance_m': float(grid.clearance[chosen_row, chosen_col]),
        })
        color = np.array(colors[(number-1) % len(colors)])
        preview[inside] = (0.35*preview[inside] + 0.65*color).astype('uint8')
        cv2.circle(preview, (chosen_col, chosen_row), 5, (0, 0, 0), -1)
        cv2.putText(preview, region['id'], (chosen_col+7, chosen_row-7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    if len(regions) < 3:
        raise ValueError('Đề bài yêu cầu tối thiểu 3 khu vực')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Phóng ảnh và thêm chú giải để thấy rõ tên phòng và hành lang.
    enlarged = cv2.resize(preview, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    legend = np.full((enlarged.shape[0], 270, 3), 250, dtype='uint8')
    cv2.putText(legend, 'PHONG VA HANH LANG', (12, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (25, 25, 25), 1)
    for i, region in enumerate(regions):
        y = 85 + i*60
        cv2.rectangle(legend, (12, y-18), (32, y+2), colors[i % len(colors)], -1)
        name = unicodedata.normalize('NFD', region['name']).replace('đ', 'd').replace('Đ', 'D')
        name = ''.join(c for c in name if not unicodedata.combining(c))
        cv2.putText(legend, region['id']+' - '+name, (42, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1)
    cv2.putText(legend, 'Cham den: dich Nav2', (12, 420),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1)
    cv2.putText(legend, 'Den: tuong / do vat', (12, 450),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1)
    cv2.imwrite(str(output/'segmented_map.png'), np.hstack([enlarged, legend]))
    cv2.imwrite(str(output/'region_labels.png'), labels)
    # Tên ảnh tương đối để mang package sang máy khác vẫn dùng được.
    result = {'frame_id': 'map', 'map_yaml': 'map.yaml', 'method': 'manual_polygons',
              'robot_clearance': margin, 'start': definitions['start'], 'regions': regions}
    (output/'regions.yaml').write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
    for region in regions:
        print(f"{region['id']}: {region['goal']}, cách vật cản {region['clearance_m']:.2f} m")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--definitions', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    segment(args.map, args.definitions, args.output)


if __name__ == '__main__':
    main()
