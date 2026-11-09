"""Hiểu câu lệnh theo mẫu rõ ràng; không đoán gần đúng tên khu vực."""
import math
import re
import unicodedata
from pathlib import Path

import yaml


def normalize(text):
    text = unicodedata.normalize('NFD', text.lower().replace('đ', 'd'))
    text = ''.join(char for char in text if not unicodedata.combining(char))
    return ' '.join(re.findall(r'[a-z0-9]+', text))


def is_stop_command(text):
    """Nhận cả câu dừng thông dụng; không tìm từ 'dừng' trong câu bất kỳ."""
    return normalize(text) in {
        'dung', 'dung lai', 'dung robot', 'robot dung lai',
        'dung ngay', 'dung di chuyen', 'hay dung lai',
        'huy', 'huy lenh', 'huy dieu huong', 'stop',
    }


class CommandError(ValueError):
    pass


class RegionCatalog:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.data = yaml.safe_load(self.path.read_text())
        if self.data.get('frame_id') != 'map':
            raise ValueError('Đích phải thuộc hệ tọa độ map')
        self.regions = {}
        self.aliases = {}
        for region in self.data['regions']:
            region_id = region['id']
            if region_id in self.regions:
                raise ValueError(f'ID trùng: {region_id}')
            if not all(math.isfinite(float(region['goal'][key])) for key in ('x', 'y', 'yaw')):
                raise ValueError('Tọa độ đích không hữu hạn')
            self.regions[region_id] = region
            names = [region_id, region['name'], *region.get('aliases', [])]
            for name in names:
                alias = normalize(name)
                if alias in self.aliases and self.aliases[alias] != region_id:
                    raise ValueError(f'Tên gọi bị trùng giữa các vùng: {name}')
                self.aliases[alias] = region_id
        if not self.regions:
            raise ValueError('Danh sách khu vực rỗng')

    def resolve(self, name):
        key = normalize(name)
        if key not in self.aliases:
            available = ', '.join(self.regions)
            raise CommandError(f'Khu vực không tồn tại: "{name}". Các vùng hiện có: {available}')
        return self.aliases[key]


def parse_command(text, catalog):
    sentence = normalize(text)
    # Chỉ câu dừng rõ ràng được coi là STOP; câu phủ định không tạo chuyển động.
    if is_stop_command(text):
        return {'action': 'stop', 'route': []}
    if any(word in sentence.split() for word in ('khong', 'dung', 'chua')):
        raise CommandError('Câu phủ định/không rõ ý định; không tạo tuyến đường')
    match = re.fullmatch(r'(?:(?:di|di chuyen) )?tu (.+)', sentence)
    if match:
        # Các từ nối đều biểu thị chặng kế tiếp, ví dụ:
        # đi từ nhà bếp ra phòng khách -> [nhà bếp, phòng khách].
        # Giữ thứ tự khi có nhiều chặng: từ A qua C rồi đi vào B.
        connector = r' (?:roi (?:di )?)?(?:den|ra|vao|toi|qua|sang) '
        names = re.split(connector, match.group(1))
        if len(names) < 2:
            raise CommandError('Lệnh "từ" phải có nơi xuất phát và nơi đến')
    else:
        match = re.fullmatch(r'(?:(?:di|di chuyen) (?:den|ra|vao|toi|sang)|den|ra|vao|toi|sang) (.+)', sentence)
        if not match:
            raise CommandError('Chưa hiểu câu lệnh. Ví dụ: Đi đến khu vực A')
        names = [match.group(1)]
    # Tra cứu hết trước: không chạy nửa câu nếu chặng sau chứa vùng lạ.
    route = [catalog.resolve(name) for name in names]
    return {'action': 'navigate', 'route': route}


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Thử hiểu câu lệnh mà không di chuyển robot')
    parser.add_argument('--regions', required=True)
    parser.add_argument('text')
    args = parser.parse_args()
    try:
        print(parse_command(args.text, RegionCatalog(args.regions)))
    except CommandError as error:
        parser.exit(2, str(error)+'\n')


if __name__ == '__main__':
    main()
