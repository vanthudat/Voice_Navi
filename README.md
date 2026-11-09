# Phân vùng bản đồ và điều hướng TurtleBot 4 bằng giọng nói

Bài thực hành 7: phân vùng bản đồ bằng OpenCV, nhận dạng tiếng Việt bằng
PhoWhisper-small và điều hướng một hoặc nhiều điểm đến qua Nav2. Giao diện
PyQt5 hiển thị câu nhận dạng, khu vực, đường đi và tiến độ hành trình.

## Môi trường

- Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.
- TurtleBot 4 Simulator, TurtleBot 4 Navigation, Nav2 và SLAM Toolbox.
- Workspace đã build các package mô phỏng/description trong `tb4_sim` mà dự án sử dụng.

Các lệnh dưới đây giả sử package nằm tại `~/ros2_ws/src/tb4_voice_navigation`.

## Cài đặt và build

Cài các phụ thuộc còn thiếu:

```bash
sudo apt install python3-pyqt5 python3-opencv python3-yaml libportaudio2
python3 -m pip install --user 'numpy>=1.23,<2' faster-whisper==1.2.1 webrtcvad-wheels==2.0.14 'sounddevice>=0.4.6,<0.6'
```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_voice_navigation --symlink-install
source install/setup.bash
ros2 run tb4_voice_navigation download_voice_model
```

Mô hình được tải và chuyển sang CTranslate2 int8 trong lần đầu; những lần sau dùng cache tại máy.

## Chạy demo

Nạp môi trường trong mỗi terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
```

**Terminal 1 — Gazebo, AMCL, Nav2 và RViz:**

```bash
ros2 launch tb4_voice_navigation building_navigation.launch.py
```

Dự án dùng bản đồ đã có, không cần chạy SLAM. Có thể thêm `headless:=true` để chạy nền Gazebo và giữ RViz.

**Terminal 2 — Undock, sau đó mở giao diện giọng nói:**

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
```

Sau khi Undock trả về `SUCCEEDED`:

```bash
ros2 launch tb4_voice_navigation voice_navigation.launch.py
```

Đợi **Nav2 đã kết nối**, chọn thiết bị thu rồi bật microphone trên giao diện.
Có thể nhập văn bản hoặc chọn nút khu vực để kiểm tra điều hướng độc lập với nhận dạng giọng nói.

## Các tình huống demo

| Câu lệnh | Kết quả mong đợi |
|---|---|
| Đi đến phòng ngủ. | Tới A — Phòng ngủ. |
| Di chuyển ra hành lang. | Tới C — Hành lang. |
| Đi từ nhà bếp ra phòng khách. | Thực hiện E → D. |
| Đi từ phòng ngủ qua hành lang rồi vào phòng sinh hoạt. | Thực hiện A → C → B. |
| Đi đến khu vực Z. | Báo khu vực không tồn tại. |
| Dừng lại. / Hủy lệnh. | Gửi yêu cầu hủy hành trình. |

Các vùng: **A — Phòng ngủ; B — Phòng sinh hoạt; C — Hành lang; D — Phòng khách; E — Nhà bếp.**
Với tuyến nhiều điểm, robot thực hiện lần lượt từng goal, kể cả điểm xuất phát được nêu trong câu.
Giao diện hiển thị toàn bộ danh sách đích và đường Nav2 của chặng đang chạy.

## Lưu ý khi chạy

- Chỉ mở một phiên mô phỏng và một launch giọng nói; không chạy mapping cùng navigation.
- Nếu scan lệch bản đồ, đặt lại pose bằng **2D Pose Estimate** trong RViz.
- Nếu giao diện chờ Nav2, xem log Terminal 1 và kiểm tra môi trường ROS giữa các terminal.
- Tắt microphone không hủy tuyến. Nút **DỪNG ROBOT** gửi hủy trực tiếp qua Nav2; lệnh dừng bằng giọng nói phải qua nhận dạng.
- Khi kết thúc, hủy hành trình và đợi phản hồi trước khi `Ctrl+C`. Nếu chỉ sửa phần giọng nói/giao diện, build lại và khởi động lại Terminal 2.

## Dữ liệu và cấu hình

- `maps/map.yaml`, `maps/map.pgm`: bản đồ điều hướng.
- `config/region_definitions.yaml`: polygon, tên và điểm đích mong muốn.
- `maps/regions.yaml`: dữ liệu khu vực được node điều hướng sử dụng.
- `maps/segmented_map.png`: ảnh phân vùng; `maps/region_labels.png`: mã vùng cho chương trình.
- `config/nav2.yaml`, `config/localization.yaml`: cấu hình Nav2 và AMCL.
- `launch/building_mapping.launch.py`: quét bản đồ khi cần thay bản đồ hiện tại.
- `ketqua/`: báo cáo và ảnh thực nghiệm.

Sau khi sửa định nghĩa vùng, tạo lại dữ liệu rồi build và mở lại launch liên quan:

```bash
cd ~/ros2_ws/src/tb4_voice_navigation
python3 -m tb4_voice_navigation.segment_map \
  --map maps/map.yaml \
  --definitions config/region_definitions.yaml \
  --output maps
```
