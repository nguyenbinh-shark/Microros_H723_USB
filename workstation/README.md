# RPBot Console trên PC Windows

Mở **RPBot Console** trên Desktop, bấm **Kết nối / Kiểm tra**, sau đó
**Mở RViz · Map** trong tab **Mapping**. Console giữ một node nhận ROS trên
PC và cập nhật trạng thái khoảng 5 lần/giây. Kiểm tra SSH/tìm lại phiên robot
chạy nền mỗi 15 giây khi bật **Tự kết nối lại**, không khóa nút thao tác.
Đóng Console sẽ dừng node nhận dữ liệu của riêng Console, giữ robot và RViz chạy.

## Chức năng

- Kiểm tra SSH và đọc cấu hình của phiên **mapping thật** đang chạy trên Pi.
  Tự lấy ROS domain và Fast DDS profile; không dùng nhầm demo domain 77 hoặc
  phiên phát lại bag dùng thời gian mô phỏng. PC dùng cổng DDS chuẩn và gửi
  discovery trực tiếp tới cổng tùy chỉnh trên Pi.
- Nhận thực tế `/map`, `/scan`, `/odom`, `/imu/data`, `/motor_fb`, `/motor_status`
  trên PC; kiểm tra TF `map → base_footprint`. SSH thành công không đồng nghĩa
  mapping đã sẵn sàng. Cảm biến không có bản tin mới trong 2 giây được báo cũ;
  khi luồng nhận trên PC thoát hoặc ngừng phản hồi, các trạng thái được đánh dấu mất dữ liệu.
- Mở RViz với sẵn map, LiDAR, robot và các lớp debug.
- Lưu map thành `.yaml` + `.pgm` trong `~/pi5-stm32-slam/maps/` trên Pi;
  từ chối ghi đè. Tên dùng chữ không dấu, số, dấu `-`, `_`.
- **Tạo map lại**: bỏ map/pose graph hiện tại trong bộ nhớ SLAM và dựng map mới
  từ các scan tiếp theo. Có xác nhận trước khi reset; các file map đã lưu được giữ.
  Nút dùng được với phiên mapping mở thủ công hoặc từ Console, không restart
  STM32, LiDAR, odometry hay dịch vụ mapping. Nếu service SLAM bị treo/không
  phản hồi, Console báo chưa xác nhận kết quả; không tự gửi reset lại.
- Xem log dịch vụ mapping, lưu log PC, bắt đầu/dừng phiên do dịch vụ Console quản lý.
- Tab **Bản đồ đã lưu** hiển thị danh sách, ngày lưu, kích thước, độ phân giải
  và ảnh xem trước của map trên Pi. Chọn map chỉ để xem/chuẩn bị navigation,
  không thay đổi map đang dựng.
- **Tải map về PC**: chọn thư mục đích, Console tạo thư mục con theo tên map
  chứa YAML/PGM, kiểm tra checksum và không ghi đè. YAML dùng đường ảnh tương đối,
  nên có thể chuyển cả thư mục sang máy khác.
- **Đổi tên** cập nhật đồng bộ YAML và PGM; bộ file cũ được giữ trong `.trash`.
  **Chuyển vào thùng rác** cần xác nhận và giữ file trong `maps/.trash/` trên Pi.
  Ảnh dùng chung với map khác được giữ. Nếu map đổi sau khi tải danh sách,
  thao tác bị từ chối để bạn tải lại danh sách trước.
- Tab **Debug** có số bản tin, tuổi dữ liệu và trạng thái từng topic cập nhật liên tục.
- Có cấu hình RViz Navigation với Nav2 panel, đặt vị trí, chọn đích, costmap và path.
  Nút chỉ mở khi phát hiện action server `/navigate_to_pose`.

## Cấu hình máy hiện tại

| Thành phần | Giá trị mặc định |
|---|---|
| Pi | `sharkagv@192.168.0.120` |
| ROS workspace trên Pi | `/home/sharkagv/pi5-stm32-slam` |
| PC WSL | `Ubuntu-24.04`, user `shark` |
| ROS | Jazzy, Fast DDS, domain lấy từ phiên đang chạy |
| Kết nối SSH | Khóa đã có trong WSL; dùng BatchMode, không lưu mật khẩu |
| Thiết lập/log PC | `%LOCALAPPDATA%\RPBotConsole\` |

Thư viện map trên Pi cần Python `yaml` (PyYAML), đã có trên máy hiện tại.
Giao diện trên Windows dùng tkinter; xem ảnh PNG không cần cài thêm thư viện ảnh.

Nếu IP Pi đổi, sửa IP trong Console rồi kiểm tra lại. Với máy mới, cần cài
ROS Jazzy + RViz trong WSL, cấu hình SSH key và chấp nhận host key của Pi
qua SSH tương tác một lần. Console không tự bỏ kiểm tra danh tính SSH.
WSL cần kết nối được DDS với Pi; máy hiện tại dùng mirrored networking.
Máy hiện tại đã có firewall rule UDP `7400–7649` chỉ từ IP Pi; bộ mở tận dụng
rule này, không sửa firewall. Nếu chuyển IP/máy, có thể kiểm tra bằng
`powershell -File workstation/configure_network.ps1`. Thêm `-Apply -PiAddress IP`
trong PowerShell Administrator để tạo rule chỉ cho UDP DDS từ địa chỉ Pi đó.

Tạo lại shortcut nếu chuyển thư mục repo:

```powershell
powershell -ExecutionPolicy Bypass -File .\workstation\install_shortcut.ps1
```

## Tự khởi động mapping trên Pi

Dịch vụ user `rpbot-mapping.service` chạy `scripts/run.sh mapping
allow_motor_enable:=false`, domain 0. Console không gửi lệnh bật motor hoặc
vận tốc. Việc cho phép robot chạy vẫn phụ thuộc firmware và đường điều khiển
hiện có của robot.

Cài và bật tự chạy ở lần boot sau, đồng thời kiểm tra dữ liệu:

```powershell
python .\workstation\check_connection.py --install-autostart
```

Thao tác cài chỉ **enable**, không khởi động lại phiên mapping đang chạy.
Nếu đã có đúng một phiên mapping thật, bấm **Bắt đầu mapping** sẽ kết nối vào
phiên đó và kiểm tra dữ liệu, không báo lỗi hoặc mở thêm phiên. Nút dừng chỉ
dừng `rpbot-mapping.service`; không tìm và giết phiên thủ công.
Nếu robot đang **Navigation**, Console kết nối vào phiên Navigation và báo rõ
chế độ đó. Nút **Bắt đầu mapping** không tự dừng Navigation hoặc chuyển chế độ.
Muốn dựng map mới, dừng phiên Navigation ở nơi đã khởi động rồi bắt đầu Mapping.
Nếu có nhiều phiên thật, thông báo liệt kê chế độ, PID và domain để phân biệt.
Không bật thêm micro-ROS Agent cùng dùng cổng STM32. Dịch vụ tự chạy cần
`Linger=yes` cho tài khoản Pi; bộ cài sẽ kiểm tra điều kiện này.

Log dịch vụ nằm trong user journal. Phiên mapping chạy bên ngoài dịch vụ
không có log trong journal của Console; xem log gốc trong `~/.ros/log/` trên Pi.
Đóng Console/RViz không dừng robot và không tự lưu map. Lần mapping mới tạo
map mới; map đã lưu không tự được nạp lại. Để tắt tự chạy, chạy trên Pi:

```bash
systemctl --user disable rpbot-mapping.service
```

## Navigation

Pi hiện đã có chế độ `navigation` trong launch. Console nhận diện được phiên
Navigation đang chạy, nhận map/cảm biến/TF và mở RViz Navigation khi phát hiện
action server Nav2. Kiểm tra GUI không gửi đích hoặc xác nhận robot đã chạy tới đích.
Console chưa có nút khởi động Navigation/chuyển chế độ; chọn map trong thư viện
chỉ để xem trước. Tạo map lại và lưu map SLAM được chặn khi đang Navigation.

## Kiểm tra cho người phát triển

```powershell
python -m unittest discover -s workstation/tests -v
python .\workstation\check_connection.py
python .\workstation\tests\smoke_gui.py --maps --download-check
```

`check_connection.py` mặc định chỉ đọc dữ liệu. `--save-map NAME` tạo bản đồ
mới trên Pi. Các kiểm tra này không phát lệnh di chuyển.
`smoke_gui.py` kiểm tra luồng liên tục, ảnh xem trước và tải file; bản tải thử nằm
trong `%LOCALAPPDATA%\RPBotConsole\verification-downloads\`.

## Khi môi trường đổi hoặc bản đồ bị sai

1. Nếu cần giữ map cũ, bấm **Lưu map trên Pi** với tên riêng trước.
2. Bấm **Kết nối / Kiểm tra**, rồi **Tạo map lại** và xác nhận.
3. Chờ dữ liệu map mới xuất hiện trên RViz, di chuyển robot để quét lại môi trường.
4. Lưu map mới với tên mới khi đã hài lòng.

**Bắt đầu mapping** mở phiên mới hoặc kết nối vào phiên mapping đang chạy. **Tạo map lại** xóa
map trong bộ nhớ của phiên đang chạy. Reset không sửa nguyên nhân gây sai map
như trượt bánh, sai TF, dữ liệu cảm biến hay tham số SLAM; nếu lỗi lặp lại cần
kiểm tra các dữ liệu đó. Nếu SLAM đã treo thì lệnh reset có thể không thực hiện được.
