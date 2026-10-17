# Kiểm tra ngày 17/10/2026

Đã kiểm tra trên PC Windows / WSL Ubuntu-24.04 và Pi `192.168.0.120`.

- PC nhận trực tiếp map thật `109 × 120`, độ phân giải `0.05 m/ô`, scan,
  odometry, IMU, phản hồi và trạng thái motor. TF `map → base_footprint` hợp lệ.
- Chọn đúng phiên mapping thật domain 0 khi trên Pi đồng thời có các phiên
  phát lại bag dùng thời gian mô phỏng ở domain khác.
- RViz đã mở và hiển thị map, LiDAR, robot. Có log shader/TF lúc khởi động;
  bản đồ vẫn hiển thị. Không dùng riêng việc tiến trình RViz còn chạy để kết
  luận đường dữ liệu đã hoạt động.
- GUI đã đọc trạng thái thật, hiển thị motor/vận tốc và tình trạng Nav2.
- Lưu thành công `~/pi5-stm32-slam/maps/console_check_20261017.yaml` và `.pgm`.
  Thử lưu lại cùng tên bị từ chối với `Map already exists`.
- Đã tạo Desktop shortcut `RPBot Console.lnk` trỏ tới Python không có console.
- `rpbot-mapping.service` đã được enable, `Linger=yes`; unit qua
  `systemd-analyze --user verify`, wrapper qua `bash -n`. Phiên mapping thủ công
  PID 9573 được giữ nguyên; không thử reboot hoặc dừng phiên đang dùng.
- 10 unit test qua: cấu hình port DDS, chọn phiên thật, từ chối phiên nhập nhằng,
  SSH quoting, tên map, không mở trùng phiên, không dừng phiên bên ngoài Console,
  cài autostart không restart robot.
- Python compile, shell syntax và `git diff --check` qua.

Chưa kiểm chứng tự chạy sau một lần reboot thực tế. Chưa chạy Nav2 hoặc thử
đi tới đích. Console không phát lệnh bật motor hay di chuyển trong các kiểm tra.
Hướng dẫn sử dụng: [README](README.md).

## Nút Tạo map lại

- Đã thêm nút và xác nhận trước khi bỏ map chưa lưu. Reset chỉ gọi
  `/slam_toolbox/reset` với `pause_new_measurements=false`, không restart pipeline
  hoặc xóa file map đã lưu.
- Kiểm tra trực tiếp trên Pi: service reset khả dụng ở phiên mapping thật
  domain 0. Dùng `check_connection.py --check-reset`; thao tác chỉ tìm service,
  không gửi yêu cầu reset map đang dùng.
- Tổng 17 unit test qua, gồm hủy xác nhận, PID/domain thay đổi, từ chối phiên
  mô phỏng, reset thành công, kết quả lỗi và timeout không tự gửi lại.
- GUI vẫn nhận map, scan, odom và TF sau khi thêm nút. Chưa thực hiện reset
  bản đồ hiện tại trên robot.

## Bấm Bắt đầu khi mapping đã chạy

- Nút nay nhận lại phiên mapping thật hiện có, báo rõ bằng tiếng Việt và kiểm
  tra dữ liệu ngay; không gọi systemd start/restart hoặc reset map trong trường hợp này.
- Thử trực tiếp trên Pi: kết quả `already_running`, PID mapping 9573 giữ nguyên.
  GUI tiếp tục nhận map, scan, odom và TF.
- Tổng 19 unit test qua, gồm bấm bắt đầu nhiều lần và xử lý phiên không rõ ràng.

## Trạng thái liên tục và thư viện map

- Node nhận ROS duy trì trên WSL, gửi snapshot khoảng 5 Hz qua stdout; GUI chỉ
  giữ snapshot mới nhất. SSH kiểm tra metadata riêng, không khóa giao diện khi
  cập nhật trạng thái. Node nhận riêng thoát khi đóng Console.
- Kiểm tra GUI với robot thật: map `247×224`, scan, odom, IMU, motor và TF hợp lệ.
  21 snapshot liên tiếp; số snapshot vẫn tăng trong lúc tải map qua SSH.
- Đọc danh sách 3 map trên Pi và xem ảnh PGM trong tab Bản đồ đã lưu. Tải thật
  map `console_check_20261017` về thư mục kiểm tra trên PC; YAML/PGM qua checksum.
  Kết nối ROS vẫn duy trì trong lúc tải. Phiên robot PID 29405 được giữ nguyên.
- 31 unit test qua. Các thao tác đổi tên/thùng rác dùng file tạm trong test;
  chưa đổi tên hoặc chuyển map của người dùng trên Pi. Test bao gồm bảo vệ
  file dùng chung, map thay đổi, ghi đè, đường dẫn ngoài thư viện, PGM nhị phân,
  YAML lỗi, checksum, gộp snapshot và phát hiện luồng nhận đã thoát.
- Bố cục gồm Mapping, Bản đồ đã lưu, Debug và Navigation. Chọn map chưa nạp
  map đó vào SLAM hay Nav2. Chưa bổ sung điều khiển tay hoặc cấu hình Nav2 trên Pi.
- Kiểm tra mất/kết nối lại: chủ động dừng riêng subscriber trên PC, GUI báo
  mất luồng, sau đó mở subscriber mới và nhận lại đầy đủ dữ liệu. Tải map về
  PC vẫn thành công sau khi kết nối lại; không gửi lệnh reset/stop tới robot.
