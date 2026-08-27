# Báo Cáo Triển Khai Tự Động Hóa Khởi Động Robot
**Dự án:** `ros_h7_usb`
**Thành phần:** System deployment & Integration

---

## 1. Vấn đề hiện tại
Trong giai đoạn phát triển, để robot di chuyển được, người dùng phải làm thủ công các bước:
1. Bật nguồn Robot.
2. Cắm cáp USB vào máy tính nhúng (PC / Raspberry Pi).
3. Mở Terminal và gõ lệnh chạy `micro_ros_agent`.
4. Tìm và ghi nhớ cổng Serial ảo (vd: `/dev/ttyACM0`) thường xuyên bị thay đổi số nếu cắm lại.
Quá trình này tốn thời gian và không phù hợp với một sản phẩm thực tế cần tính năng **"Plug & Play"** (Bật nguồn là chạy).

## 2. Giải pháp Tự động hóa

Để giải quyết triệt để, thư mục `deploy/` đã được tạo ra, chứa các file script hệ thống giúp "hô biến" máy tính nhúng thành một trạm điều khiển tự động hoàn toàn. Cơ chế này kết hợp cực kỳ ăn ý với **Auto-Reconnect Logic** đã có sẵn bên trong mã nguồn FreeRTOS của STM32 (`freertos.c`).

### 2.1 Cố định tên cổng USB bằng UDEV
Sử dụng file `deploy/99-stm32-robot.rules`:
```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", SYMLINK+="stm32_robot"
```
**Chức năng:** Bất kể cắm vào lỗ USB nào, hay cắm chung với Lidar/Camera khác, hạt nhân Linux (Kernel) sẽ luôn luôn nhận diện và cấp cho STM32 một tên cố định duy nhất là `/dev/stm32_robot`. Điều này giúp các lệnh ROS 2 không bao giờ bị trượt cổng.

### 2.2 Tự động kích hoạt micro-ROS Agent bằng Systemd
Sử dụng file `deploy/micro_ros.service`:
Hệ điều hành của máy tính nhúng sẽ tự động chạy ngầm (background) quá trình kết nối ngay khi vừa boot xong mạng lưới ROS 2. 
Nếu cáp USB bị rút ra, service sẽ tự động báo lỗi và **liên tục thử khởi động lại sau mỗi 3 giây** (`Restart=always`, `RestartSec=3`) cho tới khi cáp USB được cắm lại thành công.

### 2.3 Phản ứng an toàn từ phía Firmware STM32
Khi cáp USB bị rút hoặc máy tính nhúng bị treo (reboot):
- STM32 (Hàm `StartDefaultTask`) sẽ ping thất bại 3 lần.
- Lập tức ngắt timeout (CMD_VEL_TIMEOUT_MS) -> Phanh động cơ (Vx=0, Wz=0) về vận tốc 0.
- Giải phóng các topic Pub/Sub (`destroy_entities`) và quay về vòng lặp chờ (`WAITING_AGENT`).
- Khi Systemd khởi động lại xong Agent trên PC, STM32 sẽ tự rà quét và kết nối liền mạch trở lại. Màn hình LCD tiếp tục hiển thị Data như chưa hề có cuộc chia ly.

## 3. Hướng dẫn sử dụng & Triển khai
Để cài đặt hệ thống tự động này vào máy tính nhúng (Raspberry Pi/Ubuntu/Jetson):

**Bước 1:** Copy thư mục `deploy/` từ máy Windows (hoặc pull từ Git) vào máy tính nhúng của Robot.
**Bước 2:** Mở terminal trên máy tính nhúng tại thư mục `deploy` và cấp quyền chạy:
```bash
chmod +x install_host.sh
```
**Bước 3:** Chạy Script tự động cài đặt với quyền Root:
```bash
sudo ./install_host.sh
```

**Hoàn tất!** Máy tính nhúng của bạn giờ đây đã sẵn sàng 100%. Bạn có thể tắt máy, bật lại và kiểm chứng tính năng tự động chạy.

