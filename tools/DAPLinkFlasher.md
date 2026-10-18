# DAPLink Flasher

Nhấp đúp **`tools/DAPLinkFlasher.exe`** để mở. Bản Windows x64 chứa Python,
Tcl/Tk, hidapi và OpenOCD; có thể mang riêng EXE sang máy khác. Không cần cài
Python hay STM32CubeIDE để mở app và dùng bộ OpenOCD đi kèm.

Icon app là chip MCU có tia sét (nạp flash) trên nền xanh của app, vẽ bằng
`tools/make_app_icons.py`; màu giấy/đen/xanh đồng bộ với RPBot Studio. Chế độ sáng/tối vẫn theo thiết lập `theme` đã lưu và Windows.
Font Segoe UI/Consolas đã kiểm tra glyph tiếng Việt bằng Windows GDI.

- Bộ chọn **Tiếng Việt / English** trên thanh đầu đổi ngôn ngữ ngay, lưu cho lần
  mở sau, kể cả khi đang nạp. Nó không thay đổi job, trạng thái nút Dừng nạp,
  giao tiếp/xung hay lựa chọn kiểm tra động cơ. Cấu hình cũ mặc định tiếng Việt.
  Các dòng nhật ký đã ghi được giữ; thông báo mới theo ngôn ngữ đang chọn,
  đầu ra OpenOCD giữ nguyên. CLI tiếp tục hoạt động như trước.
- Nút **Thông tin tác giả** ở góc trên mở tên Trần Nguyên Bình và lĩnh vực
  Cơ điện tử / Hệ thống điều khiển / Robotics. Bấm Website, GitHub hoặc Email để mở
  trang cá nhân, kho mã hoặc trình soạn email. Cửa sổ này có thể đóng bằng Escape
  và vẫn cho phép thao tác nút Dừng nạp ở cửa sổ chính.
- Đầu cửa sổ luôn có kết nối mạch nạp, firmware đã chọn và tùy chọn kiểm tra
  động cơ. Mỗi trạng thái có màu và nhãn chữ; có file không đồng nghĩa chip
  đã được nạp/xác minh. Ô kiểm tra động cơ mô tả lựa chọn cấu hình, không suy
  diễn trạng thái vật lý của động cơ.
- Bên trái chọn probe, file `.elf`/`.hex`/`.bin`, giao tiếp và xung SWD.
  Cột này cuộn được khi cửa sổ nhỏ; Nạp firmware / Dừng nạp luôn nằm cố định.
  Trong khi nạp, các ô thay file/thông số tạm khóa, nút dừng vẫn hoạt động.
- Bên phải có phần trăm lớn, số byte, tốc độ/thời gian và trạng thái từng
  sector. Nhật ký dùng chữ rõ, màu theo mức; kéo thanh ngang phía trên để
  tăng/giảm chiều cao. Có thể bật/tắt phần log OpenOCD.

Thiết lập tiếp tục ở `%APPDATA%\dap_flasher\settings.json`. Bản giao diện này
không đổi các lựa chọn đã lưu, bộ phân tích image, lập kế hoạch sector, CRC,
phục hồi phiên hay lệnh kiểm tra động cơ. Lệnh kiểm tra nếu dùng WSL/SSH vẫn
cần môi trường tương ứng như trước.

## Bản Linux

DAPLink Flasher nằm chung gói Linux với RPBot Studio
(`dist/linux/RPBot-Linux-x86_64.tar.gz`, build bằng `packaging/linux/build.sh`,
cách cài xem `workstation/README.md`). Khác bản Windows:

- Không gói sẵn OpenOCD: cài `sudo apt install openocd`. App tìm theo thứ tự
  `OPENOCD`, `openocd` trên PATH, rồi STM32CubeIDE ở `/opt/st/stm32cubeide_*`
  hoặc `~/st/stm32cubeide_*`.
- Quyền USB: `./install.sh --udev` cài `60-dap-flasher.rules` (Horco faed:4873
  và Arm DAPLink 0d28:0204) rồi cắm lại mạch nạp; không cần chạy app bằng sudo.
- Cấu hình ở `~/.config/dap_flasher/settings.json`. Tùy chọn đóng OpenOCD khác
  dùng `pkill -x openocd` thay cho `taskkill`.
- Quy tắc an toàn giữ nguyên: kiểm tra động cơ mặc định bật, chỉ dừng chip khi
  MOTORS_ENABLED đã tắt.

## Đóng gói lại

Trên máy build cần Python, PyInstaller 6.22.3, hidapi 0.15.0 và OpenOCD kèm
scripts. Spec dùng cùng hàm tìm OpenOCD với app: biến môi trường `OPENOCD`,
`OPENOCD_SCRIPTS`, hoặc bản cài STM32CubeIDE. Icon/logo lấy từ `tools/assets/`,
sau khi build đã nằm trong EXE. Sửa hình trong `tools/make_app_icons.py` rồi chạy
lại script đó (cần Pillow) trước khi build.

```powershell
python -m pip install PyInstaller==6.22.3 hidapi==0.15.0
powershell -ExecutionPolicy Bypass -File tools/build_dap_flasher.ps1
python -B -X utf8 tools/tests/smoke_dap_flasher_ui.py
python -B -X utf8 tools/tests/smoke_dap_flasher_exe.py
Copy-Item -LiteralPath tools/build/release/DAPLinkFlasher.exe -Destination tools/DAPLinkFlasher.exe -Force
```

Đóng app trước khi thay EXE. Build tạo bản chờ kiểm tra trong `tools/build/`,
giữ EXE đang dùng đến khi thực hiện bước copy cuối.

## Kiểm tra giao diện ngày 17/10/2026

- So sánh mã phần xử lý trước vùng GUI và CLI với HEAD: nội dung khớp.
- Native UI qua ở sáng/tối, 1280×860 và 1060×720: glyph tiếng Việt, tình trạng
  chưa có link, file hợp lệ/lỗi, đang ghi, mất link, hoàn tất; dừng job giả và
  giữ lựa chọn cấu hình. Ảnh kiểm thử ghi rõ DỮ LIỆU GIẢ trên tiêu đề.
- Thông tin tác giả khớp `workstation/branding.py`; ba liên kết gọi đúng địa chỉ
  (trình duyệt được giả lập), mở lại dùng cùng cửa sổ, Escape/Đóng hoạt động và
  không khóa nút Dừng nạp hay khởi chạy OpenOCD.
- Chạy EXE thật từ thư mục riêng có dấu/khoảng trắng với PATH chỉ có System32:
  cửa sổ mở/đóng sạch, logo và module hid có trong gói. Toàn bộ 39 file OpenOCD
  đi kèm giống bản EXE trước; gọi `openocd --version` thành công từ bộ giải nén.
- Chưa nạp lên STM32 hay mở phiên SWD để kiểm tra thay đổi giao diện. Ảnh và
  file build có thể xóa sau khi đã thay EXE, mã smoke/build giữ để dùng lại.
