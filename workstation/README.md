# RPBot Studio trên PC Windows

## Mở bằng một file EXE

Bản Windows x64 nằm tại **`dist/RPBotStudio.exe`**. Nhấp đúp file này để chạy;
có thể copy riêng EXE sang thư mục hoặc PC khác. Bản đóng gói đã chứa Python,
Tcl/Tk, logo/icon, giao diện và các file hỗ trợ ROS/RViz; không cần cài Python
trên Windows hoặc mang theo thư mục mã nguồn. Cách đặt tài nguyên theo `__file__`
theo [hướng dẫn PyInstaller](https://pyinstaller.org/en/stable/runtime-information.html#using-file).

Các chức năng ROS và RViz vẫn cần **WSL + ROS 2 Jazzy/RViz** trên PC, cùng SSH key
đến robot. EXE không cài sẵn môi trường Linux/ROS. Hostname/IP, tài khoản SSH,
distro và user WSL đều sửa trong **Kết nối** như khi chạy mã nguồn.
Cấu hình/log tiếp tục ở `%LOCALAPPDATA%\RPBotConsole\`, giữ được sau khi đổi
thư mục hoặc thay EXE bằng bản mới; dữ liệu máy người dùng không được đóng gói.

`install_shortcut.ps1` ưu tiên EXE khi có `dist/RPBotStudio.exe`; nếu chưa build,
shortcut dùng Python như trước. Tạo/cập nhật biểu tượng **RPBot Console**:

```powershell
powershell -ExecutionPolicy Bypass -File .\workstation\install_shortcut.ps1
```

Build lại trên Windows khi sửa mã nguồn, logo hay file RViz:

```powershell
python -m pip install -r workstation/requirements-build.txt
powershell -ExecutionPolicy Bypass -File .\workstation\build_exe.ps1
python -X utf8 workstation/tests/smoke_exe.py --wsl
```

Build sử dụng danh sách file cho phép trong `RPBotStudio.spec`; không kèm thiết
lập PC, SSH key, log, bản đồ riêng hoặc firmware. Smoke test copy chỉ EXE sang
thư mục có dấu/khoảng trắng, bỏ Python khỏi PATH, mở với cấu hình trống, kiểm
tra tài nguyên và đóng bình thường. `--wsl` chỉ kiểm tra WSL đọc được các file
giải nén và cú pháp helper; không liên lạc robot hay khởi chạy ROS/động cơ.

## Giao diện

Chọn **Tiếng Việt / English** trên thanh đầu để đổi ngôn ngữ ngay khi đang mở.
Lựa chọn được lưu riêng cho mỗi app và dùng lại lần sau; cấu hình cũ mặc định
tiếng Việt. Nút, tiêu đề, trạng thái, bảng chẩn đoán và hộp thoại đổi theo ngôn ngữ.
Tên thiết bị, tên map, đường dẫn, lệnh và thông tin tác giả được giữ nguyên.
Nhật ký thao tác mới dùng ngôn ngữ hiện tại, các dòng cũ giữ nội dung đã ghi.
Đổi ngôn ngữ không nối lại SSH/ROS hay gửi lệnh robot. Danh mục dịch dùng chung
với DAPLink Flasher tại `ui_translations.py`, lớp hiển thị tại `ui_language.py`.

Giao diện desktop giữ nền giấy xám, chữ đen và điểm nhấn xanh dương lấy cảm hứng
từ [ArtCraft](https://getartcraft.com/), đồng thời ưu tiên thông tin vận hành:
tiêu đề gọn, thao tác trực tiếp và trạng thái robot luôn hiển thị ở đầu màn hình.
Icon app là robot nhìn từ trên với vòng quét LiDAR, màu đen/xanh của Studio,
vẽ bằng `tools/make_app_icons.py` và đóng gói trong `assets/`. Tên Trần Nguyên
Bình và thông tin trong **Giới thiệu** lấy từ website của chủ dự án tại
`P:\web\website-source`, lưu ở `branding.py`; chạy app không phụ thuộc website.

Tiêu đề và nội dung dùng Segoe UI, số đo và nhật ký dùng Consolas.
Đã kiểm tra các font đang vẽ tiếng Việt có đủ ký tự Việt trên Windows; không
dùng Georgia vì bản font trên máy thiếu nhiều ký tự có dấu. Preview kiểm tra
glyph thật qua Windows GDI trước khi chụp ảnh để tránh font thay thế bị ghép lẫn.

Menu bên trái gồm **Mapping**, **Navigation**, **Bản đồ đã lưu**, **Tác vụ**,
**Debug**, **Kết nối** và **Giới thiệu**. Bảng trạng thái cố định ở đầu, log ở dưới.
Sửa hostname / IP / user / WSL trong **Kết nối**. Các vùng nội dung có thể cuộn khi cửa sổ
nhỏ; bảng thư viện và ảnh map tự đổi từ hai cột sang xếp dọc.
Vùng nhật ký có chiều cao mặc định lớn hơn; kéo thanh phân cách ngay phía trên
**Nhật ký thao tác** lên/xuống để dành thêm hoặc bớt chỗ cho log.

Bảng trạng thái có **Kết nối**, **Chế độ / map**, **Nav2**, **Motor**, các ô
**LiDAR / Odom / IMU / Motor FB** và vận tốc `v`, `ω`. Màu đi cùng nhãn chữ:
xanh lá là dữ liệu đã xác nhận, vàng là đang chờ/cần xử lý, đỏ là mất kết nối,
dữ liệu cũ hoặc lỗi; xanh dương là thông tin đã phát hiện, xám là chưa có dữ liệu.
Khi odom cũ, số đo vận tốc đổi thành `—`. Có action server không được coi là
Nav2 đã active. Cảnh báo ngay dưới bảng nêu nguyên nhân và bước xử lý tiếp theo.
Thông tin đầy đủ ở **Debug**, trạng thái tự chạy sau boot ở **Kết nối**.
Nhật ký dùng chữ đậm màu hơn và đánh dấu lỗi/cảnh báo/thành công bằng màu.

Mở **RPBot Console** trên Desktop: GUI tự kết nối tới Pi và nhận diện phiên
Mapping/Navigation đang chạy. Sau đó dùng **Mở RViz · Map** hoặc tab **Navigation**.
Console giữ một node nhận ROS trên PC và cập nhật trạng thái khoảng 5 lần/giây.
Không cần bấm **Kết nối / Kiểm tra** trước mỗi thao tác; nút này vẫn có để kiểm tra ngay.
Đóng Console sẽ dừng node nhận dữ liệu của riêng Console, giữ robot và RViz chạy.

Khi bật **Tự kết nối / phục hồi** (mặc định), kiểm tra SSH chạy nền mỗi 30 giây
khi đã có phiên, mỗi 5 giây khi Pi đang rảnh. Mất kết nối thì thử lại sau
2, 4, 8 rồi tối đa 15 giây; lỗi lặp được gộp log. Trạng thái ổn định không bị
thay bằng “Đang kiểm tra” ở mỗi lần kiểm tra nền. ROS vẫn được giám sát riêng
khi SSH tạm mất kết nối.

Mở RViz dùng được với phiên đã biết khi ROS vẫn gửi dữ liệu mới, dù SSH tạm lỗi.
Lưu map / Tạo map lại tự xác nhận lại phiên nếu thông tin đã cũ rồi tiếp tục
thao tác; không yêu cầu bạn bấm Kiểm tra trước. Nếu mất mạng hoặc PID/phiên đổi
trong lúc xác nhận, thao tác đó được hủy và có thông báo; không tự gửi lại khi
mạng trở lại. Các lần nối lại nền chỉ phục hồi giám sát, không mở lại tác vụ robot.

Subscriber trên PC thoát hoặc không gửi snapshot hơn 8 giây sẽ tự mở lại;
lần khởi động đầu có khoảng chờ 15 giây. Sau 2 giây không có snapshot mới,
GUI đã báo dữ liệu cũ. Tắt tùy chọn tự phục hồi nếu muốn quản lý thủ công.

## Ping và kết nối theo tên thiết bị

Trong **Kết nối**, nhập **Thiết bị / IP** và **Tài khoản SSH** của robot muốn dùng.
Hai ô này luôn ở đầu trang, ngay trước các nút Ping / Kết nối. Ví dụ với thiết bị
có SSH config như sau, nhập `rover-2.local` và `operator`:

```sshconfig
Host rover-2.local
  HostName rover-2.local
  User operator
```

Tên và tài khoản trên chỉ là ví dụ; mỗi người nhập giá trị của thiết bị mình.
Không cần tạo file SSH config riêng. Bấm **Kết nối ngay** để lưu cấu hình trên
PC; tự kết nối cũng lưu khi các ô đã hợp lệ. Cấu hình trước đây được giữ nguyên
đến khi bạn sửa. Lần cài mới để trống tên thiết bị/tài khoản và mở trang Kết nối,
không tự liên lạc với một thiết bị được đặt tên sẵn.

**Ping thiết bị** tìm IPv4 hiện tại rồi gửi hai gói ICMP, hiển thị IP và số phản
hồi ngay dưới nút. Xanh lá là đủ hai phản hồi, vàng là mất gói hoặc không phản
hồi, đỏ là lỗi phân giải tên hoặc không chạy được phép kiểm tra. Chi tiết có
trong Nhật ký thao tác. Chỉ cần nhập host để ping, chưa cần tài khoản SSH.
Ping chạy nền, không khóa giao diện; đổi cấu hình sẽ
xóa kết quả cũ và không nhận nhầm phản hồi đang chờ của địa chỉ trước đó.

Ping và xác thực SSH là hai phép kiểm tra riêng: không có ICMP vẫn có thể bấm
**Kết nối ngay**. Tự kết nối lại vẫn hoạt động, không cần bấm Ping trước mỗi
thao tác. Tên được phân giải lại khi kiểm tra/kết nối; app không thay tên đã
lưu bằng IP hoặc tự quay về địa chỉ IP cũ.

Studio ưu tiên bộ phân giải tên trên Windows, rồi thử `getent ahostsv4` trong
WSL đã chọn. Với `.local`, IPv4 tìm được dùng cho SSH và DDS trên PC; SSH vẫn
kiểm tra host key theo tên thiết bị. Cách này xử lý trường hợp Windows tìm
được tên nhưng WSL chưa phân giải được mDNS. Cấu hình mDNS trong WSL có các
điều kiện riêng được mô tả trong [tài liệu Microsoft](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting#resolve-local-names-in-wsl).
Robot cần ở mạng có thể phân giải tên thiết bị đã nhập;
nếu mạng chặn mDNS, có thể nhập IP trực tiếp vào cùng ô.

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
- Xem log dịch vụ Mapping/Navigation, lưu log PC, bắt đầu/dừng phiên do dịch vụ Console quản lý.
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
- **Dừng tạo map**, **Dừng Nav**, **Dừng tất cả tác vụ robot** và tab **Tác vụ**
  dừng được cả phiên mở ngoài Console. Trước khi dừng có danh sách tác vụ/PID
  để xác nhận; các nút dừng vẫn dùng được khi Console đang tải hoặc lưu file.
- **Bắt đầu Nav** dùng map được chọn trong thư viện, kiểm tra map chưa thay đổi
  và từ chối mở trùng phiên. Nút không tự chuyển chế độ hoặc bật motor.
- Có cấu hình RViz Navigation với Nav2 panel, đặt vị trí, chọn đích, costmap và path.
  Mở được khi có phiên Navigation, kể cả lúc Nav2 đang chờ vị trí ban đầu.

## Cấu hình máy hiện tại

| Thành phần | Cấu hình |
|---|---|
| Pi | Hostname/IP và tài khoản SSH do người dùng nhập; không có thiết bị mặc định |
| ROS workspace trên Pi | `~/pi5-stm32-slam` trong home của tài khoản SSH đã nhập |
| PC WSL | `Ubuntu-24.04`, user `shark` |
| ROS | Jazzy, Fast DDS, domain lấy từ phiên đang chạy |
| Kết nối SSH | Khóa đã có trong WSL; dùng BatchMode, không lưu mật khẩu |
| Thiết lập/log PC | `%LOCALAPPDATA%\RPBotConsole\` |

Thư viện map trên Pi cần Python `yaml` (PyYAML), đã có trên máy hiện tại.
Giao diện trên Windows dùng tkinter; xem ảnh PNG không cần cài thêm thư viện ảnh.

Nếu dùng tên thiết bị, Console tìm lại IP khi kết nối. Nếu nhập IP trực tiếp và
IP Pi đổi, sửa IP trong Console; GUI tự kết nối theo địa chỉ mới. Với máy mới, cần cài
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
phiên đó và kiểm tra dữ liệu, không báo lỗi hoặc mở thêm phiên.
Nếu robot đang **Navigation**, Console kết nối vào phiên Navigation và báo rõ
chế độ đó. Nút **Bắt đầu mapping** không tự dừng Navigation hoặc chuyển chế độ.
Muốn dựng map mới, bấm **Dừng Nav**, chờ xác nhận kết thúc rồi **Bắt đầu mapping**.
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
Navigation đang chạy và nhận map/cảm biến/TF. Để bắt đầu từ GUI:

1. Nếu Mapping đang chạy, lưu map cần giữ, bấm **Dừng tạo map** và chờ dừng xong.
2. Trong **Bản đồ đã lưu**, tải lại danh sách và chọn map muốn dùng.
3. Sang **Navigation**, bấm **Bắt đầu Nav**. Theo dõi trạng thái và xem Debug nếu lỗi.
4. Bấm **Mở RViz · Navigation**, dùng **2D Pose Estimate** để đặt vị trí/hướng
   hiện tại của robot. Chờ localization/Nav2 sẵn sàng trước khi chọn đích.

Start dùng domain ở tab **Tác vụ** (mặc định 0, tự cập nhật khi kết nối vào phiên).
Nếu đúng phiên Nav/map/domain đang chạy, Console dùng lại phiên đó. Nếu đang có
phiên khác hoặc đang phát bag trong domain đã chọn, cần dừng trước; Console
không tự thay map hoặc dừng phiên đang chạy. Chọn map trong thư viện chỉ đổi
lựa chọn; bấm **Bắt đầu Nav** mới yêu cầu nạp map khi mở phiên mới.

Console tạo `rpbot-navigation.service` trong systemd user trên Pi và khởi động
`scripts/run.sh navigation map:=... allow_motor_enable:=false`. Dịch vụ Nav không
được bật tự chạy sau boot, không tự restart và vẫn dùng được với **Dừng Nav**.
Nút Start không gửi goal hoặc lệnh bật motor. Tạo map lại và lưu map SLAM được
chặn khi đang Navigation. Kiểm tra GUI không xác nhận robot đã chạy tới đích.

## Dừng tác vụ

| Nút | Phạm vi |
|---|---|
| Dừng tạo map, tab Mapping | Các phiên Mapping trong domain đã chọn, cùng tiến trình con |
| Dừng Nav, tab Navigation | Toàn bộ phiên Navigation trong domain đã chọn, cùng tiến trình con |
| Dừng tất cả tác vụ robot | Launch robot và ghi/phát bag của dự án trong domain đã chọn |
| Dừng tác vụ đã chọn, tab Tác vụ | Chỉ các dòng được chọn; giữ Ctrl để chọn nhiều dòng |

Domain mặc định là 0; có thể đổi trong tab **Tác vụ**. Khi kết nối vào phiên robot,
Console dùng domain của phiên đó. Bấm **Tải lại tác vụ** để xem danh sách mới.
Trước khi dừng, Console liệt kê chế độ/tên bag và PID. Hủy xác nhận sẽ không dừng gì.

Dừng Mapping sẽ bỏ map chưa lưu trong bộ nhớ, nên lưu trước nếu cần giữ.
Các file map/bag đã lưu không bị xóa. Dừng Navigation là tắt cả phiên launch,
bao gồm các node cảm biến/agent được phiên đó mở; sau đó có thể khởi động Mapping.
Đây là dừng tác vụ ROS; quyền điều khiển RC và phần cứng vẫn độc lập.

Console gửi SIGINT để ROS/bag đóng dữ liệu, chờ tiến trình con thoát và chỉ dùng
SIGTERM với các tiến trình đã nhận diện còn sót. PID và thời điểm khởi tạo được
kiểm tra lại, dùng pidfd để tránh gửi nhầm sang PID được tái sử dụng. Phiên do
systemd quản lý được dừng qua service nếu MainPID khớp. Chương trình khác trên
Pi và các domain khác không thuộc phạm vi dừng. Dừng service không tắt tự chạy
ở lần boot sau. Nếu còn PID chưa thoát, GUI báo chưa dừng hết.

## Kiểm tra cho người phát triển

```powershell
python -m unittest discover -s workstation/tests -v
python .\workstation\check_connection.py
python .\workstation\tests\smoke_gui.py --maps --download-check
python .\workstation\tests\smoke_stop_ui.py
python .\workstation\tests\smoke_navigation_ui.py
python .\workstation\tests\smoke_connection_ui.py
python .\workstation\tests\preview_studio.py
python .\workstation\tests\preview_studio.py --states
wsl -d Ubuntu-24.04 -u shark -- python3 /mnt/p/Prj_STM32/Microros_H723_USB/workstation/tests/linux_stop_smoke.py
wsl -d Ubuntu-24.04 -u shark -- python3 -m unittest discover -s /mnt/p/Prj_STM32/Microros_H723_USB/workstation/tests -p test_start_navigation.py -k NavigationBackendTests -v
```

`check_connection.py` mặc định chỉ đọc dữ liệu. `--save-map NAME` tạo bản đồ
mới trên Pi. Các kiểm tra này không phát lệnh di chuyển.
`smoke_gui.py` kiểm tra luồng liên tục, ảnh xem trước và tải file; bản tải thử nằm
trong `%LOCALAPPDATA%\RPBotConsole\verification-downloads\`.
`preview_studio.py` kiểm tra cả bảy trang ở `1420×920` và `1080×740`, chụp riêng
cửa sổ bằng Windows PrintWindow vào `%LOCALAPPDATA%\RPBotConsole\studio-preview\`.
Kiểm tra ảnh cần Pillow; app thường ngày không cần Pillow. Preview chạy offline
với thư mục settings/log tạm và không kết nối Pi.
`--states` bổ sung ảnh các trạng thái ROS đang nhận, thiếu vị trí/TF, lỗi CAN,
odom cũ, mất luồng và ngoại tuyến. Ảnh này ghi rõ **DỮ LIỆU GIẢ**; chỉ dùng để
kiểm tra giao diện, không phải kết quả chạy robot thật.

## Khi môi trường đổi hoặc bản đồ bị sai

1. Nếu cần giữ map cũ, bấm **Lưu map trên Pi** với tên riêng trước.
2. Bấm **Tạo map lại** và xác nhận. Console tự kiểm tra phiên nếu cần.
3. Chờ dữ liệu map mới xuất hiện trên RViz, di chuyển robot để quét lại môi trường.
4. Lưu map mới với tên mới khi đã hài lòng.

**Bắt đầu mapping** mở phiên mới hoặc kết nối vào phiên mapping đang chạy. **Tạo map lại** xóa
map trong bộ nhớ của phiên đang chạy. Reset không sửa nguyên nhân gây sai map
như trượt bánh, sai TF, dữ liệu cảm biến hay tham số SLAM; nếu lỗi lặp lại cần
kiểm tra các dữ liệu đó. Nếu SLAM đã treo thì lệnh reset có thể không thực hiện được.
