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

## Nhận diện Navigation lúc 18:55 ngày 17/10/2026

- Lỗi 18:53 đến từ phiên `mode:=navigation`, không phải nhiều phiên Mapping.
  Đã mở rộng nhận diện cho cả Mapping và Navigation; loại các phiên dùng
  thời gian mô phỏng. Start khi Navigation chạy trả về `different_mode` và
  không gọi start/stop/restart/reset robot.
- GUI kiểm tra trực tiếp phiên Navigation PID 77922, domain 0: nhận map đã lưu
  `180×217`, scan, odom, IMU, motor, TF và action server Nav2; 16 snapshot liên tiếp.
  Chưa gửi goal hoặc xác nhận hành vi chạy tới đích.
- 35 unit test qua, thêm các trường hợp Navigation cùng replay, bấm Start
  khi Navigation đang chạy và chặn reset map trong chế độ Navigation.

## Các nút dừng tác vụ

- Thêm Dừng tạo map, Dừng Nav, Dừng tất cả và tab Tác vụ để dừng riêng các dòng
  được chọn, bao gồm launch robot và bag của dự án. Có xác nhận mục tiêu trước
  khi gửi tín hiệu; nút dừng không bị khóa bởi thao tác tải/lưu file thông thường.
- 47 unit test qua: PID bị tái sử dụng, domain/user khác, bag ngoài dự án,
  timeout wrapper không bị nhận trùng, systemd MainPID đổi, hủy xác nhận,
  tín hiệu chỉ tới mục tiêu và con đã nhận diện, kết quả còn sót được báo partial.
- Kiểm tra tiến trình thực trong Linux/WSL bằng fixture không nhập ROS hay mở
  phần cứng: dừng riêng Mapping, Navigation, bag và các tiến trình con; giữ
  nguyên tiến trình không được chọn. Dùng SIGINT/pidfd thật.
- Kiểm tra GUI offline bằng dữ liệu tác vụ giả: danh sách, hủy xác nhận và
  nút dừng vẫn khả dụng trong lúc tác vụ thông thường đang chạy.
- Pi tại `192.168.0.120` không kết nối được trong bước kiểm tra cuối (No route
  to host). Chưa gửi lệnh dừng phiên thật trên Pi; không dùng kết quả WSL để
  kết luận đã xác minh dừng chuyển động hay việc đóng bag trên robot thật.

## Nút Bắt đầu Nav ngày 17/10/2026

- Navigation có Bắt đầu Nav, Mở RViz Navigation và Dừng Nav. Start dùng map
  được chọn, revision và domain đã chọn; kiểm tra lại trên Pi trước khi mở phiên.
  Phiên đúng map/domain được dùng lại; phiên khác hoặc map đã sửa được báo rõ,
  không tự dừng/chuyển chế độ. Có chặn phát bag trong domain đã chọn.
- 58 test trên Windows: 57 qua, 1 kiểm tra Linux được bỏ qua có chủ ý. Cả 8
  backend test qua trong WSL với flock thật. Kiểm tra Linux xác thực unit bằng
  systemd-analyze, bash -n và chạy wrapper với runner giả (đường dẫn có dấu cách);
  kiểm tra đúng map/domain/allow_motor_enable:=false và từ chối phiên chạy trùng.
  Các lệnh systemctl start được mock; không khởi động robot để chạy test.
- GUI tkinter offline: nút hiện trên tab Navigation, hướng dẫn khi chưa chọn
  map, gửi đúng tên/revision/domain và mở lại các nút sau khi nhận kết quả.
  RViz Navigation được phép mở khi phiên Nav đã tồn tại để đặt vị trí ban đầu,
  dù action server chưa xuất hiện.
- Pi đã kết nối SSH trở lại trong lần này. Đã đọc scripts/run.sh và launch:
  hỗ trợ navigation + map, mặc định không cho bật motor. Chưa bấm Start Nav
  trên phiên robot thật, chưa gửi goal hoặc kiểm chứng di chuyển tới đích.

## Tự kết nối và phục hồi, kiểm thử offline ngày 17/10/2026

- Bỏ yêu cầu bấm Kiểm tra trước thao tác khi metadata cũ: GUI tự xác nhận và
  tiếp tục thao tác ban đầu. RViz có thể dùng luồng ROS mới khi SSH tạm lỗi.
  Thao tác đang chờ bị hủy nếu lỗi mạng, hết hạn, đổi địa chỉ hoặc đổi PID/phiên;
  không được gửi muộn sau khi tự kết nối lại. Lệnh thay đổi robot không tự thử lại.
- Kiểm tra nền không đổi dòng hoạt động khi đang ổn định. SSH thử lại với
  backoff 2/4/8/15 giây và gộp log lỗi lặp. Subscriber chết hoặc treo dù tiến
  trình còn sống được phục hồi riêng trên PC; chờ khởi động tối đa 15 giây,
  snapshot ngừng hơn 8 giây thì mở lại. Dữ liệu quá 2 giây vẫn được báo cũ.
- 74 unit test: 73 qua trên Windows, 1 kiểm tra Linux dành cho Nav được bỏ qua.
  Bao gồm lỗi mạng lặp, phục hồi, luồng treo còn sống, phản hồi từ IP cũ,
  không gửi reset sang PID mới, hết hạn và hủy thao tác khi xác nhận thất bại.
- Kiểm tra cửa sổ Tk offline với inspect/SSH/ROS được thay bằng fixture:
  hai lần mất mạng rồi tự nối lại; bấm Lưu map tự kiểm tra và gửi đúng một lần;
  luồng treo được mở lại mà không bấm Kiểm tra hoặc hiện hộp thoại. Fixture dùng
  thư mục settings/log tạm, không gửi yêu cầu tới Pi. Kiểm tra GUI Navigation
  và các nút dừng hiện có vẫn qua.
- Người dùng đã đổi địa điểm và hiện không có Pi để thử. Chưa kiểm chứng lần
  mất/phục hồi Wi-Fi thật với bản sửa này; không đổi cấu hình mạng, service,
  firmware hay tác vụ trên robot.

## RPBot Studio: giao diện và nhận diện ngày 17/10/2026

- Đã đọc trang và CSS hiện tại của getartcraft.com để lấy phong cách giấy xám
  `#f2f1ee`, mực `#101014`, nhấn xanh `#2d81ff`, chữ serif và nhãn mono. App vẫn
  là tkinter native trên Windows, không có yêu cầu cài framework hay web server.
- Chuyển điều hướng sang sidebar bảy trang; cấu hình kết nối vào trang riêng,
  khung trạng thái bên phải, log ở dưới. Nút dừng toàn bộ luôn ở đầu vùng làm
  việc. Nội dung và trạng thái có thể cuộn; thư viện map đổi cách xếp theo độ rộng.
- Logo/icon sao chép nguyên từ website chủ dự án; đã thêm Trần Nguyên Bình,
  website, GitHub và email công khai trong Giới thiệu. Nhận diện được đóng gói
  cùng app, không tải logo từ mạng và không phụ thuộc checkout website khi chạy.
- 73 unit test qua trên Windows, 1 Linux test được bỏ qua. Các kiểm tra GUI
  Start Nav, dừng tác vụ và tự kết nối/phục hồi offline đều qua sau đổi bố cục.
- Preview offline mở bảy trang tại hai kích thước 1420×920 và 1080×740. Kiểm tra
  sidebar/title/trạng thái chọn và độ rộng các nút đều qua. Chụp đủ 14 ảnh cửa
  sổ bằng PrintWindow; đã xem ảnh Mapping, Navigation, thư viện và cấu hình.
  Ảnh thể hiện trạng thái ngoại tuyến, không giả lập robot đang hoạt động.
- Pi hiện không có tại địa điểm người dùng; không gọi launch, reset, stop hoặc
  gửi đích để kiểm chứng giao diện. Không đổi firmware hay cấu hình mạng.

## Sửa font tiếng Việt ngày 17/10/2026

- Kiểm tra trực tiếp Windows GDI: Georgia thiếu 92 ký tự trong tập chữ Việt
  được kiểm tra, gồm nhiều chữ ơ/ư và nguyên âm có dấu. Mã nguồn vẫn là UTF-8,
  chuẩn NFC, không có ký tự replacement hoặc dấu tổ hợp bị tách.
- Đổi cả tiêu đề trang và tiêu đề panel từ Georgia sang Cambria italic, giữ
  kiểu serif của thiết kế. Tăng các nhãn tiếng Việt nhỏ lên 9 pt; nhãn sidebar
  xuống hai dòng để đủ chỗ và không cắt chữ cuối.
- Preview nay lấy font thật của các widget có tiếng Việt rồi kiểm tra glyph
  qua GetGlyphIndicesW. Cambria italic, Segoe UI thường/đậm và Consolas đều đủ
  chữ Việt. Bảy trang ở hai kích thước tiếp tục qua kiểm tra bố cục; đã xem ảnh
  mới của Mapping và Navigation. 73 unit test qua, 1 Linux test được bỏ qua.

## Ưu tiên trạng thái vận hành ngày 17/10/2026

- Thu gọn tiêu đề và bỏ panel trang trí lớn trên Mapping/Navigation. Đưa trạng
  thái từ cột phải phải cuộn lên bảng cố định ở đầu: kết nối, mode/map, Nav2,
  motor, từng cảm biến và vận tốc. Font Segoe UI đậm, tương phản cao; mỗi mức có
  màu và nhãn chữ riêng, không chỉ dựa vào màu. Nút dừng toàn bộ luôn nổi bật.
- Phân biệt chưa có dữ liệu với dữ liệu cũ, SSH mất nhưng ROS vẫn gửi, thiếu
  TF/vị trí với lỗi cảm biến. Vận tốc không giữ số cũ khi odom mất. Có action
  endpoint chỉ báo đã phát hiện server; không suy diễn Nav2 đã active.
- Cảnh báo ưu tiên mất liên lạc, lỗi motor/CAN và cảm biến cũ, rồi mới hướng
  dẫn đặt vị trí ban đầu. Việc đang thực hiện được hiển thị ngay trong dải
  thông báo. Chi tiết dài ở Debug, tự chạy sau boot ở Kết nối. Log vẫn có thanh
  kéo chỉnh chiều cao; thêm màu lỗi/cảnh báo/thành công và tăng tương phản chữ.
- 86 unit test: 85 qua trên Windows, 1 Linux test được bỏ qua. Các trường hợp
  mới kiểm tra màu trạng thái dựa trên bằng chứng, mất/già dữ liệu, timeout,
  lỗi CAN, thiếu TF, thông tin Nav2 và không thay đổi telemetry gốc.
- Preview cả bảy trang ở hai kích thước qua; thêm sáu ảnh trạng thái với
  telemetry giả, ghi rõ DỮ LIỆU GIẢ trong tiêu đề. Đã xem ảnh lỗi CAN, chờ TF,
  ngoại tuyến và cửa sổ nhỏ. Kiểm tra GUI tự phục hồi, Start Nav, dừng tác vụ
  đều qua với backend giả. Không gửi lệnh tới Pi trong lần chỉnh này.

## Ping hostname và mDNS ngày 17/10/2026

- Thêm Ping thiết bị và nút Dùng sharkAGV.local ở đầu trang Kết nối, nhìn thấy
  ngay tại cả 1420×920 và 1080×740. Nút điền nhanh lưu host sharkAGV.local và
  user sharkagv; cấu hình IP cũ vẫn đọc được. Cài mới mặc định dùng hostname.
- Ping chạy trên worker, tìm IPv4 qua Windows rồi thử bộ phân giải WSL nếu
  cần. Chỉ đếm Echo Reply có TTL; không báo xanh nhầm khi ping Windows exit 0
  nhưng trả Destination host unreachable. Hiện IP/số phản hồi bằng màu và chữ;
  log giữ đầu ra đầy đủ. Ping thất bại không cản kiểm tra SSH.
- Kết quả cũ bị xóa khi sửa cấu hình; phản hồi từ host trước đó không cập nhật
  trạng thái mới. Ping không thay đổi metadata/session hay gửi thao tác robot.
- SSH dùng IPv4 đã phân giải nhưng giữ hostname gốc để chọn SSH config và xác
  thực host key. DDS dùng địa chỉ từ worker metadata, không phân giải mDNS
  trong vòng lặp Tk. Metadata thấy hostname đổi IP sẽ dựng lại subscriber trên
  PC với peer mới, giữ nguyên phiên trên robot.
- 103 unit test: 102 qua, 1 Linux-only được bỏ qua trên Windows. Smoke GUI
  hostname/ping kiểm tra nền không khóa Tk, lưu preset, đủ/mất phản hồi, lỗi DNS,
  đổi host giữa chừng, màu hiển thị và vị trí nút ở hai kích thước đều qua.
  Smoke tự phục hồi kết nối vẫn qua. Ảnh ping-fixture ghi rõ DỮ LIỆU PING GIẢ.
- Đã chạy ping thật trên Windows tới loopback 127.0.0.1: 2/2 phản hồi. Các kiểm
  tra hostname robot, SSH và DDS dùng fixture; Pi không có tại địa điểm hiện
  tại nên chưa xác nhận sharkAGV.local trên mạng robot thực. Không sửa mDNS,
  firewall hay danh sách SSH host key, không gửi lệnh điều khiển tới Pi.

## Tên thiết bị và tài khoản tùy người dùng ngày 17/10/2026

- Thay thiết kế điền nhanh ở trên bằng hai ô Thiết bị / IP và Tài khoản SSH
  ngay đầu trang. Bỏ nút Dùng sharkAGV.local và các hằng host/user cá nhân.
  Lần cài mới để trống cả hai ô, mở trang Kết nối; cấu hình đã lưu được giữ.
- Ping chỉ kiểm tra hostname/IP, không yêu cầu nhập tài khoản SSH trước.
  Kết nối mới xác thực đủ cấu hình và lưu theo thông tin người dùng đã nhập.
- 106 unit test: 105 qua, 1 Linux-only bỏ qua. Smoke GUI với rover-2.local và
  operator xác nhận nhập/lưu tùy ý, ping khi user còn trống, phản hồi cũ bị bỏ,
  màu kết quả và hai ô nhập/nút đều thấy ở 1420×920, 1080×740. Smoke tự kết nối
  lại và preview bảy trang qua; glyph tiếng Việt đủ, không có nút bị cắt.
- Chỉ dùng dữ liệu giả, không liên lạc robot hay sửa cấu hình thiết bị đã lưu
  của người dùng trong lần điều chỉnh này.

## EXE độc lập ngày 17/10/2026

- Build Windows x64 one-file/windowed bằng PyInstaller 6.22.3, Python 3.11.9;
  `dist/RPBotStudio.exe` 12.235.052 byte (11,7 MiB). Gắn icon cá mập, ProductName
  RPBot Studio, phiên bản 0.1.0 và CompanyName Trần Nguyên Bình vào PE resources.
- Spec có danh sách rõ chín file tài nguyên/helper ROS/RViz/logo/icon; SHA256
  nội dung trong EXE và thư mục giải nén khớp từng byte với nguồn. Python,
  Tcl/Tk và các module giao diện có trong gói; không kèm settings, SSH hay log.
- Smoke chạy chính EXE từ thư mục chỉ có một file, có dấu tiếng Việt/khoảng
  trắng, với PATH chỉ còn Windows System32 và LOCALAPPDATA trống. GUI mở đúng
  thương hiệu, trang nhập thiết bị; đóng sạch, không tự tạo cấu hình robot hay
  telemetry. Đã chụp/xem cửa sổ thật, không chỉ chạy lại chương trình Python.
- WSL Ubuntu-24.04 đọc được đủ chín tài nguyên ở thư mục giải nén; SHA256 khớp,
  các helper Python qua AST parse, script ROS qua bash -n và không chứa CRLF.
  Đây là kiểm tra đường dẫn/resource, không phải xác nhận mapping/nav thật.
- 106 unit test vẫn qua với 105 thành công và 1 Linux-only bỏ qua. Bổ sung
  stdin DEVNULL cho lệnh con không cần đầu vào khi chạy windowed; payload SSH
  vẫn đi qua stdin pipe như trước. Shortcut ưu tiên EXE, còn nhánh chạy nguồn.
- Không cài hoặc gộp WSL/ROS vào EXE; máy dùng vẫn cần môi trường này cho
  telemetry/RViz. Không gửi lệnh đến robot trong quá trình kiểm tra gói.
