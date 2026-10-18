RPBot Studio + DAPLink Flasher cho Linux (Ubuntu 24.04 trở lên, x86_64)

Cài cho người dùng hiện tại:
    ./install.sh            chép app vào ~/.local/share/rpbot, thêm vào menu ứng dụng
    ./install.sh --udev     thêm quyền mở mạch nạp CMSIS-DAP không cần sudo (hỏi mật khẩu sudo)
    ./install.sh --uninstall

Có thể chạy thẳng ./RPBotStudio hoặc ./DAPLinkFlasher mà không cần cài.

RPBot Studio cần trên máy:
    ROS 2 Jazzy có RViz (sudo apt install ros-jazzy-desktop) và ssh
    SSH key đăng nhập được vào robot
Bản Linux chạy ROS trực tiếp, không qua WSL. Cấu hình và log ở ~/.local/state/RPBotConsole.

DAPLink Flasher cần:
    OpenOCD (sudo apt install openocd); bản Linux không gói sẵn OpenOCD
    Quyền USB cho mạch nạp: ./install.sh --udev rồi cắm lại mạch nạp
Cấu hình ở ~/.config/dap_flasher. Giữ nguyên quy tắc an toàn: chỉ nạp khi động cơ chưa bật.
