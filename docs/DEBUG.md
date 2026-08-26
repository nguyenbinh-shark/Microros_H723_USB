# Hướng dẫn Gỡ lỗi (Debug) Firmware trên STM32H723

Tài liệu này hướng dẫn cách thiết lập môi trường gỡ lỗi (debug) cho firmware trên board STM32H723VGT6. Phương pháp được đề xuất sử dụng Visual Studio Code, cung cấp một giao diện đồ họa trực quan và mạnh mẽ để kiểm soát quá trình thực thi của vi điều khiển.

> 📌 **Để biết các lệnh terminal biên dịch firmware, chạy micro-ROS Agent, và tương tác với robot**, xem mục **[Bảng lệnh terminal](../README.md#-bảng-lệnh-terminal-thường-dùng)** trong [README.md](../README.md).

---

## 1. Yêu cầu Phần cứng và Phần mềm

### Phần cứng
- Board STM32H723VGT6.
- Mạch nạp/debug ST-Link (v2 hoặc v3).

### Phần mềm (Cài trên Windows)
1.  **Visual Studio Code**: Trình soạn thảo code chính.
2.  **ARM GCC Toolchain**: Đã có sẵn trong môi trường build của bạn, bao gồm `arm-none-eabi-gdb.exe`.
3.  **OpenOCD**: Phần mềm trung gian để giao tiếp giữa GDB và mạch nạp ST-Link.
    - Tải bản OpenOCD cho Windows từ [xPack OpenOCD Releases](https://github.com/xpack-dev-tools/openocd-xpack/releases/).
    - Giải nén vào một thư mục cố định, ví dụ: `C:\Tools\xpack-openocd`.
    - Thêm đường dẫn đến thư mục `bin` của OpenOCD (ví dụ: `C:\Tools\xpack-openocd\bin`) vào biến môi trường `PATH` của Windows để có thể gọi `openocd` từ bất kỳ đâu.
4.  **ST-Link Driver**: Cần thiết để Windows nhận diện mạch nạp. Bạn có thể tải từ [trang chủ của ST](https://www.st.com/zh/development-tools/stsw-stm32153.html).
5.  **Extension "Cortex-Debug" cho VS Code**:
    - Mở VS Code.
    - Vào mục Extensions (Ctrl+Shift+X).
    - Tìm kiếm `Cortex-Debug` và nhấn **Install**.

---

## 2. Cấu hình Visual Studio Code để Debug

Để debug, chúng ta cần tạo một file cấu hình `launch.json` để chỉ cho VS Code biết cách khởi động OpenOCD và GDB.

1.  **Mở thư mục dự án** (`p:\Prj_STM32\ros_h7`) trong VS Code.
2.  Chuyển sang tab **Run and Debug** (biểu tượng play với con bọ, hoặc Ctrl+Shift+D).
3.  Nhấn vào **"create a launch.json file"**. Nếu được hỏi, hãy chọn **"Cortex-Debug"**.
4.  VS Code sẽ tạo một file `.vscode/launch.json` trong thư mục dự án của bạn. Xóa nội dung mặc định và thay thế bằng nội dung sau:

    ```json
    {
        "version": "0.2.0",
        "configurations": [
            {
                "name": "Debug (OpenOCD)",
                "type": "cortex-debug",
                "request": "launch",
                "servertype": "openocd",
                "cwd": "${workspaceRoot}",
                "executable": "./build/ros_h7.elf",
                "device": "STM32H723VGTx",
                "configFiles": [
                    "interface/stlink.cfg",
                    "target/stm32h7x.cfg"
                ],
                "svdFile": "${workspaceRoot}/.vscode/STM32H723.svd",
                "runToEntryPoint": "main",
                "showDevDebugOutput": "raw",
                "postLaunchCommands": [
                    "monitor reset halt"
                ]
            }
        ]
    }
    ```

5.  **Tải file SVD (System View Description)**: File này giúp VS Code hiển thị tên và giá trị các thanh ghi ngoại vi (GPIO, UART, FDCAN, v.v.) một cách trực quan.
    - Tải file `STM32H723.svd` từ [kho SVD của ST](https://www.st.com/zh/embedded-software/stsw-stm32153.html).
    - Tạo thư mục `.vscode` trong project của bạn (nếu chưa có).
    - Lưu file vừa tải vào `p:\Prj_STM32\ros_h7\.vscode\STM32H723.svd`.

---

## 3. Bắt đầu phiên Debug

Sau khi đã cấu hình xong, bạn có thể bắt đầu gỡ lỗi.

1.  **Kết nối phần cứng**:
    - Cắm mạch nạp ST-Link vào cổng SWD trên board STM32.
    - Cắm ST-Link vào cổng USB của máy tính.
2.  **Biên dịch Firmware**: Đảm bảo bạn đã biên dịch phiên bản code mới nhất để file `.elf` được cập nhật.
    ```bash
    # Chạy trong WSL/Ubuntu
    cd /mnt/p/Prj_STM32/ros_h7
    make -j$(nproc)
    ```
3.  **Khởi động Debugger trong VS Code**:
    - Mở VS Code, chuyển sang tab **Run and Debug**.
    - Đảm bảo cấu hình **"Debug (OpenOCD)"** đang được chọn ở thanh trên cùng.
    - Nhấn nút **Start Debugging** (biểu tượng play màu xanh lá, hoặc phím F5).

VS Code sẽ tự động:
- Chạy OpenOCD.
- Chạy GDB và kết nối tới OpenOCD.
- Nạp file `ros_h7.elf` vào vi điều khiển.
- Dừng lại tại hàm `main()`.

---

## 4. Các thao tác Debug cơ bản

Khi đã ở trong phiên debug, bạn có thể sử dụng các công cụ của VS Code:

- **Thanh công cụ Debug**:
    - **Continue (F5)**: Chạy code cho đến khi gặp breakpoint tiếp theo.
    - **Step Over (F10)**: Chạy qua một dòng lệnh (không đi vào bên trong hàm).
    - **Step Into (F11)**: Đi vào bên trong một hàm để debug.
    - **Step Out (Shift+F11)**: Thoát ra khỏi hàm hiện tại.
    - **Restart (Ctrl+Shift+F5)**: Khởi động lại phiên debug.
    - **Stop (Shift+F5)**: Dừng phiên debug.

- **Cửa sổ bên trái**:
    - **VARIABLES**: Xem giá trị của các biến local và global.
    - **WATCH**: Thêm các biến hoặc biểu thức bạn muốn theo dõi liên tục.
    - **CALL STACK**: Xem chuỗi các hàm đã được gọi để dẫn đến vị trí hiện tại.
    - **BREAKPOINTS**: Quản lý tất cả các breakpoint. Bạn có thể tạo breakpoint bằng cách click vào lề trái của trình soạn thảo code.

- **Cửa sổ CORTEX PERIPHERALS**:
    - Trong panel bên trái, bạn sẽ thấy một mục `CORTEX PERIPHERALS`.
    - Mở rộng nó ra, bạn có thể xem và theo dõi trạng thái của các thanh ghi ngoại vi như `RCC`, `GPIOx`, `FDCANx` theo thời gian thực.
