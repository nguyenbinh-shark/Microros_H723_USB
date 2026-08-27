# Báo Cáo Phân Tích & Sửa Lỗi Hiển Thị LCD ST7789 trên STM32H723

**Dự án:** `ros_h7_usb`
**Thành phần:** Giao diện người dùng / LCD ST7789 qua giao thức SPI

---

## 1. Hiện trạng và Biểu hiện Lỗi
- **Biểu hiện:** Giao diện màn hình Boot Screen (viền Cyan, chữ cảnh báo cấu hình) được lập trình để vẽ ngay trong `main.c` không hiển thị. Màn hình hoàn toàn trống không (màu đen).
- **Log hệ thống:** Trên Terminal (UART) vẫn báo Log `[BOOT] LCD Init Done! SPI1: State=1...` và sau đó báo Task LCD đã khởi chạy bình thường, chứng tỏ vi điều khiển không bị treo (HardFault).

## 2. Phân tích Nguyên nhân Cốt lõi & Giải pháp Khắc phục

Quá trình điều tra mã nguồn (Codebase) đã phát hiện ra lỗi không đến từ một nguyên nhân đơn lẻ, mà là hệ quả của **3 xung đột kỹ thuật** xảy ra cùng một lúc giữa cấu hình ngoại vi SPI, thiết kế điện tử (Hardware GPIO) và Logic của FreeRTOS:

### Nguyên nhân 1: Xung đột chế độ truyền tải SPI (SPI Mode)
* **Phân tích:** Lõi phần cứng SPI1 được cấu hình ở **SPI Mode 2** (`CPOL = HIGH` và `CPHA = 1EDGE`). Tuy nhiên, chip điều khiển ST7789 (và cả thư viện giả lập Soft-SPI đi kèm `lcd.c`) đều vận hành theo tiêu chuẩn **SPI Mode 0** (`CPOL = LOW`, `CPHA = 1EDGE`). Việc lệch cấu hình Mode khiến tín hiệu data (MOSI) bị chốt sai thời điểm cạnh xung Clock, làm LCD không thể giải mã lệnh khởi tạo.
* **Cách khắc phục:** 
  Sửa trong file `Core/Src/spi.c` về Mode 0:
  ```c
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  ```

### Nguyên nhân 2: Băng thông xung Clock vượt ngưỡng đáp ứng của GPIO_SPEED_LOW
* **Phân tích:** Dựa theo khuyến nghị phần cứng, các chân IO truyền tải SPI (PB3, PD7) được thiết lập ở chế độ **`GPIO_SPEED_FREQ_LOW`** (giới hạn đáp ứng tần số tối đa chỉ khoảng 2MHz). 
Trong khi đó, bộ chia tần số SPI lại đang đặt ở `SPI_BAUDRATEPRESCALER_8` từ Clock gốc 48MHz, tạo ra xung nhịp SCK ở tần số **6MHz**. Khi một xung 6MHz đi qua chân IO chỉ hỗ trợ 2MHz, tín hiệu số (xung vuông) bị suy hao và biến dạng nghiêm trọng thành hình sin (do điện dung ký sinh). Hệ quả là IC màn hình không nhận diện được mức logic HIGH/LOW.
* **Cách khắc phục:**
  Tăng hệ số chia xung nhịp trong `Core/Src/spi.c` để hạ tần số SPI xuống 1.5MHz, đảm bảo toàn vẹn tín hiệu (Signal Integrity) qua chân LOW Speed:
  ```c
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  ```

### Nguyên nhân 3: Lỗi Logic ghi đè (Hardware Reset) trong chu trình chạy FreeRTOS Task
* **Phân tích:** Mã nguồn thiết kế hiển thị màn hình kiểm tra trực quan ngay từ Bootloader trong `main.c` thông qua việc gọi hàm `LCD_Init()`. Tuy nhiên, ngay sau khi hệ điều hành FreeRTOS khởi động, Task `LCD_Task_Entry` lại lập tức gọi hàm `LCD_Debug_Init()`, và trong ruột hàm này lại có một lời gọi `LCD_Init()` thứ hai. 
Lời gọi thứ hai này đã kích hoạt lệnh kéo chân Reset cứng (`LCD_RES_Clr()`), ngay lập tức xóa trắng bộ đệm hiển thị (GRAM) của màn hình chỉ khoảng 200ms sau khi màn hình Boot được vẽ. Kết quả là mắt người không thể nhìn kịp giao diện khởi động.
* **Cách khắc phục:**
  Loại bỏ lệnh gọi `LCD_Init()` thừa trong file `User/APP/lcd_task.c`:
  ```c
  void LCD_Debug_Init(void)
  {
      // Đã xoá LCD_Init(); -> Việc khởi tạo phần cứng chỉ thực hiện 1 lần duy nhất ở main.c
      NavKey_Init();
      s_current_page = 0;
      s_last_drawn_page = 0xFF;
  }
  ```

## 3. Kết luận
Lỗi đen màn hình LCD là tổ hợp của sự bất đồng bộ giữa tín hiệu vật lý, giao thức truyền thông SPI và vòng đời khởi tạo phần cứng trong môi trường đa nhiệm (RTOS). 

Sau khi áp dụng 3 bản vá trên, quá trình giao tiếp từ vi điều khiển H7 đến ST7789 đã ổn định. Giao diện Boot Screen khởi chạy chính xác, hiển thị các màu sắc đặc trưng và theo sau đó là các thông số giám sát hệ thống (IMU, CAN, UART) vận hành mượt mà trên FreeRTOS.

