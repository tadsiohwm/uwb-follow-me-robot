# A1 + LiDAR X3 — bản test theo state

Mở `A1_Lidar.ino` trong Arduino IDE, giữ cả bốn file `.h` cùng thư mục. Cảm biến là **YDLIDAR X3**: UART 115200, PWM motor 10 kHz, duty 200/255 theo code người dùng; chỉ nhận khoảng cách 0,12–8 m. Chọn **ESP32 Dev Module thường**, Arduino-ESP32 **3.x**, dùng đúng thư viện DW1000 đang chạy với A1 của bạn. Không dùng nguyên cấu hình này cho ESP32-C3/S2.

Đây là sketch mới tách riêng từ bản A1 người dùng gửi ngày 14/10/2026. **Chỉ điều khiển motor quay LiDAR; chưa phát xung servo hoặc điều khiển động cơ xe.** `GO`, `goc`, `d` là đầu ra logic để thử cảm biến/state. Chưa có thông số servo, driver, chân điều khiển và giới hạn cơ khí nên chưa ánh xạ góc mục tiêu thành góc servo.

## Nối dây

| Tín hiệu | GPIO trên ESP32 A1 |
|---|---:|
| Nhận dữ liệu UART từ A2 | 16 (UART2 RX) |
| UART2 TX về A2, nếu hệ hiện tại có dùng | 17 |
| Nhận luồng dữ liệu LiDAR | **32 (UART1 RX)** |
| Điều khiển tốc độ motor LiDAR | 25 |
| UART1 TX dự phòng | 33, **để trống** |
| BU01 SCK / MISO / MOSI / CS | 18 / 19 / 23 / 4 |
| BU01 RST / IRQ | 27 / 34 |

Chuyển **dây dữ liệu LiDAR trước đây đã đọc được ở GPIO16** sang GPIO32. Nếu đầu nối LiDAR chỉ ghi RX hoặc ký hiệu không rõ, phải đối chiếu đúng dây dữ liệu đầu ra/adapter, không suy luận chỉ từ nhãn. Nối GND chung, tín hiệu vào ESP32 phải phù hợp mức 3,3 V. GPIO25 là tín hiệu điều khiển motor qua giao tiếp/driver LiDAR như mạch cũ, không phải cấp nguồn trực tiếp cho motor.

Không dùng UART0/GPIO3 cho LiDAR vì Serial Monitor đang dùng để gửi lệnh và đọc kết quả. UART1 TX đặt rõ GPIO33 để tránh mặc định TX27 đụng chân reset DW1000.

## Mỗi tab làm gì?

| Tab | Vai trò |
|---|---|
| `A1_Lidar.ino` | `setup()` và `loop()` |
| `Config.h` | Chân nối, tham số, cấu trúc dữ liệu, enum State |
| `Uwb.h` | Nhận A2, ghép đúng poll, calibration, hình học và Kalman |
| `Lidar.h` | Đọc gói không chờ, kiểm tra checksum, ghép vòng quét, gom cụm |
| `Control.h` | Khóa/bám/dừng/xác nhận, vote, lệnh Serial và log |

Các file `.h` chứa phần cài đặt và chỉ được include từ sketch này; không thêm `.cpp` include lại chúng. Cách tách này giữ hàm dễ đọc, không cần sửa thư viện.

## Luồng và cách test

1. Bật cả hệ, mở Serial Monitor 115200. Đặt Tag yên giữa hai Anchor, phía trước khoảng 1 m. `CALIB`: ổn định 2 s rồi thu ít nhất 30 cặp trong ít nhất 3 s, độ lệch chuẩn mỗi bên không quá 10 cm. `offA1 = meanA2 - meanA1` được chốt, cộng vào A1; A2 giữ nguyên.
2. `KHOA_CHU`: lùi ra khoảng 1,3–1,5 m so với trung điểm Anchor để ra ngoài vùng dừng, đứng một mình phía trước. Khi một ứng viên phù hợp góc UWB ±15° và khoảng cách, ổn định 3 **vòng quét khác nhau**, chuyển `BAM_CHU`. Nếu vùng dừng thực tế lớn hơn, đứng xa hơn cho phù hợp.
3. `BAM_CHU`: tìm cụm gần vị trí chủ của vòng trước. LiDAR có trọng số 2; UWB ổn định, còn mới và phù hợp có trọng số 1. Vote 7 hướng trong 500 ms, lấy trung bình góc có trọng số của hướng thắng. Giữ Kalman UWB Q=36/R=25 và vùng trễ nhãn 2° từ bản cũ.
4. `GO=1` chỉ khi đã có kết quả vote, mục tiêu còn mới, khoảng cách lớn hơn 1,35 m và góc không vượt ±45°. Khi đạt khoảng cách, giữ `BAM_CHU` nhưng `GO=0`; không tự lùi. `goc` là góc mục tiêu, **không phải góc servo tuyệt đối**.
5. Vật cản hoặc chủ quá gần, LiDAR hỏng/mất dấu, nhiều ứng viên ngang nhau, hai nguồn mâu thuẫn lớn: chuyển `DUNG` và xóa lệnh chạy/vote ngay. Không giữ lệnh chạy cũ. UWB bị giảm tin cậy có thể tạm bám bằng LiDAR; mất hoàn toàn cặp UWB quá 2 s thì dừng.
6. Đưa chủ về vị trí nhìn rõ, đường trống rồi gửi **`r`**: vào `XAC_NHAN_LAI`, yêu cầu UWB phù hợp và 3 vòng quét mới trước khi bám. Chờ 3 s tự nó không làm xe chạy lại.

Lệnh: **`s` dừng**, **`r` xác nhận lại từ DUNG**, **`c` hiệu chuẩn lại** (đưa Tag về vị trí calibration trước). Có hoặc không có newline đều được.

Log: `U` = góc UWB đủ mới/ổn định, `ua` = góc UWB; `F/L/R` tính bằng mét, `inf` là không có khoảng cách hợp lệ, không phải bảo đảm đường trống; `cum` = số cụm ứng viên; `pkt=tốt/hỏng` giúp kiểm tra dữ liệu LiDAR; `cal` = số cặp hiệu chuẩn. Trong state chờ, góc/khoảng cách đầu ra bằng 0 vì chưa có mục tiêu điều khiển.

## Tham số phải kiểm tra

- `L=0.40 m`, antenna delay **16461**, Tag **0x007D**, EUI và mode **SHORTDATA_FAST_LOWPOWER** giữ theo file A1 vừa gửi.
- Mặc định trục giữa xe: x dương bên phải, y dương phía trước. A1 ở trái, A2 ở phải. `LIDAR_ZERO_DEG`, `LIDAR_SIGN` chỉnh để LiDAR và UWB cùng trục/cùng dấu. Thử một vật trước, trái, phải trước khi thử khóa chủ.
- `LIDAR_X_M/Y_M` là vị trí LiDAR so với trung điểm Anchor; mặc định 0 chỉ là giả định. `TAG_HEIGHT_DIFF_M` là chênh cao Tag–Anchor, mặc định 0. Cần đo thực tế; chưa xét pitch/roll hoặc chiều cao thay đổi khi đi.
- Chiều rộng xe mặc định 0,60 m, dự phòng mỗi bên 0,10 m; đầu xe cách trung điểm Anchor 0,20 m. **Đây là số tạm để test, không phải kích thước đã đo.** Vùng dừng dài thêm 1 m từ đầu xe và bao phủ bề rộng này; chưa mô hình đường quét thân xe khi đánh lái.
- Ngưỡng ±15°, sai lệch khoảng cách 0,45 m, dịch chuyển cụm 0,35 m/vòng, thời gian mất dữ liệu 500 ms và các kích thước cụm cần chỉnh qua log. Hai cảm biến không đo cùng một điểm trên người.
- Giao thức LiDAR theo mẫu người dùng: `AA 55`, 2 byte khoảng cách/mẫu, khoảng cách raw/4 mm, UART115200, PWM10kHz/8bit/200. Thêm checksum XOR, nội suy chia `N-1`, marker đầu vòng CT bit0 và công thức hiệu chỉnh góc LiDAR tam giác YDLIDAR. **Cần xác nhận đúng biến thể X3**; nếu khác định dạng, không bỏ checksum để ép chạy. Công thức hiệu chỉnh đặt ở `ANGLE_CORRECTION`; mặc định bật theo tài liệu giao thức, cần xác nhận với model thực.

## Giới hạn bản đầu

- Không phải bộ nhận diện con người: gom cụm hình học, ghép hai cụm hẹp gần nhau có thể nhầm chân kệ/người khác. Chỉ thử một chủ trong khu vực đơn giản. Không xác nhận danh tính chỉ bằng ID Tag rồi suy ra mọi điểm LiDAR tương ứng đều là chủ.
- Chưa có FPPower/RXPower ở cả hai đầu: độ tin cậy UWB hiện dựa trên tuổi mẫu, hình học và độ liên tục. NLOS lệch ổn định vẫn có thể lọt; Kalman không tự chữa sai số đó.
- Vòng quét thiếu/hỏng hoặc quá hạn không được dùng như đường trống. Hai bên chỉ hiển thị khoảng cách; chưa kết luận đó là kệ và chưa né/đi song song kệ.
- Không có MPU6050, GP, bảng bù theo góc, tự né hoặc tự chạy theo hướng cũ. Chưa có encoder/bù chuyển động xe, nên bước liên kết cụm đơn giản chỉ phù hợp thử chậm; cần đo độ trễ thực tế.
- **Tag và A2 phải dùng bản hiện tại của bạn**: A2 gửi `$A2,poll,range_mm,age_ms,valid*HH\n` với checksum XOR. Các sketch Tag/A2 gốc đang ở thư mục khác trong repo là bản lịch sử, không thay thế được bản A2 UART này.

## Kiểm tra đã thực hiện

Kiểm tra C++ trên máy bằng API mô phỏng và AddressSanitizer/UndefinedBehaviorSanitizer: offset hai dấu, khoảng cách sau bù không hợp lệ, ghép poll/tuổi mẫu/checksum A2, gói LiDAR chia nhỏ/hỏng/quá hạn/vượt 360°/255 mẫu, vòng quét và vật cản, khóa 3 vòng mới, vote, dừng ngay, xác nhận lại, hai nguồn mâu thuẫn, mất cảm biến và tràn millis. **Chưa biên dịch với ESP32 toolchain/thư viện DW1000 thực tế, chưa nạp BU01/X3.** Thư viện riêng có `timePollSent` giữ đúng API bản A1 người dùng, không tự thêm API mới.

Nguồn giao thức: [YDLIDAR SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md), [UART ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html), [LEDC ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).
