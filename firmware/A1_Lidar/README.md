# BU01 mẫu + LiDAR X3 — chỉ hiển thị dữ liệu

Bản ngày 15/10/2026 làm lại từ **code BU01 mẫu** do chủ dự án gửi cùng ngày, không dựa trên thuật toán khóa/bám chủ của bản trước. Chỉ cần một file `A1_Lidar.ino`; không có file `.h` riêng của chương trình.

Giữ cấu hình và thuật toán UWB của mẫu: A2 làm mốc, cộng/trừ offset vào A1; L = 0,40 m; antenna delay A1 = 16461; mode SHORTDATA_FAST_LOWPOWER; ghép mẫu cùng poll; calibration; Kalman và vote hướng 500 ms. Có thêm kiểm tra A1 sau bù phải dương và trong giới hạn trước khi tính góc.

LiDAR chỉ lấy **F: khoảng cách nhỏ nhất trong vùng trước ±15° giữa các lần in**. Kiểm tra checksum gói X3, đọc UART theo byte có sẵn, không chờ đủ byte bằng vòng lặp chặn. Không tính trái/phải, cụm người, state, khóa chủ, điều khiển xe hoặc trộn số đo UWB–LiDAR.

## Đấu dây

| Kết nối trên ESP32 A1 | GPIO |
|---|---:|
| A2 UART2 → RX A1 | 16 |
| TX UART2 A1 → A2 (nếu hệ hiện tại dùng) | 17 |
| Dây dữ liệu đầu ra X3 → RX UART1 A1 | **32** |
| PWM motor X3 | 25 |
| TX UART1 dự phòng — để trống | 33 |
| BU01 SCK / MISO / MOSI / CS | 18 / 19 / 23 / 4 |
| BU01 RST / IRQ | 27 / 34 |

Nối GND chung. Giữ dây dữ liệu X3 đang chạy ở bản ghép trước trên GPIO32. A2 đã dùng GPIO16 nên không đưa dữ liệu X3 vào GPIO16. Sketch dùng ESP32 thường và Arduino-ESP32 3.x (`ledcAttach`). Giữ đúng bản thư viện DW1000/DW1000Ranging, Tag và A2 UART đang dùng; không cần sửa/cài thêm thư viện LiDAR.

## Chạy thử

1. Mở `A1_Lidar.ino`, nạp vào ESP32 A1; mở Serial Monitor 115200 baud.
2. Giữ Tag đứng yên trên đường trung trực hai Anchor, khoảng 1 m phía trước, cùng mặt phẳng đo như BU01 mẫu. Chờ `CALIB_OK`.
3. Quan sát A1, A2, góc UWB và F. Trong lúc đang calibration cũng in A1/A2/F mỗi giây; sau calibration in mỗi 500 ms. Không phải đứng ngoài 1,3 m mới xem được F.
4. Gửi `c` để lấy lại calibration, hoặc nhấn reset A1. Không có lệnh `s`/`r` vì bản này chỉ hiển thị.

Ví dụ minh họa (không phải kết quả thử phần cứng):

```text
A1=1.054 m | A2=1.054 m | goc=0.0 do | THANG | UWB=1 | F=1.020 m
A1=-- | A2=-- | goc=-- | UWB=0 | F=1.020 m
```

- **A1**: mẫu A1 mới nhất, đã cộng offset sau calibration; trước calibration là raw. **A2**: mẫu A2 hợp lệ mới nhất, giữ nguyên làm mốc. Hai cột này cho xem từng Anchor, không bắt buộc cùng poll; mẫu quá 500 ms được ghi `--`.
- **goc / UWB**: góc Kalman của hướng thắng vote trong cửa sổ 500 ms; việc tính góc vẫn bắt buộc cặp A1/A2 cùng poll. `UWB=0` nghĩa là không có phiếu góc hợp lệ trong cửa sổ, không phải toàn bộ phần cứng UWB hỏng.
- **F**: khoảng cách mét, điểm gần nhất trong vùng ±15° được nhận kể từ lần in trước. `--` khi không có điểm hợp lệ mới trong vùng; không có nghĩa là đường trống. Dữ liệu quá 500 ms không được hiển thị là mới. Trong calibration, khoảng giữa các lần in là 1 giây.
- `LIDAR_ZERO_DEG` mặc định 0. Đầu LiDAR phải có hướng 0° trùng phía trước xe; nếu lắp xoay, sửa biến này.

A1/A2 là khoảng cách đến Tag, F là khoảng cách đến bề mặt phản xạ gần nhất phía trước. F có thể là chân người, bàn, tường hoặc vật khác; hai cảm biến cũng lấy mẫu khác thời điểm nên không mặc định các số phải bằng nhau.

## Code BU01 mẫu và kiểm tra

Bản UWB mẫu riêng: [`../BU01_mau/BU01_mau.ino`](../BU01_mau/BU01_mau.ino). Bản Markdown gửi gốc lưu tại `docs/source/BU01_mau_original.md` để đối chiếu. Bản riêng chỉ chuẩn hóa escape/khoảng trắng, giữ thuật toán và cấu hình người dùng.

Đã biên dịch/kiểm tra logic bằng C++ trên máy tính với Arduino/DW1000 giả lập và AddressSanitizer/UBSan: calibration hai dấu offset, ghép UART cùng poll, gói X3 phân mảnh/checksum, bỏ điểm bên hông, lấy min F, số đo hết hạn và hiển thị F độc lập với UWB. **Chưa biên dịch cho ESP32 hoặc thử BU01/X3 thực tế**. Chưa xác định nguyên nhân tỉ lệ gói bị từ chối cao ở log bản trước.

Giao thức tham chiếu: [YDLIDAR SDK Communication Protocol](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md).
