# BU01 mẫu + LiDAR X3 — theo dõi cụm eo người

Bản ngày 16/10/2026 phát triển từ **A1_Lidar_TheoDoiChu** theo yêu cầu chủ dự án. Dùng lại cấu trúc trước lần chia hàm; thay cụm chân bằng một cụm vùng eo, bỏ hoàn toàn bước ghép hai chân. Thêm vote chọn cụm chủ trong lúc calibration, theo dõi cụm và xuất **goc_lidar**. Vẫn chỉ một file `A1_Lidar.ino`; không có file `.h` riêng của chương trình. **Code BU01 mẫu** riêng được giữ nguyên.

Giữ cấu hình và thuật toán UWB của mẫu: A2 làm mốc, cộng/trừ offset vào A1; L = 0,40 m; antenna delay A1 = 16461; mode SHORTDATA_FAST_LOWPOWER; ghép mẫu cùng poll; calibration; Kalman và vote hướng 500 ms. Có thêm kiểm tra A1 sau bù phải dương và trong giới hạn trước khi tính góc.

LiDAR chỉ lấy **F: khoảng cách nhỏ nhất trong vùng trước ±15° giữa các lần in**. Kiểm tra checksum gói X3, đọc UART theo byte có sẵn, không chờ đủ byte bằng vòng lặp chặn. Không tính F trái/phải hoặc điều khiển xe. Phần cụm chủ dùng các điểm trong vùng trước **±80°**, độc lập với F ±15°. Góc UWB cũ và góc LiDAR được xuất riêng.

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

1. **Đặt mặt phẳng quét LiDAR nằm ngang và đi qua eo người**, đeo Tag ở eo. Không nghiêng LiDAR đang đặt thấp để chiếu lên eo: mặt phẳng nghiêng làm số đo phụ thuộc hướng/khoảng cách và không khớp mô hình 2D này.
2. Mở `A1_Lidar.ino`, nạp vào ESP32 A1; mở Serial Monitor 115200 baud.
3. Giữ Tag đứng yên trên đường trung trực hai Anchor, khoảng 1 m phía trước, cùng mặt phẳng đo như BU01 mẫu. Chờ `CALIB_OK`.
4. Giữ nguyên vị trí đến khi `CHU=DA_KHOA`. Trong lúc calibration, code vote cụm LiDAR khớp UWB; chỉ khóa sau khi offset đã chốt và đủ **5 vòng quét mới liên tiếp**. Nếu chưa đủ, tiếp tục chờ ở `CHU=CHO_n/5`.
5. Di chuyển chậm sang hai phía phía trước xe. `goc_lidar` là góc tâm cụm theo LiDAR, cập nhật theo cụm đang theo dõi; `goc` vẫn là góc Kalman/vote UWB. Dương là bên phải, âm là bên trái nếu lắp đúng trục.
6. `CHU=MAT_CHU` nghĩa là mất cụm hoặc dữ liệu tham chiếu. Bản này không tự gán người khác vào mục tiêu: về giữa khoảng 1 m rồi gửi `c` để calibration/khóa lại. Gửi `d` để xem số đếm nhận UWB khi cần kiểm tra. Không có lệnh `s`/`r`.

Vẫn in A1/A2/F khi chưa khóa hoặc đã mất chủ. Trong lúc đang calibration in mỗi giây; sau calibration in mỗi 500 ms. Không cần đứng ngoài 1,3 m mới xem được F.

Ví dụ minh họa (không phải kết quả thử phần cứng):

```text
A1=1.054 m | A2=1.054 m | goc=0.0 do | THANG | UWB=1 | F=1.020 m | goc_lidar=1.2 do | CHU=DA_KHOA
A1=-- | A2=-- | goc=-- | UWB=0 | F=1.020 m | goc_lidar=-- | CHU=MAT_CHU
```

- **A1**: mẫu A1 mới nhất, đã cộng offset sau calibration; trước calibration là raw. **A2**: mẫu A2 hợp lệ mới nhất, giữ nguyên làm mốc. Hai cột này cho xem từng Anchor, không bắt buộc cùng poll; mẫu quá 500 ms được ghi `--`.
- **goc / UWB**: góc Kalman của hướng thắng vote trong cửa sổ 500 ms; việc tính góc vẫn bắt buộc cặp A1/A2 cùng poll. `UWB=0` nghĩa là không có phiếu góc hợp lệ trong cửa sổ, không phải toàn bộ phần cứng UWB hỏng.
- **F**: khoảng cách mét, điểm gần nhất trong vùng ±15° được nhận kể từ lần in trước. `--` khi không có điểm hợp lệ mới trong vùng; không có nghĩa là đường trống. Dữ liệu quá 500 ms không được hiển thị là mới. Trong calibration, khoảng giữa các lần in là 1 giây.
- `LIDAR_ZERO_DEG` mặc định 0. Đầu LiDAR phải có hướng 0° trùng phía trước xe; nếu lắp xoay, sửa biến này.

A1/A2 là khoảng cách đến Tag, F là khoảng cách đến bề mặt phản xạ gần nhất phía trước. F có thể là chân người, bàn, tường hoặc vật khác; hai cảm biến cũng lấy mẫu khác thời điểm nên không mặc định các số phải bằng nhau.

## Vote cụm và theo dõi

- Chia các điểm hợp lệ thành ô góc 1° trong ±80°, giữ điểm gần nhất mỗi ô. Ghép các ô gần nhau thành cụm: ít nhất **5 ô góc có điểm**, rộng **15–65 cm**, khoảng cách điểm kề không quá 20 cm; cho phép khuyết tối đa 2 ô ở giữa. Mỗi cụm là một ứng viên vùng eo. **Không ghép hai cụm riêng với nhau**. Ngưỡng 15–65 cm là giá trị khởi đầu để thử, không phải số đo cơ thể chuẩn hoặc kích thước đã được đo trên chủ dự án.
- Sau ít nhất 10 cặp calibration ổn định, dùng trung bình A2 và A1 đã bù theo trung bình tạm để làm tham chiếu vote; chưa xuất góc chủ hợp lệ trước khi calibration hoàn tất.
- Mỗi vòng quét mới (mốc CT của X3) chỉ cho 1 phiếu. Cụm phải nằm trong ±15° hướng UWB và có khoảng cách hình học dự kiến đến **cả A1 và A2** lệch không quá 35 cm. Nếu hai ứng viên có điểm số gần nhau, không bỏ phiếu tùy ý. Cụm vote liên tiếp không được nhảy quá 20 cm giữa các vòng. Cần 5 phiếu liên tiếp và calibration hoàn tất để khóa.
- Khi đã khóa, ưu tiên cụm gần vị trí trước (tối đa 35 cm/vòng), đồng thời kiểm tra UWB rộng hơn: góc lệch tối đa 25°, mỗi range lệch tối đa 50 cm. Chỉ dùng tham chiếu UWB mới trong 200 ms để cập nhật cụm.
- Không cập nhật được cụm hoặc UWB quá 500 ms, hoặc UWB bị reset/kết nối lại: báo mất chủ. Góc LiDAR còn được giữ tối đa 500 ms giữa các quan sát hợp lệ; `--` sau khi mất chủ. Cần `c` để khóa lại.
- Chỉ các điểm từ gói checksum hợp lệ mới được dùng. Bỏ vòng đầu đang dở, hoặc vòng dài ngoài 60–500 ms; một vòng có thể thiếu điểm nếu mất gói. Đây là theo dõi hình học đơn giản, không nhận dạng danh tính và không bảo đảm tránh đổi chủ khi hai người/vật sát nhau.

Các thông số cần đúng với cách lắp:

| Biến | Mặc định | Ý nghĩa |
|---|---:|---|
| `L` | 0,40 m | Khoảng cách tâm anten A1–A2 |
| `LIDAR_ZERO_DEG` | 0° | Đổi trục góc LiDAR về trước xe |
| `LIDAR_X_M`, `LIDAR_Y_M` | 0, 0 m | Vị trí LiDAR so với trung điểm Anchor: x dương sang phải, y dương ra trước |
| `TAG_HEIGHT_DIFF_M` | 0 m | Chênh cao Tag–Anchor cho phần đối chiếu cụm/range |
| `OWNER_VOTES` | 5 | Số vòng quét vote liên tiếp |
| `WAIST_MIN_WIDTH_M`, `WAIST_MAX_WIDTH_M` | 0,15 / 0,65 m | Giới hạn chiều rộng cụm eo để thử; chỉnh theo dữ liệu thực |
| `WAIST_MIN_POINTS` | 5 | Số ô góc tối thiểu trong cụm |

Mặc định giả sử LiDAR gần trung điểm hai Anchor, A1 bên trái/A2 bên phải, Tag ở gần cùng cao độ với Anchor. Nếu Tag ở eo và Anchor thấp hơn, đo chênh cao và sửa `TAG_HEIGHT_DIFF_M`. Phần góc UWB gốc vẫn giữ mô hình phẳng của BU01 mẫu; tham số chênh cao dùng cho phần ghép cụm mới. LiDAR phải quét trúng **eo**; mặt phẳng quét ở chân không thể chuyển thành đo eo chỉ bằng phần mềm. Chiều rộng cụm là bề rộng bề mặt đang nhìn thấy, thay đổi khi xoay người, mặc quần áo, bị tay hoặc vật khác che; cần thử cả hướng chính diện lẫn hướng ngang trước khi chỉnh ngưỡng.

## Bản sửa ưu tiên UWB và kiểm tra nhận A1/A2

Phần LiDAR được xử lý **sau khi ghép và xử lý cặp UWB**, theo thứ tự của BU01 mẫu. Trước đây, callback UWB được gọi trong phần đọc LiDAR có thể ghi đè mẫu A1 bằng poll tiếp theo trước khi A1 cũ được ghép với A2. Đã tái hiện việc mất cặp này trong host test và kiểm tra bản sửa giữ được cả poll cũ lẫn poll tiếp theo. Đây là lỗi có thể xảy ra trong code; chưa có log phần cứng để kết luận đó là toàn bộ nguyên nhân người dùng không thấy A1/A2.

Gửi `d` trong Serial Monitor để in một dòng chẩn đoán. Gửi hai lần cách nhau khoảng 1 giây và xem cột nào tăng; bộ đếm tích lũy từ khi bật nguồn, không xóa bởi `c`. Dòng thường A1/A2/F/góc/trạng thái chủ giữ nguyên.

| Cột DBG | Ý nghĩa |
|---|---|
| `A1_rx` | Số callback range A1 nhận được đúng short address Tag 0x007D |
| `addr` | Short address của thiết bị trong callback range gần nhất |
| `A2_bytes` | Tổng byte đọc từ UART2 của A2, kể cả dữ liệu không hợp lệ/bị bỏ |
| `A2_ok` | Số mẫu A2 mới qua kiểm tra checksum, định dạng, range và tuổi mẫu |
| `flag` | Trạng thái cuối do A2 gửi: 1 = báo phép đo hợp lệ, 0 = báo không hợp lệ, -1 = sự kiện reset/kết nối; -2 = chưa đọc được dòng đúng định dạng/checksum |
| `PAIR` | Số cặp A1/A2 cùng poll và đủ mới đã được ghép |

`A1_rx` không tăng: kiểm tra đường UWB A1–Tag, địa chỉ và mode. `A2_bytes` không tăng: kiểm tra A2 TX → A1 GPIO16, GND chung và nguồn A2. Byte tăng nhưng `A2_ok` không tăng: cần xem phiên bản/gói UART, flag và tuổi mẫu. Cả hai nhận mẫu mà `PAIR` không tăng: chưa ghép được cùng poll trong cửa sổ 80 ms, cần đối chiếu code Tag/A2 đang nạp.

Bản sửa giữ nguyên chân: BU01 CS4/RST27/IRQ34, SPI18/19/23; A2 RX16/TX17; LiDAR RX32/motor25. Không đổi thư viện, mode, antenna delay hoặc ngưỡng nhận cụm eo.

## Code BU01 mẫu và kiểm tra

Bản UWB mẫu riêng: [`../BU01_mau/BU01_mau.ino`](../BU01_mau/BU01_mau.ino). Bản Markdown gửi gốc lưu tại `docs/source/BU01_mau_original.md` để đối chiếu. Bản riêng chỉ chuẩn hóa escape/khoảng trắng, giữ thuật toán và cấu hình người dùng.

Đã biên dịch/kiểm tra logic bằng C++ trên máy tính với Arduino/DW1000 giả lập và AddressSanitizer/UBSan: calibration hai dấu offset, ghép UART cùng poll, gói X3 phân mảnh/checksum, bỏ điểm bên hông, lấy min F, số đo hết hạn và hiển thị F độc lập với UWB; thêm kiểm tra vote thật trong calibration, không khóa sớm, theo dõi đổi góc, cụm eo liên tục, loại cụm hẹp/rộng, không ghép hai ứng viên eo, ứng viên mơ hồ, sai góc/range, dữ liệu cũ, mất chủ và `c` khóa lại. **Chưa biên dịch cho ESP32 hoặc thử BU01/X3 thực tế**. Chưa xác định nguyên nhân tỉ lệ gói LiDAR bị từ chối cao ở log bản trước; sửa ưu tiên UWB không thay phần checksum LiDAR.

Giao thức tham chiếu: [YDLIDAR SDK Communication Protocol](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md).
