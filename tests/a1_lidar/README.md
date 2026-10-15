# Kiểm tra BU01 + LiDAR theo dõi cụm eo

Chạy từ thư mục gốc repo:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g -Itests/a1_lidar tests/a1_lidar/check.cpp -o /tmp/check_a1_waist
ASAN_OPTIONS=detect_leaks=0 /tmp/check_a1_waist
```

Giữ kiểm tra calibration offset dương/âm, ghép cùng poll và chống dòng A2 lỗi/lặp/cũ; gói X3 mẫu, phân mảnh, checksum sai, gói 255 điểm; F chỉ lấy trước, lấy min, hết hạn và rollover millis; hiển thị khi thiếu UWB và lệnh c.

Cảnh cụm eo tổng hợp qua gói X3: vote khi calibration nhưng chưa khóa trước khi offset chốt; đủ 5 vòng mới khóa; theo dõi đổi góc; mất chủ không tự đổi người và c xóa khóa/vote. Kiểm tra cụm eo liên tục được chấp nhận, cụm hẹp/rộng bị loại, hai cụm chân hẹp không được ghép, hai cụm eo gần nhau vẫn là hai ứng viên và không khóa tùy ý khi ngang điểm. Còn kiểm tra sai góc/range, UWB cũ, mất tham chiếu và vòng quét trống làm đứt chuỗi phiếu.

Các header trong thư mục test là mock chạy trên máy tính; không phải thư viện nạp ESP32. Kích thước cụm tổng hợp dùng để kiểm tra logic và ngưỡng code, không xác nhận kích thước eo thực tế. Chưa biên dịch ESP32 hoặc thử trên BU01/X3 thật.

Thêm kiểm tra hồi quy thứ tự xử lý: cấp A1/A2 poll 123 rồi cho callback UWB trong LiDAR đưa A1 poll 124 đến. Bản cũ mất poll 123; bản sửa ghép 123 trước và vẫn ghép tiếp 124. Mock DW1000 được mở rộng để thực sự gọi callback và trả thiết bị/timestamp, thay vì loop rỗng. Kiểm tra lệnh d và số đếm nhận A1, byte A2, mẫu A2 hợp lệ và cặp ghép.

Bản RX/FP: cùng test biên dịch **cả sketch A1 lẫn A2 thật** với mock, chuyển gói A2 phát ra sang bộ đọc A1. Kiểm tra RX/FP được chụp cùng range trong callback, không thay đổi khi device đổi sau đó; FIFO chưa trống không gửi, tuổi mẫu cũ bị bỏ, dòng dài nhất vẫn dưới 64 byte, missing/NaN/Inf dùng sentinel, reset/non-Tag. Kiểm tra tương thích gói cũ không giữ nhầm power, gói thiếu/thừa trường/checksum sai, sai poll/hình học, tín hiệu yếu hoặc chênh lớn, quá tuổi 80 ms/rollover và báo A2 invalid sau cặp tốt. Kiểm tra log `CALIB | sc=`, `ag_lidar`, không báo khóa sớm, cờ tin cậy không làm thay đổi trạng thái chủ. Đây là kiểm tra logic máy tính; không thay thế bản build ESP32 và đo thực tế.
