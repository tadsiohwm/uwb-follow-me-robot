# Kiểm tra BU01 + LiDAR theo dõi cụm eo

Chạy từ thư mục gốc repo:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g -Itests/a1_lidar tests/a1_lidar/check.cpp -o /tmp/check_a1_waist
ASAN_OPTIONS=detect_leaks=0 /tmp/check_a1_waist
```

Giữ kiểm tra calibration offset dương/âm, ghép cùng poll và chống dòng A2 lỗi/lặp/cũ; gói X3 mẫu, phân mảnh, checksum sai, gói 255 điểm; F chỉ lấy trước, lấy min, hết hạn và rollover millis; hiển thị khi thiếu UWB và lệnh c.

Cảnh cụm eo tổng hợp qua gói X3: vote khi calibration nhưng chưa khóa trước khi offset chốt; đủ 5 vòng mới khóa; theo dõi đổi góc; mất chủ không tự đổi người và c xóa khóa/vote. Kiểm tra cụm eo liên tục được chấp nhận, cụm hẹp/rộng bị loại, hai cụm chân hẹp không được ghép, hai cụm eo gần nhau vẫn là hai ứng viên và không khóa tùy ý khi ngang điểm. Còn kiểm tra sai góc/range, UWB cũ, mất tham chiếu và vòng quét trống làm đứt chuỗi phiếu.

Các header trong thư mục test là mock chạy trên máy tính; không phải thư viện nạp ESP32. Kích thước cụm tổng hợp dùng để kiểm tra logic và ngưỡng code, không xác nhận kích thước eo thực tế. Chưa biên dịch ESP32 hoặc thử trên BU01/X3 thật.
