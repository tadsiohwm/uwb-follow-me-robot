# Kiểm tra BU01 + F + cụm chủ LiDAR

Chạy từ thư mục gốc repo:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g -Itests/a1_lidar tests/a1_lidar/check.cpp -o /tmp/check_a1_owner
ASAN_OPTIONS=detect_leaks=0 /tmp/check_a1_owner
```

Kiểm tra calibration offset dương/âm, ghép cùng poll và chống dòng A2 lỗi/lặp/cũ, gói X3 mẫu của hãng, nhận phân mảnh, checksum sai, gói tối đa 255 điểm, chỉ lấy trước, lấy min không bị điểm xa ghi đè, hiển thị F độc lập khi thiếu UWB/đang cali, hết hạn và rollover millis, lệnh c.

Thêm cảnh tổng hợp qua các gói X3 thật: vote trong calibration nhưng chưa khóa trước khi chốt offset; đủ 5 vòng mới khóa; di chuyển sang 10° và góc LiDAR xuất đúng; mất chủ không tự gán mục tiêu mới; `c` xóa khóa/vote. Kiểm tra ghép hai cụm chân, hai ứng viên ngang điểm, ứng viên sai góc/range, UWB cũ và mất tham chiếu. Việc in Serial nhiều lần không làm tăng phiếu.

Các header trong thư mục test là mock chạy trên máy tính, không phải thư viện nạp ESP32 và không đưa vào thư mục sketch. Arduino IDE tự sinh khai báo hàm; harness khai báo tương ứng trước khi include `.ino`. Không thay thế việc biên dịch bằng thư viện thật hoặc thử trên phần cứng.

Sau khi chia lại hàm: harness khai báo đầy đủ prototype do Arduino IDE thường tự sinh. Chạy cùng bộ tình huống trên sketch trước/sau, cả hai qua ASan/UBSan và trace Serial giống từng byte (2.737 byte). Việc chia hàm không thêm ngưỡng, lệnh, trường log hoặc thuật toán mới.
