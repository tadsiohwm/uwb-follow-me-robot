# Kiểm tra bản chỉ hiển thị BU01 + F

Chạy từ thư mục gốc repo:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g -Itests/a1_lidar tests/a1_lidar/check.cpp -o /tmp/check_a1_front
ASAN_OPTIONS=detect_leaks=0 /tmp/check_a1_front
```

Kiểm tra calibration offset dương/âm, ghép cùng poll và chống dòng A2 lỗi/lặp/cũ, gói X3 mẫu của hãng, nhận phân mảnh, checksum sai, gói tối đa 255 điểm, chỉ lấy trước, lấy min không bị điểm xa ghi đè, hiển thị F độc lập khi thiếu UWB/đang cali, hết hạn và rollover millis, lệnh c.

Các header trong thư mục test là mock chạy trên máy tính, không phải thư viện nạp ESP32 và không đưa vào thư mục sketch. Arduino IDE tự sinh khai báo hàm; harness khai báo tương ứng trước khi include `.ino`. Không thay thế việc biên dịch bằng thư viện thật hoặc thử trên phần cứng.
