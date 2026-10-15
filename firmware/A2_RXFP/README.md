# A2 UART + RX/FP

Nạp `A2_RXFP.ino` vào ESP32 A2, `A1_Lidar.ino` vào A1. A2 TX17 → A1 RX16, GND chung; UART 115200, mức logic 3.3 V. Giữ Tag hiện tại.

Phát triển trực tiếp từ sketch A2 UART tại `feature/anchor-uart-processing`, commit `165786c0271d7dcb54d1da4256944d4bb76c7420`. Giữ EUI, antenna delay 16450, mode và chân BU01. Thêm chụp RX/FP từ device trong callback range và hai trường power vào gói UART. Không sửa thư viện.

`$A2,poll,range_mm,age_ms,valid,rx100,fp100*HH\n`: power dBm ×100, sentinel 32767 khi thiếu. `valid` vẫn chỉ báo range có giá trị; A1 tự đánh giá `TIN_CAY`. Gói dài dưới 64 byte, vẫn dự phòng UART 6 ms và bỏ mẫu quá tuổi 80 ms. Bản lịch sử trong `firmware/Anchor2` không bị ghi đè.

Xem [hướng dẫn A1](../A1_Lidar/README.md) để đọc log và giới hạn cờ tin cậy. Đã kiểm tra C++ host với mock cả A1/A2, chưa build cho ESP32 hoặc thử phần cứng.
