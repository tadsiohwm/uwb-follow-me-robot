# Code BU01 mẫu

Tên do chủ dự án đặt ngày 15/10/2026, tương ứng file đính kèm `Markdown đã dán (1)(3).md` trong cuộc trò chuyện. Bản gốc Markdown lưu ở `docs/source/BU01_mau_original.md`.

`BU01_mau.ino` chỉ chuẩn hóa escape Markdown, khoảng trắng và dòng trống để dùng với Arduino IDE. Giữ thuật toán, các tham số, cách ghép poll, calibration lấy A2 làm mốc/bù A1, Kalman và vote như file gửi. Dùng cùng Tag/A2 UART và thư viện DW1000 hiện tại của chủ dự án. Không dùng các sketch Anchor/Tag lịch sử thay thế tùy ý.

Bản ghép thêm F LiDAR nằm tại `firmware/A1_Lidar/A1_Lidar.ino`; tên **code BU01 mẫu** chỉ bản UWB riêng này, không chỉ bản state cũ.
