# Xe tự cân bằng Arduino

Dự án này là mã nguồn Arduino cho một xe tự cân bằng 2 bánh (self-balancing robot) sử dụng:

- Cảm biến MPU6050 (Gia tốc kế + Con quay hồi chuyển)
- Bộ điều khiển PID
- Logic mờ (Fuzzy Logic)
- Điều khiển qua Bluetooth (SoftwareSerial)

## Tính năng chính

- Đọc dữ liệu góc và vận tốc góc từ MPU6050
- Kết hợp PID và logic mờ để tính toán lực điều khiển động cơ
- Điều khiển hai động cơ DC thông qua driver động cơ
- Nhận lệnh điều khiển từ module Bluetooth:
  - `F`: tiến
  - `B`: lùi
  - `L`: rẽ trái
  - `R`: rẽ phải
  - `S`: dừng

## Phần cứng cần thiết

- Arduino (Uno/Nano/...) 
- Cảm biến MPU6050
- 2 động cơ DC và driver điều khiển động cơ (H-bridge)
- Module Bluetooth (ví dụ HC-05/HC-06)
- Dây nối và nguồn phù hợp

## Chân kết nối trong mã

- ENA: PWM điều khiển tốc độ động cơ trái (chân 5)
- IN1 / IN2: điều khiển chiều động cơ trái (chân 8, 9)
- ENB: PWM điều khiển tốc độ động cơ phải (chân 6)
- IN3 / IN4: điều khiển chiều động cơ phải (chân 10, 11)
- Bluetooth RX: chân 7
- Bluetooth TX: chân 13

## Hướng dẫn sử dụng

1. Kết nối phần cứng theo sơ đồ phù hợp.
2. Cài đặt thư viện Arduino cần thiết:
   - `Wire`
   - `I2Cdev`
   - `MPU6050`
   - `PID_v1`
   - `Fuzzy`
   - `SoftwareSerial`
3. Nạp `Codexecanbang.ino` vào Arduino.
4. Mở ứng dụng điều khiển Bluetooth trên điện thoại và gửi lệnh.

## Ghi chú

- Mã đã bao gồm phần khởi tạo 25 luật fuzzy và PID cơ bản.
- Nếu xe ngã quá 45 độ, động cơ sẽ dừng để bảo vệ phần cứng.
