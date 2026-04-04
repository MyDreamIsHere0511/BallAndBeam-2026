# 🏗️ DỰ ÁN BALL AND BEAM — TÀI LIỆU TỔNG HỢP
### Điều khiển cân bằng bóng trên thanh ngang
**Cập nhật lần cuối:** 24/03/2026  
**Trạng thái:** Giai đoạn chuẩn bị — Chốt linh kiện, nghiên cứu thuật toán, chuẩn bị trước khi lắp ráp thực tế.

---

# MỤC LỤC
1. [Tổng quan dự án](#1-tổng-quan-dự-án)
2. [Chốt linh kiện chính thức](#2-chốt-linh-kiện-chính-thức)
3. [Sơ đồ nối dây chi tiết](#3-sơ-đồ-nối-dây-chi-tiết)
4. [Thuật toán ĐO LƯỜNG — Bộ lọc cảm biến](#4-thuật-toán-đo-lường--bộ-lọc-cảm-biến)
5. [Thuật toán ĐIỀU KHIỂN — PID / Fuzzy Logic và các phương án khác](#5-thuật-toán-điều-khiển--pid--fuzzy-logic-và-các-phương-án-khác)
6. [Hiển thị & Giám sát — LabVIEW vs MQTT vs Webserver](#6-hiển-thị--giám-sát--labview-vs-mqtt-vs-webserver)
7. [Tiến trình dự án — Đã làm gì, chưa làm gì](#7-tiến-trình-dự-án--đã-làm-gì-chưa-làm-gì)
8. [Ghi chú lỗi cần sửa trong code VL53L0X.ino](#8-ghi-chú-lỗi-cần-sửa-trong-code-vl53l0xino)
9. [Lộ trình phát triển tiếp theo](#9-lộ-trình-phát-triển-tiếp-theo)

---

# 1. Tổng quan dự án

## 1.1 Mô tả hệ thống
Hệ thống Ball and Beam là một bài toán điều khiển kinh điển: một quả bóng đặt trên thanh ngang (beam), thanh được gắn với động cơ bước ở điểm tựa. Hệ thống phải tự động điều chỉnh góc nghiêng của thanh để giữ quả bóng ở vị trí mong muốn (setpoint).

**Đặc điểm quan trọng:** Hệ thống này là **open-loop không ổn định** — nghĩa là nếu không có bộ điều khiển, bóng sẽ LUÔN lăn ra ngoài. Đây là thách thức chính và cũng là mục đích học tập.

## 1.2 Mục tiêu
- **Giai đoạn 1 (Hiện tại):** Điều khiển PID cơ bản — giữ bóng cân bằng tại 1 điểm
- **Giai đoạn 2:** Nâng cấp lên Fuzzy Logic để xử lý phi tuyến tốt hơn
- **Giai đoạn 3 (Tương lai xa):** Tích hợp camera + AI để tự học cân bằng

## 1.3 Nguyên lý hoạt động tổng thể

```
    Setpoint (Vị trí mong muốn, VD: 20cm)
         │
         ▼
    ┌─────────┐     ┌──────────────┐     ┌──────────────┐     ┌───────────┐
    │ Tính    │────▶│ Bộ điều khiển│────▶│   MKS EMM42  │────▶│   Thanh   │
    │ Error   │     │ PID / Fuzzy  │     │  Step Motor   │     │   Beam    │
    │ e = SP  │     │              │     │  (Quay góc θ) │     │  (Nghiêng)│
    │   - PV  │     └──────────────┘     └──────────────┘     └─────┬─────┘
    └────▲────┘                                                      │
         │                                                           ▼
         │                                                    ┌───────────┐
         │           Feedback                                 │   Bóng    │
         │◀───────────────────────────────────────────────────│   lăn     │
         │                                                    └─────┬─────┘
         │                                                          │
    ┌────┴────────┐                                                 │
    │  VL53L0X    │◀────────── Đo khoảng cách ─────────────────────┘
    │  (Laser ToF)│    Phản xạ laser hồng ngoại từ bóng
    └─────────────┘
    
    Đồng thời:
    ┌─────────────┐     ┌──────────────────────────────┐
    │ OLED 0.96"  │◀────│ Hiển thị: Khoảng cách, PID,  │
    │ (Trên board)│     │ setpoint, trạng thái hệ thống │
    └─────────────┘     └──────────────────────────────┘
    
    ┌─────────────┐     ┌──────────────────────────────┐
    │ Máy tính    │◀────│ MQTT/Webserver: Đồ thị thời  │
    │ (Giám sát)  │     │ gian thực, tuning PID từ xa   │
    └─────────────┘     └──────────────────────────────┘
```

---

# 2. Chốt linh kiện chính thức

## 2.1 Bảng linh kiện (CHỐT HẠ — Không thay đổi)

| # | Linh kiện | Model cụ thể | Điện áp | Vai trò | Lý do chọn |
|---|---|---|---|---|---|
| 1 | **Vi điều khiển** | ESP32-S3 WROOM-1 N16R8 | 3.3V | Não bộ xử lý chính | Dual-core 240MHz, 2 bộ I2C, WiFi/BLE, PSRAM 8MB, OTA update |
| 2 | **Cảm biến** | VL53L0X (ToF Laser) | 3.3V | Đo vị trí bóng | Laser hồng ngoại 940nm, 30Hz, I2C, native 3.3V, không cần level shifter |
| 3 | **Driver motor** | MKS EMM42 v4.0 | 7-32V | Điều khiển step | Closed-loop (encoder 14-bit), chống mất bước, PID nội bộ |
| 4 | **Động cơ** | NEMA 17 (1.8°/step) | 12/24V | Nghiêng thanh | Gắn sẵn trên MKS EMM42, moment xoắn đủ cho tải nhẹ |
| 5 | **Màn hình** | SSD1306 OLED 0.96" I2C | 3.3V | Hiển thị trạng thái | Nhỏ gọn, I2C, tiêu thụ thấp, hiển thị rõ ràng |
| 6 | **Nguồn motor** | Adapter 12V/10A | 12V | Cấp nguồn driver | Đủ dòng cho NEMA 17, an toàn cho prototype |
| 7 | **Bóng** | Bóng bàn trắng (40mm) | — | Vật thể cân bằng | Nhẹ, phản xạ tia hồng ngoại tốt |

## 2.2 Quá trình chọn linh kiện

### Tại sao chọn VL53L0X thay vì HC-SR04?

| Tiêu chí | VL53L0X (✅ Đã chọn) | HC-SR04 (❌ Loại bỏ) |
|---|---|---|
| Tốc độ đo | ~30Hz (đủ nhanh cho PID) | ~20Hz (chậm hơn) |
| Góc tia | 25° (hẹp, chính xác) | 30-45° (rộng, **dội vách máng gây echo ảo**) |
| Mức logic | **3.3V native** | 5V (cần level shifter, **nếu không cháy ESP32**) |
| Nhiễu từ máng | Ít (tia thẳng) | **Rất nhiều** (sóng siêu âm dội vách) |
| Kết nối | I2C (2 dây) | Trigger + Echo (2 dây + level shifter) |

> **Kết luận:** SR04 KHÔNG PHÙ HỢP cho máng trượt hẹp. Sóng siêu âm sẽ dội vách máng liên tục tạo tín hiệu nhiễu giả.

### Tại sao chọn MKS EMM42 + NEMA 17 thay vì Servo MG996R?

Ban đầu dự kiến dùng servo RC MG996R (góc quay liên tục, điều khiển bằng PWM). Tuy nhiên đã **chuyển sang MKS EMM42** vì:

| Tiêu chí | MKS EMM42 + NEMA 17 (✅ Đã chọn) | MG996R (❌ Loại bỏ) |
|---|---|---|
| Độ chính xác | 0.022° (encoder 14-bit) | ~1° (feedback potentiometer) |
| Chống mất bước | ✅ Closed-loop | ❌ Open-loop |
| Moment xoắn | Cao, ổn định | Thấp, giảm theo tải |
| Tốc độ phản hồi | Rất nhanh (20kHz loop) | Chậm (~50Hz PWM) |
| Điều khiển | STP/DIR/EN (linh hoạt) | PWM góc (đơn giản nhưng hạn chế) |

> **Kết luận:** MKS EMM42 chính xác hơn 50 lần và có closed-loop feedback. Phù hợp hơn nhiều cho điều khiển chính xác vị trí beam.

---

# 3. Sơ đồ nối dây chi tiết

## 3.1 Bản đồ GPIO chính thức của ESP32-S3 N16R8

| GPIO | Chức năng | Nối đến | Loại tín hiệu | Ghi chú |
|------|-----------|---------|---------------|---------|
| GPIO4 | **STP** (Step) | MKS EMM42 → STP | Output PWM | Xung bước điều khiển góc quay |
| GPIO5 | **DIR** (Direction) | MKS EMM42 → DIR | Output Digital | HIGH = CW, LOW = CCW |
| GPIO6 | **EN** (Enable) | MKS EMM42 → EN | Output Digital | LOW = Enable (thường) |
| GPIO8 | **SDA** (I2C Bus 2) | OLED SSD1306 → SDA | I2C Data | Bus riêng cho màn hình |
| GPIO9 | **SCL** (I2C Bus 2) | OLED SSD1306 → SCL | I2C Clock | Bus riêng cho màn hình |
| GPIO14 | **SDA** (I2C Bus 1) | VL53L0X → SDA | I2C Data | Bus riêng cho cảm biến |
| GPIO15 | **SCL** (I2C Bus 1) | VL53L0X → SCL | I2C Clock | Bus riêng cho cảm biến |
| 3.3V | **COM** | MKS EMM42 → COM | Mức logic | ⚠️ BẮT BUỘC 3.3V, không 5V |
| 3.3V | **VCC** | VL53L0X, OLED → VCC | Nguồn | Cấp nguồn module |
| GND | **GND chung** | Tất cả module → GND | Ground | GND phải nối chung |

> ⚠️ **GPIO 26-32**: KHÔNG SỬ DỤNG — kết nối nội bộ với Flash/PSRAM trên module N16R8.

## 3.2 Sơ đồ kết nối vật lý

```
    ┌────────────────────── ESP32-S3 N16R8 ──────────────────────┐
    │                                                             │
    │              ┌── MKS EMM42 v4.0 ──┐                        │
    │  3.3V ──────▶│ COM                │                        │
    │  GPIO4 ─────▶│ STP (Step)         │                        │
    │  GPIO5 ─────▶│ DIR (Direction)    │                        │
    │  GPIO6 ─────▶│ EN  (Enable)      │      ┌──── NEMA 17 ───┐│
    │  GND ───────▶│ GND               │      │ A+ A- B+ B-    ││
    │              │              12V ──▶ V+   │ (4 dây motor)  ││
    │              │              GND ──▶ GND  └────────────────┘│
    │              └────────────────────┘                        │
    │                                                             │
    │    ┌── I2C Bus 1 (Cảm biến — Ưu tiên cao) ──┐             │
    │    │                                          │             │
    │  GPIO14 (SDA) ──▶ VL53L0X SDA                │             │
    │  GPIO15 (SCL) ──▶ VL53L0X SCL                │             │
    │  3.3V ──────────▶ VL53L0X VCC                │             │
    │  GND ───────────▶ VL53L0X GND                │             │
    │    └─────────────────────────────────────────┘             │
    │                                                             │
    │    ┌── I2C Bus 2 (Màn hình — Không cần nhanh) ──┐         │
    │    │                                              │         │
    │  GPIO8  (SDA) ──▶ OLED SSD1306 SDA               │         │
    │  GPIO9  (SCL) ──▶ OLED SSD1306 SCL               │         │
    │  3.3V ──────────▶ OLED SSD1306 VCC               │         │
    │  GND ───────────▶ OLED SSD1306 GND               │         │
    │    └─────────────────────────────────────────────┘         │
    └─────────────────────────────────────────────────────────────┘

    NGUỒN:
    ┌────────────────┐
    │ Adapter 12V    │──▶ V+ / GND cọc to trên MKS EMM42
    │ (hoặc 24V)     │    (KHÔNG nối vào ESP32!)
    └────────────────┘
    
    ┌────────────────┐
    │ USB-C / 5V     │──▶ ESP32-S3 Dev Board (tự hạ về 3.3V nội bộ)
    └────────────────┘
```

## 3.3 Tại sao tách 2 Bus I2C?

ESP32-S3 có **2 bộ I2C hardware** (Wire và Wire1). Chúng ta PHẢI tách riêng vì:

```
BUS CHUNG (❌ Không nên):
  VL53L0X ──┬── SDA/SCL ──── OLED
             │
  Khi OLED đang vẽ (15-20ms), nó KHOÁ bus I2C
  → VL53L0X không đọc được → PID mất mẫu → hệ thống giật

BUS RIÊNG (✅ Làm theo cách này):
  VL53L0X ──── Bus I2C 1 (GPIO14/15) ──── Đọc liên tục 30-100Hz
  OLED    ──── Bus I2C 2 (GPIO8/9)   ──── Vẽ chậm 5-10Hz
  → Hai thiết bị KHÔNG BAO GIỜ chặn nhau
```

---

# 4. Thuật toán ĐO LƯỜNG — Bộ lọc cảm biến

## 4.1 Tại sao phải lọc nhiễu?
VL53L0X đo bằng laser, nhưng tín hiệu thô (raw) luôn có nhiễu:
- **Spike noise**: Tia laser phản xạ từ cạnh bóng → giá trị nhảy đột ngột
- **Gaussian noise**: Nhiễu điện tử → dao động ±5-15mm quanh giá trị thực
- **Dropout**: Mất tín hiệu → trả về 0mm hoặc 8190mm
- **Multipath**: Laser dội nhiều lần trong máng → giá trị sai

Nếu đưa dữ liệu thô vào PID → motor sẽ giật liên tục, bóng không thể cân bằng.

## 4.2 Pipeline lọc 5 tầng (Đang triển khai trong `VL53L0X.ino`)

```
Raw (mm) → [1. Validation Gate] → [2. Hampel Filter] → [3. Median Filter]
         → [4. Adaptive 2D Kalman] → [5. Rate Limiter] → Clean Output (mm)
```

### Tầng 1: Validation Gate (Cổng kiểm tra)
- **Mục đích**: Loại bỏ các giá trị chắc chắn SAI
- **Logic**: Giữ lại nếu `20mm ≤ khoảng cách ≤ 1200mm` VÀ `RangeStatus == 0 hoặc 4`
- **Kết quả**: Chặn ~5-10% số đọc trong điều kiện bình thường

### Tầng 2: Hampel Filter (Lọc ngoại lai MAD)
- **Mục đích**: Phát hiện và thay thế outlier theo ngữ cảnh
- **Cách hoạt động**: Duy trì cửa sổ 5 mẫu gần nhất. Tính median và MAD (Median Absolute Deviation). Nếu giá trị mới lệch > 2.5 × MAD so với median → thay bằng median
- **Ưu điểm so với range checking**: Phát hiện được giá trị bất thường TRONG khoảng hợp lệ (VD: bóng ở 200mm mà nhảy lên 500mm vẫn trong 20-1200mm)
- **Breakdown point**: 50% — ngay cả khi 49% dữ liệu bị nhiễu vẫn hoạt động

### Tầng 3: Median Filter (Lọc trung vị)
- **Mục đích**: Triệt tiêu impulse noise còn sót
- **Cách hoạt động**: Cửa sổ trượt 5 mẫu, lấy giá trị trung vị
- **Tại sao dùng Median thay vì Mean?**
  - Mean [200, 201, 199, 800, 200] = **320** ← SAI!
  - Median [200, 201, 199, 800, 200] = **200** ← ĐÚNG!
- **Ưu điểm**: Bảo toàn biên (edge preserving) — khi bóng di chuyển nhanh, không bị trễ phản hồi

### Tầng 4: Adaptive 2D Kalman Filter (Bộ lọc Kalman thích nghi)
- **Đây là bộ lọc mạnh nhất và phức tạp nhất**
- **2D** = ước lượng đồng thời 2 đại lượng: Vị trí (mm) + Vận tốc (mm/s)
- **Adaptive** = Tự động điều chỉnh mức tin cậy (R) dựa trên innovation variance
- **Mô hình**: Constant Velocity — giả định bóng chuyển động đều trong khoảng thời gian ngắn
- **Kalman Gain (K)**: 
  - K → 0: Tin mô hình dự đoán (sensor đang nhiễu)
  - K → 1: Tin sensor (dữ liệu ổn định)
- **Bonus**: Cho ta vận tốc bóng miễn phí (dùng cho thành phần D trong PID)

### Tầng 5: Rate Limiter (Giới hạn tốc độ)
- **Mục đích**: Ràng buộc vật lý cuối cùng
- **Logic**: Bóng không thể dịch chuyển > 30mm/cycle (ở 30Hz). Nếu vượt → cắt bớt
- **An toàn**: Bắt được mọi glitch cực hiếm mà 4 tầng trước bỏ sót

## 4.3 So sánh với các phương pháp lọc khác

| Phương pháp | Độ phức tạp | Hiệu quả | Tình trạng dùng |
|---|---|---|---|
| **Trung bình cộng (Mean)** | Rất thấp | Kém (bị ảnh hưởng outlier) | ❌ Không dùng |
| **Moving Average** | Thấp | Trung bình (gây trễ tín hiệu) | ❌ Không dùng |
| **Exponential Moving Average (EMA)** | Thấp | Khá tốt (đơn giản, ít trễ) | ⚠️ Có thể thêm vào nếu cần |
| **Median Filter** | Trung bình | Tốt (chống spike) | ✅ Đang dùng (Tầng 3) |
| **Hampel Filter** | Trung bình | Rất tốt (adaptive outlier) | ✅ Đang dùng (Tầng 2) |
| **Kalman Filter** | Cao | Xuất sắc (optimal estimator) | ✅ Đang dùng (Tầng 4) |
| **Complementary Filter** | Thấp | Tốt (cho sensor fusion 2 nguồn) | ❌ Chỉ có 1 sensor, không cần |
| **Particle Filter** | Rất cao | Xuất sắc (phi tuyến) | ❌ Quá nặng cho ESP32 thời gian thực |

> **Kết luận**: Pipeline 5 tầng hiện tại là cực kỳ mạnh mẽ cho ứng dụng Ball and Beam. Không cần thay đổi phương pháp lọc, chỉ cần tinh chỉnh hệ số.

---

# 5. Thuật toán ĐIỀU KHIỂN — PID / Fuzzy Logic và các phương án khác

## 5.1 Giai đoạn 1: Bộ điều khiển PID

### PID là gì?
PID (Proportional - Integral - Derivative) là bộ điều khiển phổ biến nhất trong kỹ thuật điều khiển. Nó tính toán tín hiệu điều khiển dựa trên 3 thành phần:

```
u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt

Trong đó:
  e(t) = setpoint − measured_position  (Sai số)
  Kp   = Hệ số tỉ lệ      → Phản ứng nhanh với sai số
  Ki   = Hệ số tích phân    → Xóa sai số xác lập (steady-state error)
  Kd   = Hệ số vi phân      → Giảm dao động, tăng ổn định
```

### Tại sao chọn PID cho giai đoạn 1?
- Đơn giản, dễ hiểu, dễ triển khai trên ESP32
- Tài liệu học tập phong phú
- Đủ mạnh cho hệ thống Ball and Beam cơ bản
- Là nền tảng để hiểu trước khi chuyển sang thuật toán nâng cao

### Hệ số PID khởi đầu để tuning

| Tham số | Giá trị bắt đầu | Phạm vi tuning | Vai trò |
|---------|------------------|-----------------|---------| 
| Kp | 2.0 | 0.5 – 10.0 | Phản ứng tỉ lệ với error |
| Ki | 0.0 (bắt đầu từ 0) | 0.0 – 1.0 | Loại bỏ steady-state error |
| Kd | 1.0 | 0.1 – 5.0 | Giảm overshoot, giảm dao động |
| Ts | 10ms (100Hz) | 5 – 20ms | Chu kỳ lấy mẫu |

### Quy trình tuning PID
```
Bước 1: Đặt Ki = 0, Kd = 0.
        Tăng Kp từ 0 cho đến khi bóng BẮT ĐẦU dao động đều → ghi lại Kp_critical
Bước 2: Đặt Kp = 0.6 × Kp_critical
        Tăng Kd dần → mục tiêu: overshoot < 20%
Bước 3: Tăng Ki RẤT NHỎ (0.01 mỗi lần) → xóa steady-state error
        Cẩn thận: Ki quá lớn → integral windup → mất ổn định
Bước 4: Fine-tune lại, test với nhiều setpoint và thử đẩy bóng nhẹ (disturbance)
```

### Các kỹ thuật bổ trợ PID
- **Anti-Windup**: Giới hạn integral term khi actuator bão hòa (góc max)
- **Derivative Filtering**: Lọc thấp qua EMA trên thành phần D để tránh khuếch đại nhiễu
- **Setpoint Ramping**: Thay đổi setpoint từ từ thay vì nhảy đột ngột

## 5.2 Giai đoạn 2: Fuzzy Logic Controller (Nâng cấp)

### Fuzzy Logic là gì?
Thay vì dùng công thức toán chính xác như PID, Fuzzy Logic dùng **quy tắc ngôn ngữ tự nhiên**:
- "NẾU bóng ở xa bên trái VÀ đang lao nhanh sang trái THÌ nghiêng MẠNH sang phải"
- "NẾU bóng gần đúng vị trí VÀ gần như đứng yên THÌ giữ nguyên"

### Tại sao nâng cấp lên Fuzzy?
| Tiêu chí | PID | Fuzzy Logic |
|----------|-----|-------------|
| Cần mô hình toán | Có | **KHÔNG** |
| Xử lý phi tuyến | Kém | **Tốt** |
| Tuning | Thử và sai, khó | Quy tắc trực giác |
| Robustness | Trung bình | **Cao** |
| Tài nguyên CPU | Thấp | Trung bình |

### Cấu trúc Fuzzy Controller cho Ball and Beam
- **2 đầu vào**: Position Error (e) và Velocity Error (ė)
- **1 đầu ra**: Beam Angle Change (Δθ)
- **5 membership functions** mỗi biến: NB, NS, ZE, PS, PB
- **25 rules** trong bảng luật (Rule Base)
- **Defuzzification**: Centroid (Center of Gravity)
- **Thư viện ESP32**: eFLL (Embedded Fuzzy Logic Library)

## 5.3 So sánh tổng hợp các thuật toán điều khiển khác

| Thuật toán | Độ phức tạp | Hiệu quả cho Ball & Beam | Trạng thái |
|---|---|---|---|
| **PID** | Thấp | Tốt (đủ dùng cơ bản) | ✅ Giai đoạn 1 |
| **Fuzzy Logic (Mamdani)** | Trung bình | Rất tốt (phi tuyến) | 📋 Giai đoạn 2 |
| **LQR** (Linear Quadratic Regulator) | Cao | Xuất sắc (optimal) | ⚠️ Cần mô hình toán chính xác |
| **MPC** (Model Predictive Control) | Rất cao | Xuất sắc (nhìn trước) | ❌ Quá nặng cho ESP32 |
| **Sliding Mode Control** | Cao | Tốt (robust) | ⚠️ Chattering vấn đề |
| **Neural Network / AI** | Cực cao | Tiềm năng lớn | 📋 Giai đoạn 3 (cần camera) |
| **Hybrid PID-Fuzzy** | Trung bình-Cao | Rất tốt | ⚠️ Biến thể hay — dùng Fuzzy auto-tune PID |
| **State-Space + Observer** | Cao | Xuất sắc | ⚠️ Cần linearization |

> **Kết luận lộ trình**: PID (đơn giản, học nền tảng) → Fuzzy (xử lý phi tuyến tốt hơn) → AI/Camera (tương lai xa). Đây là lộ trình hợp lý nhất vừa học vừa làm.

---

# 6. Hiển thị & Giám sát — LabVIEW vs MQTT vs Webserver

## 6.1 Phương án ban đầu: LabVIEW (❌ Loại bỏ)
- **Ưu điểm**: Giao diện kéo thả, vẽ đồ thị real-time tốt
- **Nhược điểm đã gặp**:
  - Phần mềm nặng, tốn license
  - Chỉ chạy trên máy có cài LabVIEW
  - Kết nối Serial → chỉ hoạt động khi cắm USB trực tiếp
  - Không thể giám sát từ xa (điện thoại, máy khác)
- **Trạng thái**: Đã có file `.vi` trong thư mục `BaoCao/1_LabView/` (giữ lại tham khảo)

## 6.2 Phương án MQTT qua HiveMQ (📋 Đề xuất — Phương án A)

```
ESP32-S3 ──WiFi──▶ HiveMQ Cloud Broker ──▶ Dashboard Web / App điện thoại
                        │
                        ▼
                   Topic cấu trúc:
                   ballbeam/data    → {pos, vel, setpoint, pid_output}
                   ballbeam/config  → {kp, ki, kd, setpoint}  (tuning từ xa)
                   ballbeam/status  → {state, error, uptime}
```

**Ưu điểm**:
- Giám sát từ bất cứ đâu (điện thoại, máy tính khác)
- ESP32-S3 có WiFi sẵn, thư viện MQTT rất ổn định
- HiveMQ Cloud có gói miễn phí (100 connections)
- Có thể dùng Node-RED hoặc Grafana để vẽ dashboard chuyên nghiệp
- Tuning PID từ xa (gửi Kp, Ki, Kd qua MQTT topic)

**Nhược điểm**:
- Cần WiFi ổn định
- Latency ~50-200ms (không ảnh hưởng hiển thị, nhưng không dùng cho điều khiển)
- Phụ thuộc server bên ngoài

## 6.3 Phương án Webserver trên ESP32 (📋 Đề xuất — Phương án B)

```
ESP32-S3 ──WiFi──▶ Web Server nội bộ (192.168.x.x)
                        │
                        ▼
                   Trình duyệt bất kỳ trên cùng mạng WiFi
                   → Trang web hiển thị đồ thị + nút tuning PID
                   → WebSocket cho real-time data streaming
```

**Ưu điểm**:
- Không phụ thuộc server bên ngoài
- Hoạt động offline (chỉ cần cùng mạng WiFi)
- Tốc độ nhanh hơn MQTT (~10-50ms qua WebSocket)
- Tự thiết kế giao diện hoàn toàn

**Nhược điểm**:
- Phải tự code giao diện web (HTML/JS/CSS)
- Chỉ truy cập được trên cùng mạng WiFi
- ESP32 xử lý web server thêm → tốn tài nguyên

## 6.4 Đề xuất: Dùng cả hai!

| Mục đích | Phương án | Lý do |
|---|---|---|
| Hiển thị trực tiếp trên board | OLED SSD1306 | Luôn có, không cần WiFi |
| Giám sát & tuning PID | Webserver trên ESP32 | Nhanh, offline, tự chủ |
| Logging dài hạn & từ xa | MQTT (HiveMQ) | Ghi lại toàn bộ phiên tuning |

> **Gợi ý thực tế**: Bắt đầu bằng **Webserver + WebSocket** trước (tự chủ hoàn toàn). Sau khi ổn định, thêm MQTT nếu cần giám sát từ xa.

---

# 7. Tiến trình dự án — Đã làm gì, chưa làm gì

## 7.1 ✅ Đã hoàn thành

| # | Hạng mục | Chi tiết | File liên quan |
|---|---|---|---|
| 1 | Chọn linh kiện | Chốt: ESP32-S3, VL53L0X, MKS EMM42, OLED 0.96" | Mục 2 tài liệu này |
| 2 | Sơ đồ chân GPIO | Phân công đầy đủ 2 bus I2C + STP/DIR/EN | Mục 3 tài liệu này |
| 3 | Code test cảm biến VL53L0X | 823 dòng, bộ lọc 5 tầng, hoạt động ổn ở ~10cm | `BaoCao/2_main/VL53L0X/VL53L0X.ino` |
| 4 | Tài liệu kỹ thuật VL53L0X | 650 dòng, giải thích chi tiết từ nguyên lý đến code | `BaoCao/readme/README.sensor.md` |
| 5 | Báo cáo lý thuyết mở đầu | Tổng quan PID, mục tiêu, phương pháp | `BaoCao/NghienCuu.docx` |
| 6 | Thiết kế lý thuyết PID | Hệ số khởi đầu, quy trình tuning, anti-windup | `.agent/context/algorithm/pid_parameters.md` |
| 7 | Thiết kế lý thuyết Fuzzy | 25 rules, membership functions, eFLL code | `.agent/context/algorithm/fuzzy_logic_rules.md` |
| 8 | Chuẩn kiến trúc firmware | Layered Architecture, FreeRTOS tasks, State Machine | `.agent/context/coding_standard/firmware_architecture.md` |
| 9 | File LabVIEW (tham khảo) | Serial.vi và ViewProject.vi | `BaoCao/1_LabView/` |

## 7.2 ❌ Chưa làm (cần thực hiện)

| # | Hạng mục | Mức độ | Ghi chú |
|---|---|---|---|
| 1 | **Sửa code VL53L0X.ino** | 🔴 Quan trọng | Xem Mục 8 chi tiết lỗi |
| 2 | **Code firmware chính (PlatformIO)** | 🔴 Quan trọng | FreeRTOS: Task Sensor, Task PID, Task Motor, Task Display |
| 3 | **Bản vẽ cơ khí CAD** | 🟡 Trung bình | Thanh 40cm, vị trí trục motor, giá đỡ sensor |
| 4 | **Lắp ráp phần cứng thực tế** | 🟡 Trung bình | Nối dây theo sơ đồ Mục 3 |
| 5 | **Code PID controller** | 🔴 Quan trọng | Module `svc_pid.cpp` theo kiến trúc phân tầng |
| 6 | **Code điều khiển Step Motor** | 🔴 Quan trọng | Module `drv_step_motor.cpp` |
| 7 | **Giao diện Webserver / MQTT** | 🟡 Trung bình | Dashboard giám sát + tuning PID |
| 8 | **Tuning PID trên hệ thống thực** | 🟡 Trung bình | Chỉ làm được sau khi lắp xong phần cứng |
| 9 | **Code Fuzzy Logic** | 🟢 Sau này | Giai đoạn 2, chỉ làm khi PID đã ổn |
| 10 | **Tích hợp Camera + AI** | ⚪ Tương lai xa | Giai đoạn 3, chưa cần thiết |

---

# 8. Ghi chú lỗi cần sửa trong code VL53L0X.ino

> ⚠️ **LƯU Ý**: Code hiện tại HOẠT ĐỘNG TỐT cho mục đích TEST cảm biến riêng lẻ. Các lỗi dưới đây chỉ xuất hiện khi TÍCH HỢP vào hệ thống hoàn chỉnh.

## Lỗi #1: Dùng chung 1 Bus I2C — PHẢI TÁCH (Ưu tiên cao)
**File**: `VL53L0X.ino`, dòng 34-36  
**Hiện tại**:
```cpp
#define I2C_SDA_PIN  8     // ← Cả VL53L0X và OLED chung bus
#define I2C_SCL_PIN  9
```
**Cần sửa thành**: 2 bus riêng biệt:
- VL53L0X: SDA=GPIO14, SCL=GPIO15 (Wire1)
- OLED: SDA=GPIO8, SCL=GPIO9 (Wire)

**Lý do**: Khi OLED vẽ (15-20ms), nó khoá bus → cảm biến không đọc được → vòng PID mất mẫu → hệ thống giật. Trong code test riêng lẻ thì không thấy vì chưa có PID.

## Lỗi #2: Chế độ đo mặc định — Cần chuyển LONG_RANGE (Ưu tiên cao)
**File**: `VL53L0X.ino`, dòng 755  
**Hiện tại**:
```cpp
vl53.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
```
**Cần sửa thành**:
```cpp
vl53.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
```
**Lý do**: Chế độ DEFAULT tối ưu cho ≤20cm trên bề mặt cong. LONG_RANGE tăng timing budget, giúp thu tia phản xạ yếu từ bóng ở 30-40cm. Tốc độ giảm từ ~30Hz xuống ~20Hz nhưng đủ dùng cho PID 50-100Hz (PID có thể dùng giá trị cuối cùng nếu sensor chưa update).

## Lỗi #3: Calibration offset hardcode — Cần dynamic (Ưu tiên thấp)
**File**: `VL53L0X.ino`, dòng 61  
**Hiện tại**:
```cpp
#define CALIBRATION_OFFSET_MM  -10.0f     // ← Giá trị cố định
```
**Cần cải thiện**: Thêm chế độ auto-calibration khi khởi động. Đặt vật chuẩn ở vị trí biết trước → tự tính offset. Hoặc lưu offset vào EEPROM/NVS để không mất khi reset.

## Lỗi #4: Biến `sensor` là global struct — Cần encapsulate (Ưu tiên thấp)
**File**: `VL53L0X.ino`, dòng 182  
**Hiện tại**: `SensorData sensor;` là biến global.  
**Cần cải thiện**: Khi chuyển sang FreeRTOS multi-task, nhiều task truy cập cùng lúc → race condition. Cần dùng **Queue** hoặc **Mutex** để bảo vệ.

## Lỗi #5: Hàm `oled.display()` là blocking — Cần chuyển DMA (Ưu tiên thấp)
**File**: `VL53L0X.ino`, dòng 624  
**Hiện tại**: `oled.display()` gửi toàn bộ framebuffer (1KB) qua I2C → chiếm bus ~7ms ở 400kHz.  
**Khi tách bus I2C riêng**: Vấn đề này giảm đi nhiều. Nhưng nếu muốn tối ưu hơn nữa, có thể dùng async I2C DMA.

---

# 9. Lộ trình phát triển tiếp theo

## Phase 0: Chuẩn bị (⬅️ ĐANG Ở ĐÂY)
- [x] Nghiên cứu lý thuyết PID, Fuzzy Logic
- [x] Chọn và chốt linh kiện
- [x] Code test cảm biến VL53L0X
- [x] Tài liệu kỹ thuật đầy đủ
- [ ] Thiết kế bản vẽ cơ khí (bạn tự vẽ/thiết kế)

## Phase 1: Lắp ráp & Test phần cứng riêng lẻ
- [ ] Lắp ráp mô hình cơ khí (thanh 40cm, khung, trục motor)
- [ ] Nối dây theo sơ đồ Mục 3
- [ ] Sửa code VL53L0X.ino (theo Mục 8)
- [ ] Test VL53L0X trên thanh thực tế với bóng có dán băng phản quang
- [ ] Test step motor MKS EMM42 (quay nhẹ, kiểm tra STP/DIR)
- [ ] Test OLED hiển thị

## Phase 2: Firmware chính + PID
- [ ] Tạo project PlatformIO (`BallAndBeam/code/`)
- [ ] Viết config.h (định nghĩa chân theo sơ đồ chốt)
- [ ] Viết HAL layer (hal_i2c, hal_gpio, hal_timer)
- [ ] Viết Driver layer (drv_vl53l0x, drv_step_motor, drv_ssd1306)
- [ ] Viết Service layer (svc_pid)
- [ ] Viết Application layer (app_main — State Machine)
- [ ] Tạo FreeRTOS tasks: Sensor, Control, Motor, Display
- [ ] Tuning PID trên hệ thống thực

## Phase 3: Giám sát & Nâng cấp
- [ ] Tạo Webserver/WebSocket dashboard trên ESP32
- [ ] (Tùy chọn) Thêm MQTT qua HiveMQ
- [ ] Code Fuzzy Logic controller
- [ ] So sánh PID vs Fuzzy trên cùng hệ thống
- [ ] Viết báo cáo kết quả thực nghiệm

## Phase 4: Mở rộng (Tương lai xa)
- [ ] Tích hợp camera ESP32-CAM
- [ ] Training AI cân bằng tự động
- [ ] OTA firmware update qua WiFi

---

> **Tài liệu này là "sổ tay cá nhân" của dự án Ball and Beam.**  
> **Cập nhật mỗi khi có tiến triển mới.**  
> **Ngày tạo**: 24/03/2026  
> **Tác giả**: [Tên bạn] — Với sự hỗ trợ của AI Assistant
