# 📡 CẢM BIẾN VL53L0X - TÀI LIỆU KỸ THUẬT CHI TIẾT
## Dự án Ball and Beam | ESP32-S3


## Mục lục
1. [Giới thiệu cảm biến VL53L0X](#1-giới-thiệu-cảm-biến-vl53l0x)
2. [Nguyên lý hoạt động - Time of Flight (ToF)](#2-nguyên-lý-hoạt-động---time-of-flight-tof)
3. [Giao thức I2C](#3-giao-thức-i2c---giải-thích-từ-a-đến-z)
4. [ESP32-S3 đọc dữ liệu VL53L0X như thế nào?](#4-esp32-s3-đọc-dữ-liệu-vl53l0x-như-thế-nào)
5. [Hệ thống lọc nhiễu 5 tầng (Filter Pipeline)](#5-hệ-thống-lọc-nhiễu-5-tầng-filter-pipeline)
6. [Ứng dụng thực tế và công nghiệp](#6-ứng-dụng-thực-tế-và-công-nghiệp)
7. [Sơ đồ kết nối phần cứng](#7-sơ-đồ-kết-nối-phần-cứng)
8. [Hướng dẫn sử dụng code test](#8-hướng-dẫn-sử-dụng-code-test)
9. [Format dữ liệu Serial (cho LabVIEW)](#9-format-dữ-liệu-serial-cho-labview)
10. [Tài liệu tham khảo](#10-tài-liệu-tham-khảo)

---

## 1. Giới thiệu cảm biến VL53L0X

### 1.1 VL53L0X là gì?

**VL53L0X** là cảm biến đo khoảng cách sử dụng công nghệ **Time of Flight (ToF)** — nghĩa là đo thời gian bay của ánh sáng laser. Cảm biến này do **STMicroelectronics** (Pháp/Ý) sản xuất, sử dụng công nghệ độc quyền **FlightSense™**.

Nói đơn giản: VL53L0X bắn ra một tia laser, tia laser đập vào vật thể rồi dội lại, cảm biến đo thời gian tia laser bay đi và bay về, từ đó tính ra khoảng cách.

### 1.2 Thông số kỹ thuật

| Thông số | Giá trị |
|---|---|
| Nhà sản xuất | STMicroelectronics |
| Công nghệ | Time of Flight (ToF) - FlightSense™ |
| Nguồn sáng | Laser VCSEL 940nm (hồng ngoại, không nhìn thấy) |
| Tầm đo | 30mm → 2000mm (điều kiện lý tưởng) |
| Tầm đo thực tế tin cậy | 30mm → 1200mm |
| Độ chính xác | ±3% trong điều kiện tiêu chuẩn |
| Góc đo (FoV) | 25° (hình nón) |
| Giao tiếp | I2C (địa chỉ mặc định: 0x29) |
| Tốc độ I2C | Lên đến 400kHz (Fast Mode) |
| Điện áp hoạt động | 2.6V → 3.5V |
| Dòng tiêu thụ | ~10mA khi đang đo |
| Thời gian đo | 20ms (tốc độ cao) → 200ms (độ chính xác cao) |
| Kích thước chip | 4.4 x 2.4 x 1.0 mm |
| An toàn laser | Class 1 (an toàn cho mắt người) |

### 1.3 Tại sao chọn VL53L0X cho Ball and Beam?

- **Tốc độ đo nhanh** (~30Hz): Đủ nhanh cho điều khiển PID thời gian thực.
- **Không tiếp xúc**: Không cần chạm vào quả bóng → không gây ma sát.
- **Kích thước nhỏ gọn**: Dễ gắn lên thanh beam.
- **Giao tiếp I2C**: Chỉ cần 2 dây dữ liệu, chia sẻ bus với OLED.
- **Độ chính xác đủ dùng**: ±3% là chấp nhận được cho điều khiển vị trí.

---

## 2. Nguyên lý hoạt động - Time of Flight (ToF)

### 2.1 Ví dụ dễ hiểu: Nguyên lý tiếng vang

Hãy tưởng tượng bạn đứng trước một ngọn núi và hét lên **"Aaaa!"**. Tiếng của bạn bay đến ngọn núi, đập vào vách đá rồi dội lại thành tiếng vang. Nếu bạn đo được thời gian từ lúc hét đến lúc nghe tiếng vang, bạn có thể tính khoảng cách đến ngọn núi.

**VL53L0X hoạt động giống hệt như vậy, nhưng thay vì dùng âm thanh, nó dùng ÁNH SÁNG LASER.**

| So sánh | Tiếng vang | VL53L0X |
|---|---|---|
| Tín hiệu | Âm thanh | Laser hồng ngoại 940nm |
| Tốc độ | 343 m/s (âm thanh) | 299,792,458 m/s (ánh sáng) |
| Phát | Miệng người | Diode laser VCSEL |
| Thu | Tai người | Mảng SPAD detector |
| Tính khoảng cách | d = v × t / 2 | d = c × t / 2 |

### 2.2 Chi tiết quá trình đo (6 bước)

```
┌─────────────────────────────────────────────────────────┐
│                    QUÁ TRÌNH ĐO VL53L0X                 │
│                                                         │
│   ┌──────────┐                        ┌──────────┐      │
│   │  VCSEL   │  ──── Xung laser ────▶ │ Vật thể  │      │
│   │  Laser   │                        │ (bóng)   │      │
│   │  940nm   │  ◀── Phản xạ ──────── │          │      │
│   └──────────┘                        └──────────┘      │
│        ↕                                                 │
│   ┌──────────┐                                          │
│   │  SPAD    │  Thu nhận photon phản xạ                 │
│   │  Array   │                                          │
│   └──────────┘                                          │
│        ↓                                                 │
│   ┌──────────┐                                          │
│   │  Chip    │  Tính thời gian bay → Khoảng cách       │
│   │  xử lý  │  d = c × t / 2                           │
│   └──────────┘                                          │
└─────────────────────────────────────────────────────────┘
```

#### Bước 1: VCSEL phát xung laser
- **VCSEL** (Vertical-Cavity Surface-Emitting Laser) là loại diode laser đặc biệt.
- Phát ra các xung laser cực ngắn (nanosecond = 1 phần tỷ giây) ở bước sóng **940nm** (hồng ngoại).
- Bước sóng 940nm **không nhìn thấy bằng mắt thường** → an toàn cho mắt (Class 1 theo tiêu chuẩn IEC 60825-1).
- Tia laser phát ra theo hình nón với góc mở khoảng **25°**.

#### Bước 2: Laser chiếu lên vật thể
- Chùm laser hồng ngoại bay đến vật thể (trong dự án này là quả bóng trên thanh beam).
- Tốc độ ánh sáng: **c = 299,792,458 m/s** (gần 300 triệu mét mỗi giây!).

#### Bước 3: Ánh sáng phản xạ
- Một phần ánh sáng laser đập vào bề mặt vật thể và **phản xạ ngược lại** cảm biến.
- Lượng ánh sáng phản xạ phụ thuộc vào:
  - **Màu sắc** bề mặt: trắng phản xạ nhiều, đen hấp thụ nhiều.
  - **Khoảng cách**: càng xa, ánh sáng phản xạ càng yếu.
  - **Góc**: chiếu thẳng phản xạ tốt hơn chiếu xiên.

#### Bước 4: Mảng SPAD thu nhận photon
- **SPAD** (Single Photon Avalanche Diode) là loại photodiode siêu nhạy.
- SPAD có khả năng phát hiện **TỪNG PHOTON ĐƠN LẺ** (1 hạt ánh sáng).
- VL53L0X dùng một **mảng (array) 16x16 = 256 SPAD**, nhưng chỉ kích hoạt một phần tùy điều kiện.
- Đây là công nghệ **FlightSense™** độc quyền của STMicroelectronics - hiện nay chỉ có ST mới có khả năng sản xuất chip ToF nhỏ gọn đến mức này.

#### Bước 5: Chip xử lý tính thời gian bay
- Chip xử lý tích hợp bên trong VL53L0X đo **thời gian chính xác** từ lúc phát laser đến lúc SPAD nhận được photon phản xạ.
- Thời gian này gọi là **Time of Flight (ToF)** — thời gian bay.
- Ví dụ: vật ở cách 1 mét → laser bay đi 1m rồi bay về 1m = 2m
  - Thời gian = 2m / (3×10⁸ m/s) = **6.67 nanosecond** (6.67 phần tỷ giây!)
- Để đo thời gian cực ngắn như vậy, chip sử dụng kỹ thuật **TCSPC** (Time-Correlated Single Photon Counting):
  - Phát nhiều xung laser liên tiếp.
  - Đo thời gian đến của từng photon.
  - Xây dựng **biểu đồ histogram** thống kê.
  - Đỉnh (peak) của histogram = thời gian bay thực sự.
  - Phương pháp này giúp lọc bỏ nhiễu từ ánh sáng môi trường.

#### Bước 6: Tính khoảng cách

```
          c × t
    d = ─────────
            2

    Trong đó:
    d = khoảng cách (m)
    c = tốc độ ánh sáng = 299,792,458 m/s
    t = thời gian bay (s)
    Chia 2 vì ánh sáng đi MỘT CHIỀU rồi DỘI LẠI (đi + về)
```

**Ví dụ tính**:
- Quả bóng ở cách cảm biến 200mm (20cm)
- Ánh sáng đi + về = 400mm = 0.4m
- Thời gian bay = 0.4 / 299,792,458 ≈ **1.33 ns** (nanosecond)
- VL53L0X đo được thời gian này và trả về kết quả: **200mm**

### 2.3 Ưu điểm so với cảm biến siêu âm

| Tiêu chí | VL53L0X (ToF Laser) | HC-SR04 (Siêu âm) |
|---|---|---|
| Tốc độ đo | ~30Hz (33ms) | ~20Hz (50ms+) |
| Tầm đo tối thiểu | 30mm | 20mm |
| Góc đo | 25° (hẹp, chính xác) | 30-45° (rộng, dễ nhiễu) |
| Ảnh hưởng nhiệt độ | Không | Có (tốc độ âm thanh thay đổi) |
| Kích thước | 4.4mm chip | 45mm module |
| Chống nhiễu | Tốt (histogram filtering) | Kém (dễ bị echo giả) |

---

## 3. Giao thức I2C - Giải thích từ A đến Z

### 3.1 I2C là gì?

**I2C** (đọc là "Ai-Tua-Xi" hoặc "I-squared-C") viết tắt của **Inter-Integrated Circuit** — nghĩa là "giao tiếp giữa các mạch tích hợp với nhau".

I2C được phát minh bởi hãng **Philips** (nay là NXP Semiconductors) vào **năm 1982** với mục đích: cho phép nhiều chip (IC) trên cùng một mạch in nói chuyện với nhau chỉ qua **2 sợi dây**.

**Ví dụ dễ hiểu**: Hãy tưởng tượng I2C như một **phòng họp** nơi:
- Có 1 **sếp (Master)** = ESP32-S3
- Có nhiều **nhân viên (Slave)** = VL53L0X, OLED, v.v.
- Chỉ có **2 kênh liên lạc** = SDA và SCL
- Mỗi nhân viên có **số ID riêng** = I2C Address
- Sếp gọi tên (địa chỉ), nhân viên đó mới được phát biểu

### 3.2 Hai dây dữ liệu: SDA và SCL

| Dây | Tên đầy đủ | Chức năng |
|---|---|---|
| **SDA** | Serial Data | Truyền dữ liệu 2 chiều (Master ↔ Slave) |
| **SCL** | Serial Clock | Xung nhịp đồng hồ (Master tạo, Slave đọc) |

- Cả 2 dây đều cần **điện trở kéo lên (pull-up resistor)** nối đến VCC (thường 4.7kΩ).
- Các module breakout board (như module VL53L0X bạn mua) thường **đã có sẵn pull-up** → không cần gắn thêm.
- Nhiều thiết bị **nối song song** trên cùng 2 dây này (miễn là khác địa chỉ).

### 3.3 Master và Slave

```
                    I2C Bus
                ┌─────────────────────────────────┐
                │     SDA ────────────────────     │
                │     SCL ────────────────────     │
                └─────────────────────────────────┘
                      │           │           │
               ┌──────┴───┐ ┌────┴─────┐ ┌───┴──────┐
               │ ESP32-S3 │ │ VL53L0X  │ │ SSD1306  │
               │ (Master) │ │ (Slave)  │ │ (Slave)  │
               │          │ │ Addr:0x29│ │ Addr:0x3C│
               └──────────┘ └──────────┘ └──────────┘
```

**Master (Chủ) — ESP32-S3**:
- Tạo xung clock SCL.
- Quyết định khi nào bắt đầu/kết thúc giao tiếp.
- Gọi Slave bằng địa chỉ.
- Gửi lệnh đọc/ghi.

**Slave (Tớ) — VL53L0X, SSD1306**:
- Lắng nghe bus, chỉ phản hồi khi được gọi đúng địa chỉ.
- Không tự ý phát dữ liệu.
- Mỗi Slave có địa chỉ **7-bit duy nhất** (tổng cộng 128 địa chỉ khả dụng).

### 3.4 Quá trình truyền dữ liệu I2C

```
Trình tự đọc dữ liệu từ VL53L0X:

1. Master gửi START condition (SDA xuống LOW khi SCL đang HIGH)
2. Master gửi 7-bit địa chỉ Slave (0x29) + bit Write (0)
3. Slave (VL53L0X) gửi ACK (xác nhận "Tôi có mặt!")
4. Master gửi địa chỉ thanh ghi muốn đọc
5. Slave gửi ACK
6. Master gửi REPEATED START
7. Master gửi 7-bit địa chỉ Slave (0x29) + bit Read (1)
8. Slave gửi ACK
9. Slave gửi dữ liệu (byte khoảng cách High)
10. Master gửi ACK
11. Slave gửi dữ liệu (byte khoảng cách Low)
12. Master gửi NACK (không cần thêm dữ liệu)
13. Master gửi STOP condition (SDA lên HIGH khi SCL đang HIGH)
```

### 3.5 Tốc độ I2C

| Chế độ | Tốc độ | Sử dụng |
|---|---|---|
| Standard Mode | 100 kHz | Mặc định, tương thích tốt |
| **Fast Mode** | **400 kHz** | **Dùng trong dự án này** |
| Fast Mode Plus | 1 MHz | Ít thiết bị hỗ trợ |
| High Speed | 3.4 MHz | Chuyên dụng |

Chúng ta dùng **400kHz Fast Mode** vì cả VL53L0X và SSD1306 đều hỗ trợ tốc độ này, giúp giao tiếp nhanh hơn và giảm thời gian chiếm bus.

---

## 4. ESP32-S3 đọc dữ liệu VL53L0X như thế nào?

### 4.1 Phần cứng: Kết nối dây

ESP32-S3 hỗ trợ **I2C trên bất kỳ GPIO nào** (không cố định như Arduino Uno). Trong dự án này:

| ESP32-S3 Pin | Chức năng | Nối đến |
|---|---|---|
| GPIO8 | SDA (Data) | SDA của VL53L0X VÀ SDA của OLED |
| GPIO9 | SCL (Clock) | SCL của VL53L0X VÀ SCL của OLED |
| 3V3 | Nguồn 3.3V | VIN/VCC của cả 2 module |
| GND | Mass | GND của cả 2 module |

> **Lưu ý**: Thay đổi GPIO8/GPIO9 thành chân khác bằng cách sửa `I2C_SDA_PIN` và `I2C_SCL_PIN` trong code.

### 4.2 Phần mềm: Wire Library

ESP32-S3 sử dụng thư viện **Wire** (có sẵn trong ESP32 Arduino Core) để giao tiếp I2C:

```cpp
// Khởi tạo I2C
Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Chọn chân SDA, SCL
Wire.setClock(400000);                   // Tốc độ 400kHz

// Đọc cảm biến (thư viện Adafruit làm hộ phần phức tạp)
VL53L0X_RangingMeasurementData_t measure;
vl53.rangingTest(&measure, false);
uint16_t distance = measure.RangeMilliMeter;   // Kết quả (mm)
uint8_t  status   = measure.RangeStatus;       // Trạng thái đo
```

### 4.3 Chuỗi sự kiện khi ESP32-S3 đọc khoảng cách

```
ESP32-S3                           VL53L0X
   │                                  │
   │──── "Bắt đầu đo!" ───────────▶  │
   │     (Ghi vào thanh ghi lệnh)     │
   │                                  │
   │     (VL53L0X tự động:)           │
   │     ┌─ Phát laser ──────────────▶│ Vật thể
   │     │  Thu photon ◀──────────────│
   │     │  Tính toán histogram       │
   │     │  Xác định khoảng cách      │
   │     └─ Lưu kết quả vào thanh ghi │
   │                                  │
   │──── "Gửi kết quả đi!" ────────▶ │
   │     (Đọc thanh ghi kết quả)      │
   │                                  │
   │◀──── Khoảng cách: 200mm ──────── │
   │◀──── Trạng thái: OK ──────────── │
   │◀──── Signal Rate: 2.5 Mcps ───── │
   │                                  │
```

---

## 5. Hệ thống lọc nhiễu 5 tầng (Filter Pipeline)

### 5.1 Tại sao cần lọc nhiễu?

VL53L0X tuy chính xác nhưng **KHÔNG HOÀN HẢO**. Dữ liệu thực tế sẽ có nhiễu:

| Loại nhiễu | Nguyên nhân | Biểu hiện |
|---|---|---|
| **Spike noise** | Đa phản xạ (multipath), laser phản xạ từ cạnh bóng | Đột ngột nhảy lên 800mm rồi quay lại 200mm |
| **Gaussian noise** | Nhiễu điện tử, ánh sáng môi trường | Dao động nhỏ ±5-15mm quanh giá trị thực |
| **Dropout** | Mất tín hiệu (bề mặt hấp thụ, ngoài tầm) | Trả về 0mm hoặc 8190mm |
| **Drift** | Nhiệt độ thay đổi, aging | Sai lệch chậm theo thời gian |

Nếu đưa dữ liệu thô vào PID controller → servo sẽ **run rẩy, giật**, bóng không ổn định.

### 5.2 Tổng quan Filter Pipeline

```
    Dữ liệu thô từ VL53L0X
           │
           ▼
   ┌───────────────────┐
   │  Tầng 1:          │   Loại bỏ giá trị lỗi (0mm, 8190mm,
   │  VALIDATION GATE  │   sensor error codes)
   └───────┬───────────┘
           ▼
   ┌───────────────────┐
   │  Tầng 2:          │   Phát hiện outlier bằng MAD
   │  HAMPEL FILTER    │   (Median Absolute Deviation)
   └───────┬───────────┘
           ▼
   ┌───────────────────┐
   │  Tầng 3:          │   Lọc trung vị cửa sổ 5 mẫu
   │  MEDIAN FILTER    │   (triệt tiêu impulse noise)
   └───────┬───────────┘
           ▼
   ┌───────────────────┐
   │  Tầng 4:          │   Ước lượng tối ưu vị trí + vận tốc
   │  ADAPTIVE KALMAN  │   (tự điều chỉnh theo mức nhiễu)
   │  2D FILTER        │
   └───────┬───────────┘
           ▼
   ┌───────────────────┐
   │  Tầng 5:          │   Giới hạn tốc độ thay đổi
   │  RATE LIMITER     │   (ràng buộc vật lý)
   └───────┬───────────┘
           ▼
    Dữ liệu sạch → OLED + Serial (LabVIEW)
```

### 5.3 Tầng 1: Validation Gate (Cổng kiểm tra)

**Mục đích**: Loại bỏ các giá trị mà ta **biết chắc là sai**.

**Quy tắc**:
- VL53L0X trả về `0mm` → LỖI (cảm biến không đo được).
- VL53L0X trả về `8190mm` → KHÔNG CÓ VẬT THỂ trong tầm đo.
- Khoảng cách < 20mm → Ngoài tầm đo tối thiểu (không tin cậy).
- Khoảng cách > 1200mm → Ngoài tầm đo tin cậy.
- `RangeStatus ≠ 0 và ≠ 4` → Cảm biến báo lỗi đo.

**Kết quả**: Tầng này chặn ~5-10% số đọc trong điều kiện bình thường.

### 5.4 Tầng 2: Hampel Filter (Lọc ngoại lai bằng MAD)

**Mục đích**: Phát hiện và thay thế các giá trị **ngoại lai (outlier)** — tức là những giá trị đột nhiên khác thường so với các giá trị gần đây.

**Tại sao không dùng "range checking" đơn giản?**
- Range checking (kiểm tra khoảng) chỉ loại giá trị ngoài min/max cố định.
- Ví dụ: nếu bóng ở 200mm mà đột nhiên nhảy lên 500mm (vẫn trong khoảng hợp lệ!), range checking sẽ **BỎ SÓT**.
- Hampel filter phát hiện được outlier **tùy theo ngữ cảnh** (context-dependent).

**Thuật toán**:
1. Duy trì một cửa sổ (buffer) 5 giá trị gần nhất.
2. Tính **median** (trung vị) của cửa sổ.
3. Tính **MAD** (Median Absolute Deviation):
   - MAD = median(|x₁ - median|, |x₂ - median|, ...) × 1.4826
   - Hệ số 1.4826 giúp MAD tương đương với độ lệch chuẩn (standard deviation) khi dữ liệu phân phối chuẩn.
4. Tính **score** = |giá trị mới - median| / MAD
5. Nếu score > 3.0 (ngưỡng 3-sigma) → **OUTLIER** → thay bằng median.
6. Nếu score ≤ 3.0 → **HỢP LỆ** → giữ nguyên.

**Ưu điểm**: Có **breakdown point = 50%** — nghĩa là ngay cả khi 49% dữ liệu bị nhiễu, Hampel vẫn hoạt động đúng.

### 5.5 Tầng 3: Median Filter (Lọc trung vị)

**Mục đích**: Làm mượt tín hiệu bằng cách lấy **giá trị trung vị** trong cửa sổ trượt 5 mẫu.

**Tại sao dùng Median thay vì Mean (trung bình)?**
- **Mean** (trung bình cộng): BỊ ẢNH HƯỞNG bởi giá trị cực đoan.
  - Ví dụ: [200, 201, 199, 800, 200] → mean = 320 (SAI!)
- **Median** (trung vị): KHÔNG BỊ ẢNH HƯỞNG bởi giá trị cực đoan.
  - Ví dụ: [200, 201, 199, 800, 200] → sắp xếp: [199,200,200,201,800] → median = 200 (ĐÚNG!)

**Tính chất**:
- Lọc phi tuyến (non-linear) → **bảo toàn cạnh (edge preserving)**.
- Khi bóng di chuyển nhanh, median filter không làm chậm phản hồi (quan trọng cho PID).
- Tối ưu cho nhiễu xung (impulse noise) — đặc trưng của cảm biến ToF.

### 5.6 Tầng 4: Adaptive 2D Kalman Filter (Bộ lọc Kalman thích nghi 2 trạng thái)

**Đây là bộ lọc mạnh nhất và phức tạp nhất trong pipeline.**

**Kalman Filter là gì?**

Bộ lọc Kalman (phát minh bởi Rudolf E. Kálmán, 1960) là thuật toán **ước lượng tối ưu (optimal estimation)** — nghĩa là nó cho ra kết quả **tốt nhất có thể** dựa trên:
- Mô hình vật lý (bóng chuyển động theo quy luật nào)
- Dữ liệu đo (VL53L0X đo được bao nhiêu)
- Mức độ tin cậy của mỗi nguồn

**Tại sao "2D" (2 trạng thái)?**

Thay vì chỉ ước lượng **vị trí**, Kalman 2D ước lượng đồng thời:
1. **Vị trí** (position) — bóng đang ở đâu (mm)
2. **Vận tốc** (velocity) — bóng đang di chuyển nhanh bao nhiêu (mm/s)

Lợi ích:
- **Dự đoán tốt hơn**: Biết vận tốc → dự đoán vị trí tiếp theo chính xác hơn.
- **Vận tốc miễn phí**: Không cần tính đạo hàm nhiễu → dùng luôn cho PID (thành phần D).
- **Chống dropout**: Nếu mất 1-2 mẫu đo, Kalman vẫn dự đoán được vị trí dựa trên vận tốc.

**Mô hình toán học**:

```
Trạng thái:  x = [vị_trí, vận_tốc]ᵀ

Phương trình chuyển trạng thái:
    vị_trí_mới  = vị_trí_cũ + vận_tốc × dt
    vận_tốc_mới = vận_tốc_cũ
    
Ma trận chuyển:  F = | 1  dt |
                     | 0   1 |

Ma trận đo:  H = [1  0]   (ta chỉ đo được vị trí, không đo trực tiếp vận tốc)
```

**Chu kỳ Predict-Update**:

```
Predict (Dự đoán):
    1. Dự đoán trạng thái: x̂ = F × x_old
    2. Dự đoán uncertainty: P̂ = F × P_old × Fᵀ + Q

Update (Cập nhật khi có phép đo mới):
    3. Tính innovation: y = z_measured - H × x̂
    4. Tính Kalman gain: K = P̂ × Hᵀ / (H × P̂ × Hᵀ + R)
    5. Cập nhật trạng thái: x = x̂ + K × y
    6. Cập nhật uncertainty: P = (I - K×H) × P̂
```

**Tính năng "Adaptive" (Thích nghi)**:

Kalman tiêu chuẩn cần đặt R (nhiễu đo) cố định. Nhưng trong thực tế, mức nhiễu thay đổi liên tục. Bộ lọc trong code **tự động điều chỉnh R**:

- Theo dõi cửa sổ 10 giá trị innovation (sai số dự đoán).
- Nếu innovation lớn liên tục → sensor đang nhiều nhiễu → **tăng R** → tin phần dự đoán nhiều hơn.
- Nếu innovation nhỏ → sensor ổn định → **giảm R** → phản hồi nhanh hơn theo phép đo.

### 5.7 Tầng 5: Rate Limiter (Giới hạn tốc độ thay đổi)

**Mục đích**: Đặt ràng buộc vật lý cuối cùng — quả bóng **không thể dịch chuyển tức thời**.

**Trong dự án Ball and Beam**:
- Thanh beam dài khoảng 500mm.
- Bóng lăn với tốc độ tối đa thực tế khoảng 500mm/s.
- Ở tần số đo 30Hz → mỗi chu kỳ bóng di chuyển tối đa ~17mm.
- Ta đặt giới hạn **50mm/cycle** (có biên an toàn).

Nếu sau 4 tầng lọc trước mà vẫn còn 1 giá trị nhảy đột ngột (rất hiếm!) → Rate Limiter sẽ **cắt bớt** sự thay đổi.

---

## 6. Ứng dụng thực tế và công nghiệp

### 6.1 Ứng dụng của cảm biến VL53L0X trong đời thường

| Ứng dụng | Mô tả |
|---|---|
| **Robot hút bụi** | Phát hiện vật cản, vách tường, cầu thang (tránh rơi) |
| **Drone** | Giữ độ cao ổn định khi bay thấp (altitude hold) |
| **Điện thoại thông minh** | Tắt màn hình khi áp sát tai (proximity sensor) - Samsung, Xiaomi sử dụng cảm biến ToF tương tự |
| **Camera lấy nét** | Hỗ trợ autofocus nhanh (laser AF) |
| **Vòi nước tự động** | Phát hiện tay đưa vào rửa |
| **Nhận dạng cử chỉ** | Điều khiển thiết bị bằng tay (swipe in/out) |

### 6.2 Ứng dụng trong công nghiệp

| Ứng dụng | Mô tả |
|---|---|
| **Dây chuyền sản xuất** | Phát hiện sản phẩm trên băng chuyền (đếm, phân loại) |
| **AGV / AMR** | Tránh vật cản cho xe tự hành trong nhà máy |
| **Đo mức chất lỏng** | Đo khoảng cách từ cảm biến đến mặt chất lỏng trong bồn |
| **Đóng gói tự động** | Kiểm tra kích thước sản phẩm, phát hiện vật thể trên pallet |
| **An ninh** | Phát hiện người đi qua cửa, đếm người trong phòng |
| **Pick & Place** | Robot gắp nhả chính xác, kiểm tra có vật hay không |
| **Kiểm tra chất lượng** | Đo độ dày, kiểm tra lắp ráp đúng vị trí |

### 6.3 Ứng dụng trong dự án Ball and Beam

Hệ thống **Ball and Beam** là một bài toán điều khiển kinh điển (classic control problem) được giảng dạy rộng rãi trong các trường đại học kỹ thuật trên toàn thế giới.

**Mô tả hệ thống**:
- Một **thanh beam** (thanh ngang) được gắn vào **servo motor** ở giữa.
- Một **quả bóng** đặt trên thanh beam, có thể lăn tự do.
- **Mục tiêu**: Điều khiển góc nghiêng của beam (qua servo) để giữ bóng ở vị trí mong muốn.

**Vai trò của VL53L0X**:
- Đặt ở một đầu thanh beam, chiếu laser dọc theo beam.
- Đo khoảng cách từ cảm biến đến quả bóng → biết **vị trí bóng**.
- Cung cấp **phản hồi (feedback)** cho bộ điều khiển PID.
- Bộ điều khiển PID so sánh vị trí thực tế với vị trí mong muốn → tính góc servo → điều khiển beam.

**Ứng dụng thực tế của bài toán Ball and Beam**:

Bài toán này mô phỏng nhiều hệ thống thực tế:
| Hệ thống thực | Tương đồng |
|---|---|
| Tên lửa giữ thăng bằng | Điều khiển góc nghiêng để giữ quỹ đạo |
| Self-balancing robot | Giữ thăng bằng trên 2 bánh |
| Ổn định tàu thuyền | Chống lắc trên sóng biển |
| Hệ thống lái xe tự động | Giữ xe đúng làn đường |
| Điều khiển nhiệt độ lò công nghiệp | Giữ nhiệt độ ổn định quanh setpoint |

---

## 7. Sơ đồ kết nối phần cứng

```
    ┌─────────────────────────────────────────────────────────┐
    │                    ESP32-S3 DevKit                      │
    │                                                         │
    │    3V3 ─────────┬───────────────────┬──── 3V3           │
    │    GND ─────────┼───────────────────┼──── GND           │
    │    GPIO8 (SDA) ─┼─── SDA ──────────┼──── SDA           │
    │    GPIO9 (SCL) ─┼─── SCL ──────────┼──── SCL           │
    │                 │                   │                    │
    │           ┌─────┴─────┐       ┌────┴──────┐            │
    │           │  VL53L0X  │       │  SSD1306  │            │
    │           │  (0x29)   │       │  OLED     │            │
    │           │           │       │  (0x3C)   │            │
    │           └───────────┘       └───────────┘            │
    └─────────────────────────────────────────────────────────┘

    Chú ý:
    - VL53L0X và SSD1306 nối SONG SONG trên cùng bus I2C
    - Khác địa chỉ (0x29 vs 0x3C) → KHÔNG xung đột
    - Pull-up resistor: có sẵn trên module breakout
    - Nguồn: dùng 3.3V từ ESP32-S3 (KHÔNG dùng 5V cho VL53L0X)
```

---

## 8. Hướng dẫn sử dụng code test

### 8.1 Cài đặt thư viện (Arduino IDE)

Vào **Sketch → Include Library → Manage Libraries**, tìm và cài:

1. `Adafruit VL53L0X` (by Adafruit)
2. `Adafruit SSD1306` (by Adafruit)
3. `Adafruit GFX Library` (by Adafruit) — cài tự động khi cài SSD1306

### 8.2 Cấu hình board

- **Board**: ESP32S3 Dev Module
- **USB CDC On Boot**: Enabled (để Serial Monitor hoạt động)
- **Upload Speed**: 921600
- **Flash Size**: 4MB hoặc theo board
- **Partition Scheme**: Default

### 8.3 Chỉnh sửa chân I2C (nếu cần)

Mở file `VL53L0X.ino`, sửa dòng:
```cpp
#define I2C_SDA_PIN   8    // Đổi thành GPIO bạn dùng
#define I2C_SCL_PIN   9    // Đổi thành GPIO bạn dùng
```

### 8.4 Upload và chạy

1. Kết nối ESP32-S3 qua USB.
2. Chọn đúng board và COM port.
3. Upload code.
4. Mở Serial Monitor (115200 baud).
5. Quan sát:
   - Startup: I2C scan, sensor init
   - `$DATA,...` — dữ liệu đo mỗi 50ms
   - `$DIAG,...` — thông tin chẩn đoán mỗi 2s

---

## 9. Format dữ liệu Serial (cho LabVIEW)

### 9.1 Các loại message

| Prefix | Khi nào | Nội dung |
|---|---|---|
| `$HEADER` | 1 lần khi khởi động | Tên các cột dữ liệu |
| `$DATA` | Mỗi 50ms (20Hz) | Dữ liệu đo lường |
| `$DIAG` | Mỗi 2s (0.5Hz) | Thông tin chẩn đoán |

### 9.2 Format $DATA

```
$DATA,raw_mm,filtered_mm,velocity_mm_s,kalman_gain,valid_pct

Ví dụ:
$DATA,203.0,201.5,12.3,0.4521,98.5
```

| Field | Ý nghĩa | Đơn vị |
|---|---|---|
| raw_mm | Khoảng cách thô từ VL53L0X | mm |
| filtered_mm | Khoảng cách sau 5 tầng lọc | mm |
| velocity_mm_s | Vận tốc ước lượng (Kalman) | mm/s |
| kalman_gain | Kalman gain hiện tại (0→1) | — |
| valid_pct | Tỷ lệ đọc hợp lệ | % |

### 9.3 Hướng dẫn parse trong LabVIEW

1. Dùng **VISA Read** để đọc từ COM port.
2. Kiểm tra prefix bằng **String Subset** hoặc **Match Pattern**.
3. Nếu bắt đầu bằng `$DATA,` → tách bằng **Spreadsheet String To Array** (delimiter = `,`).
4. Convert sang float array.
5. Hiển thị lên Waveform Chart / Gauge / Numeric Indicator.

---

## 10. Tài liệu tham khảo

1. **VL53L0X Datasheet** — STMicroelectronics: [st.com/resource/en/datasheet/vl53l0x.pdf](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
2. **VL53L0X API User Manual** — UM2039: [st.com](https://www.st.com/resource/en/user_manual/um2039.pdf)
3. **FlightSense™ Technology** — AN4846: Application note giải thích chi tiết nguyên lý ToF.
4. **Adafruit VL53L0X Library**: [github.com/adafruit/Adafruit_VL53L0X](https://github.com/adafruit/Adafruit_VL53L0X)
5. **Kalman Filter — Greg Welch & Gary Bishop**: "An Introduction to the Kalman Filter" — University of North Carolina.
6. **Hampel Filter**: *Hampel, F. R. (1974). "The Influence Curve and Its Role in Robust Estimation."*
7. **ESP32-S3 Technical Reference Manual**: [espressif.com](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
8. **I2C Specification** — NXP UM10204: [nxp.com](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

---

> **Tài liệu này là một phần của dự án Ball and Beam.**
> **Hardware**: ESP32-S3 + VL53L0X + SSD1306 OLED + Servo MG996R
