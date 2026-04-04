# 📡 Nghiên Cứu Chuyên Sâu: Giao Thức MQTT (Message Queuing Telemetry Transport)
> **Ứng dụng**: Giám sát và điều khiển từ xa cho dự án Ball and Beam
> **Ngày cập nhật**: 25/03/2026

---

## 1. MQTT là gì?
**MQTT** là viết tắt của *Message Queuing Telemetry Transport*.
Mặc dù bạn dùng từ "ngôn ngữ khác" trong câu hỏi, nhưng thực chất MQTT không phải là "ngôn ngữ lập trình" (như C++, Python, hay JavaScript), mà nó là một **Giao thức truyền thông (Communication Protocol)** hoạt động trên nền mạng TCP/IP, giống như HTTP (dùng để duyệt web) hay WebSockets.

MQTT được thiết kế năm 1999 bởi IBM dành riêng cho các hệ thống:
- Mạng có độ trễ cao, chập chờn (như 2G/3G hoặc mạng vệ tinh thuở sơ khai).
- Thiết bị có cấu hình cực thấp, bộ nhớ ít (như Cảm biến, Vi điều khiển ESP32, STM32).
- Tiêu thụ điện năng cực ít (phù hợp cho thiết bị chạy pin).

Ngày nay, MQTT đã trở thành **Tiêu chuẩn công nghiệp số 1 (De-facto standard)** cho hệ sinh thái **Internet of Things (IoT)**.

---

## 2. Nguyên lý hoạt động (Khác biệt mấu chốt)

### 2.1. Mô hình Publish/Subscribe (Xuất bản / Nơi nhận)
Hầu hết các giao thức truyền thống như HTTP hoạt động theo mô hình **Client-Server (Client hỏi, Server trả lời)**.
- Ví dụ Web: Bạn gõ `google.com` (Hỏi) ──▶ Server Google trả về trang web (Trả lời). Nếu bạn không hỏi, Server không tự nhiên gửi cho bạn.

Ngược lại, MQTT hoạt động theo mô hình **Publish/Subscribe (Pub/Sub) có trung gian**:
Trong MQTT có 3 thành phần chính:
1. **Publisher (Bên gửi):** Thiết bị phát dữ liệu (VD: ESP32 gửi khoảng cách bóng của dự án Ball and Beam).
2. **Subscriber (Bên nhận):** Thiết bị chờ nhận dữ liệu (VD: App điện thoại hoặc Web Dashboard giám sát).
3. **Broker (Máy chủ trung gian):** Điểm nút trung tâm nhận mọi tin nhắn từ Publisher và tự động chuyển đến đúng các Subscriber đang "đăng ký" nhận tin nhắn đó.

*Đặc điểm hay nhất*: Publisher và Subscriber KHÔNG hề biết mặt nhau, không kết nối trực tiếp với nhau. Cả hai chỉ kết nối với cái **Broker**. ESP32 cứ việc gửi dữ liệu lên Broker, còn ai lấy dữ liệu đó xem thì ESP32 không cần quan tâm.

### 2.2. Khái niệm "Topic" (Chủ đề)
Làm sao Broker biết tin nhắn nào phải gửi cho ai? Thông qua "Topic".
Topic hoạt động như một địa chỉ email hoặc một đường dẫn thư mục.

Ví dụ cho mô hình Ball and Beam:
- ESP32 đẩy vị trí bóng lên bằng cách nói với Broker: *"Ê Broker, tôi **Publish** (Gửi) số `250` vào topic `ballbeam/sensor/distance` nhé!"*
- Giả sử bạn có 1 cái Laptop và 1 cái Điện thoại đang mở App giám sát. Cả 2 cùng nhắc Broker: *"Ê Broker, tôi **Subscribe** (Đăng ký) topic `ballbeam/sensor/distance` nhé, có tin mới thì báo tui!"*
- Ngay lập tức, Broker gửi số `250` đến cả Laptop và Điện thoại cực nhanh (chỉ mất ~50ms).

### 2.3. Chất lượng Dịch vụ (QoS - Quality of Service)
MQTT đảm bảo không mất phần tin nhắn bằng 3 mức độ (QoS):
- **QoS 0 (At most once)**: Gửi 1 lần rồi thôi, mất ráng chịu (Nhanh nhất, dùng để gửi cảm biến liên tục).
- **QoS 1 (At least once)**: Gửi đến khi nào nhận được phản hồi ACK, có thể bị trùng tin nhắn.
- **QoS 2 (Exactly once)**: Đảm bảo đến đúng 1 lần, không trùng, không sót (Chậm nhất, dùng cho điều lệnh quan trọng như nhấn nút Dừng khẩn cấp).

---

## 3. So sánh với các giao thức khác (HTTP, WebSockets, Serial)

| Tiêu chí | MQTT (Đề xuất dùng) | HTTP (Web thông thường) | WebSockets (Tốt cho Web nội bộ) | Serial (COM) (LabVIEW cũ) |
|---|---|---|---|---|
| **Mô hình** | Publish / Subscribe | Request / Response | Bi-directional (Hai chiều) | Dây cáp trực tiếp |
| **Kích thước gói tin** | Cực nhỏ (chỉ mất **2 byte** header) | Lớn (Vài trăm byte cho HTTP Header) | Nhỏ | Nhỏ |
| **Tiêu thụ pin, RAM** | Rất thấp (hoàn hảo cho ESP32) | Cao (nặng CPU) | Trung bình | Thấp |
| **Tốc độ (Latency)** | Nhanh (**ms**), vì TCP giữ kết nối liên tục | Chậm (Phải mở/đóng kết nối mỗi lần) | Nhanh (Real-time) | Cực nhanh (ns) |
| **Phạm vi** | Toàn cầu (Qua Internet) | Toàn cầu (Qua Internet) | Thường dùng mạng nội bộ (LAN) | Dây cáp tối đa 2 mét |
| **1 gửi / Nhiều nhận** | Có. Tự động chia (1 Publish đến 100 Subscribe) | Không. Phải gửi 100 cái Request | Có, nhưng phức tạp ở phía nội bộ server | Không  |

**Kết luận**: 
- MQTT vượt trội hơn hẳn HTTP khi cần hiển thị biểu đồ theo thời gian thực (vì ESP32 đẩy liên tục ~20 lần/giây, HTTP sẽ làm tràn RAM của ESP32 nếu liên tục bị spam Request). 
- MQTT tốt hơn Serial vì bạn có thể mang cái ESP32 xuống một tầng nhà, và ngồi giám sát từ tầng khác qua WiFi mạng lưới đồ thị!

---

## 4. Cách triển khai thực tế MQTT khác WebServer (Web nội bộ) như thế nào?

Có 2 hướng đi thông dụng cho IoT ESP32, tôi phân tích cách CÀI ĐẶT và khác biệt:

### Phương án A: Dùng WebServer + WebSockets (Trực tiếp nhúng trên ESP32)
- **Kiến trúc**: Chính cái ESP32 biến thành 1 "Máy chủ trang Web" mini.
- **Cách vào xem**: Bạn mở điện thoại (phải kết nối **chung cục bộ WiFi** với ESP32), gõ IP của ESP32 (Ví dụ: `192.168.1.104`) vào trình duyệt web.
- **Điểm mạnh**: Không cần Internet, không cần Server thứ 3. Hệ thống "Tự cung tự cấp" hoàn toàn. Trễ bằng 0.
- **Điểm Yếu**: Ra khỏi nhà là tịt mù tắt ngấm. Màn hình điện thoại phải tự xử lý vẽ biểu đồ (Cần code Frontend HTML/CSS/JS nhúng vào bộ nhớ ESP32 khá cực).

### Phương án B: Dùng MQTT + (HiveMQ / Node-RED / Grafana)
- **Kiến trúc**: 
  - Đăng ký một hãng thứ 3 (Ví dụ **HiveMQ Cloud**, broker miễn phí 100 thiết bị kết nối dài hạn trọn đời).
  - ESP32 nối WiFi Internet, âm thầm đẩy dữ liệu `ballbeam/distance` lên HiveMQ Cloud.
  - Bạn mở phần mềm trên máy tính (như *Node-RED, MQTTX, MQTT Dash*), không cần chung WiFi mạng, kết nối với HiveMQ Cloud bằng thông tin Account. Bạn sẽ ngay lập tức được vẽ một Dashboard đẹp mê ly đồ thị.
- **Điểm Mạnh**: Quản lý Dashboard siêu đỉnh cao, có thể code Node-RED kéo thả rất đẹp. Cho phép lưu lại Log vào Database cho làm báo cáo. Theo dõi Ball & Beam từ quán Coffee mút chỉ bằng điện thoại 4G.
- **Điểm Yếu**: Phải phụ thuộc nhà mạng và Internet. Có độ trễ nhất định do đi qua server thế giới (100-300ms do Server HiveMQ Cloud đặt ở nước ngoài, AWS). Trễ này không ảnh hưởng chuyện hiển thị vẽ đồ thị biểu đồ giám sát, nhưng KHÔNG NÊN làm bộ điều khiển vòng kín qua nó.

---

## 5. Các bước triển khai MQTT cho Ball and Beam (Cầm tay chỉ việc)

Nếu bạn chốt **Triển khai phương án MQTT**:

### Bước 1: Đầu Não Broker (Trung tâm chuyển phát tin học)
- Đăng nhập trang `https://console.hivemq.cloud/` và tạo 1 tài khoản Free. 
- Lấy thông tin **Cluster URL** (Vd: `xyz123.s1.eu.hivemq.cloud`), **Port** (8883 - TLS Secure), **Username** và **Password**.

### Bước 2: ESP32 Code (Vi mạch nhúng)
- Trỏ thư viện **`PubSubClient`** của Nick O'Leary trên PlatformIO / Arduino IDE.
- Đổi code vòng `loop()`:
```cpp
// Định nghĩa Topic
#define TOPIC_DISTANCE "ballbeam/sensor/distance"
#define TOPIC_ERROR    "ballbeam/pid/error"
#define TOPIC_OUTPUT   "ballbeam/pid/output"

// Đặt ở cuối 1 vòng lặp đọc sensor/PID
if (millis() - lastPublish > 100) { // Đẩy 10 lần/giây
    mqttClient.publish(TOPIC_DISTANCE, String(distance_mm).c_str());
    mqttClient.publish(TOPIC_OUTPUT, String(pid_output).c_str());
}
```

### Bước 3: Xem Data Dashboard (Biểu đồ vĩ mô)
Sử dụng công cụ không cần code (low-code GUI):
- **Trên Máy tính**: Cài **MQTTX** (Phần mềm Test) hoặc đỉnh cao là Cài Node.js Server + **Node-RED** (kéo thả bảng Dashboard UI biểu đồ chuyên nghiệp của Industry 4.0).
- **Trên Điện thoại**: Tải các ứng dụng như "IoT MQTT Panel" hoặc "MQTT Dash", kéo thả nút bấm (Nút Enable/Disable), biểu đồ kim vạch (Khoảng cách), và biểu đồ đường (Error tín hiệu) trên màn hình cảm ứng để theo dõi. 

---

## 6. Các Nguồn Tài Liệu Nghiên Cứu Chi Tiết

Dưới đây là link bách khoa toàn thư từ dễ lên cực chuyên sâu:

1. **Hiểu bản chất MQTT nhanh bằng Video Animation:**
   - *Video Cơ bản của IBM:* Keyword Youtube (What is MQTT?) - Cơ bản.
   - *Video về Topic vs QoS:* HiveMQ Youtube Channel.

2. **Bài Đọc Tổng thể và Cực dễ hiểu (Tiếng Việt/Anh, Recommended 💯):**
   - **HiveMQ MQTT Essentials (CĂN BẢN VÀ CỰC KỲ DỄ HIỂU)**: [MQTT Essentials (Tất cả 10 phần)](https://www.hivemq.com/mqtt-essentials/) (Không thể bỏ qua bài viết này, nó là giáo án chuẩn nhất thế giới hiện nay cho việc học MQTT).

3. **Cài Đặt Node-RED để vẽ UI Dashboard (Giải pháp tốt thay thế LabVIEW):**
   - Tài liệu học Node-RED (RandomNerdTutorials): [Node-RED Dashboard cho MQTT / ESP32](https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/) (Đọc bài này họ hướng dẫn từ A-Z setup MQTT cho ESP32 và màn hình hiển thị Dashboard Cầu Kiều ra sao).

---
*Báo cáo được biên soạn để thay thế hoàn toàn mạng giám sát bằng cáp Serial / LabVIEW trên dự án Ball and Beam hiện tại, mở ra hệ thống giám sát không dây Wireless và Dashboard mượt mà chuyên nghiệp hơn.*
