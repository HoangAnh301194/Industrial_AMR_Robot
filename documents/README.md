
## 1. Tổng quan đề bài

### 1.1. Bài toán thiết kế

Nhà máy X yêu cầu thiết kế 01 robot di động tự hành AMR (Autonomous Mobile Robot) thực hiện nhiệm vụ:

- Nhận khay linh kiện tại kho A.
- Vận chuyển và giao khay lần lượt đến các trạm B, C, D.
- Tự động xác định vị trí và điều hướng.
- Phát hiện, tránh người và vật cản trong quá trình di chuyển.
- Hoàn thành nhiệm vụ và tự động quay về trạm sạc.

### 1.2. Mục tiêu hệ thống

Xây dựng robot AMR có khả năng vận chuyển hàng hóa tự động trong môi trường nhà máy, đáp ứng các yêu cầu về tải trọng, tốc độ, độ chính xác, an toàn và thời gian hoạt động.

## 2. Các yêu cầu kỹ thuật

| STT | Thông số | Yêu cầu |
|---|---|---|
| 1 | Tải trọng (Payload) | 20 kg |
| 2 | Chiều rộng hành lang | Tối thiểu 1,2 m |
| 3 | Tốc độ di chuyển | Tối thiểu 1,2 m/s |
| 3 | Sai số dừng (Docking Accuracy) | Không quá ±30 mm |
| 5 | Thời gian hoạt động (Runtime) | Tối thiểu 3 giờ |
| 6 | Tránh vật cản | Phát hiện và tránh người/vật cản |
| 7 | Điều hướng | Tự động di chuyển giữa A, B, C, D |
| 8 | Sạc | Tự động quay về trạm sạc |
### 2.1. Yêu cầu chức năng (Functional Requirements)

- Nhận lệnh vận chuyển.
- Xác định vị trí hiện tại.
- Lập kế hoạch đường đi.
- Điều khiển chuyển động.
- Phát hiện và tránh vật cản.
- Giao nhận khay tại các trạm.
- Quản lý trạng thái pin.
- Tự động quay về trạm sạc.
- Báo cáo trạng thái và kết quả nhiệm vụ.

### 2.2. Yêu cầu phi chức năng (Non-functional Requirements)

- Đảm bảo an toàn khi hoạt động cùng con người.
- Đảm bảo độ chính xác khi tiếp cận trạm.
- Hoạt động ổn định liên tục tối thiểu 3 giờ.
- Đảm bảo khả năng bảo trì và nâng cấp.
- Phù hợp với không gian hành lang rộng tối thiểu 1,2 m.
## 3. Mô hình chữ V (V-Model)

### 3.1. Khái niệm

V-Model là mô hình phát triển hệ thống liên kết các giai đoạn phân tích, thiết kế với các giai đoạn kiểm thử tương ứng.


### 3.2. Ánh xạ V-Model vào dự án AMR

| Nhánh thiết kế | Nhánh kiểm thử tương ứng |
|---|---|
| Nhu cầu người dùng (User Needs) | Kiểm thử xác nhận nhiệm vụ (Mission Validation) |
| Yêu cầu hệ thống (System Requirements) | Kiểm chứng yêu cầu hệ thống (System Verification) |
| Thiết kế kiến trúc (Architecture Design) | Kiểm thử tích hợp (Integration Testing) |
| Thiết kế chi tiết (Detailed Design) | Kiểm thử đơn vị (Unit Testing) |
| Hiện thực hóa phần cứng và phần mềm | Chế tạo và triển khai module |

### 3.3. Các cấp độ kiểm thử

- **Unit Test:** Kiểm tra riêng từng module như cảm biến, động cơ, encoder, bộ điều khiển.
- **Integration Test:** Kiểm tra sự phối hợp giữa các module cơ khí, điện và phần mềm.
- **System Test:** Kiểm tra toàn bộ robot theo các yêu cầu kỹ thuật.
- **Acceptance / Validation Test:** Kiểm tra robot có thực hiện đúng nhiệm vụ thực tế tại nhà máy hay không.

### 3.4. Các cổng đánh giá thiết kế (Design Reviews)

| Cổng | Nội dung |
|---|---|
| SRR | System Requirements Review – Đánh giá và chốt yêu cầu hệ thống |
| PDR | Preliminary Design Review – Đánh giá phương án và kiến trúc sơ bộ |
| CDR | Critical Design Review – Đánh giá thiết kế chi tiết trước chế tạo |
| TRR | Test Readiness Review – Đánh giá mức độ sẵn sàng kiểm thử |
| FDR | Final Design Review – Đánh giá thiết kế cuối cùng dựa trên kết quả thử nghiệm |
## 4. Kiến trúc phân tầng hệ thống (System Architecture)

Hệ thống được tổ chức theo các tầng chức năng, từ tầng quản lý nhiệm vụ đến tầng phần cứng thực thi.

### 4.1. Tầng 1 – Quản lý nhiệm vụ (Mission Layer)

- Nhận nhiệm vụ vận chuyển.
- Quản lý thứ tự giao hàng A → B → C → D.
- Quản lý trạng thái nhiệm vụ.
- Quyết định quay về trạm sạc.
- Gửi thông báo hoàn thành hoặc lỗi.

### 4.2. Tầng 2 – Điều hướng và nhận thức (Navigation & Perception Layer)

- Định vị robot.
- Xây dựng và sử dụng bản đồ.
- Lập kế hoạch đường đi.
- Phát hiện và tránh vật cản.
- Xác định vị trí trạm giao nhận.
- Điều hướng và docking.

### 4.3. Tầng 3 – Điều khiển (Control Layer)

- Điều khiển tốc độ và hướng di chuyển.
- Điều khiển động cơ.
- Nhận dữ liệu encoder.
- Điều khiển chuyển động theo lệnh từ tầng điều hướng.
- Giám sát trạng thái và thực hiện dừng khẩn cấp.

### 4.4. Tầng 4 – Phần cứng (Hardware Layer)

- Khung cơ khí và bánh xe.
- Động cơ và driver.
- LiDAR, camera, IMU.
- Encoder.
- SBC/Jetson và MCU.
- Pin, BMS và hệ thống sạc.
- Mạch an toàn.

