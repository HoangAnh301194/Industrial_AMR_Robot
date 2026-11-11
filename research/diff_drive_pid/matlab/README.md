# MATLAB — Động học, động lực học, điều khiển PI/PID robot vi sai

Thư mục này bổ sung **mã MATLAB (.m) chạy mô phỏng số** cho dự án Factory Delivery AMR. Mọi thông số phần cứng và hệ số PI trong ví dụ đều là **giả định kỹ thuật** để giải thích phương pháp; không phải tham số đã hiệu chỉnh trên robot thật.

## Yêu cầu

- MATLAB hỗ trợ syntax function và đồ thị cơ bản; `run_all.m` chỉ cần MATLAB cơ bản, **không cần Simulink hoặc toolbox điều khiển**.
- `plot_from_python_csv.m` có dùng `readmatrix`, phù hợp từ MATLAB R2019a trở lên.
- Không cung cấp `.slx` (đây là mô hình số `.m`, chưa phải dự án Simulink).

## Chạy toàn bộ mô phỏng MATLAB

Chuyển thư mục làm việc (*Current Folder*) trong MATLAB đến thư mục chứa mã nguồn này (`research/diff_drive_pid/matlab`) hoặc chạy:

```matlab
% Nếu mở MATLAB từ thư mục gốc repository:
cd research/diff_drive_pid/matlab
run_all
```

`run_all` tạo:

- **14 biểu đồ PNG** có tên `matlab_*.png` trong `../docs/assets/`;
- **7 bảng dữ liệu CSV** có tên `matlab_*.csv` trong `../examples/data/`;
- kiểm tra tự động động học thuận–nghịch, giới hạn mô-men giả định và việc hoàn thành waypoint D.

## Danh sách tệp MATLAB

| Tệp | Mục đích |
|---|---|
| `run_all.m` | Chạy toàn bộ kịch bản, xuất dữ liệu và ảnh |
| `amr_params.m` | Các giả định khối lượng, kích thước, bánh, quán tính, PI |
| `diffdrive_inverse.m` / `diffdrive_forward.m` | Động học nghịch và thuận |
| `wheel_pi_step.m` | Động lực học hai bánh và PI có chống bão hòa |
| `wheel_pid_step.m` | Động lực học hai bánh và PID với đạo hàm lọc |
| `simulate_pi_pid_comparison.m` | So sánh PI/PID khi tốc độ đặt thay đổi |
| `simulate_straight_pi.m` | Đi thẳng, tăng/giảm tốc, bám 1,2 m/s |
| `simulate_turning.m` | Chạy cong, đổi hướng, quay tại chỗ |
| `simulate_p_vs_pi.m` | So sánh P/PI khi nhiễu tải một bánh |
| `simulate_waypoints.m` | Điều khiển tuyến A→B→C→D bằng waypoint minh họa |
| `amr_plot_all.m` | Vẽ 12 đồ thị và lưu PNG |
| `amr_write_csv.m` / `amr_save_png.m` | Xuất dữ liệu/hình |
| `validate_amr.m` | Kiểm tra nhanh mô hình số |
| `plot_from_python_csv.m` | Tùy chọn: đọc CSV Python cũ và vẽ lại hình trong MATLAB |

## Thông số ví dụ

- Tải tính toán: robot 55 kg (giả định) + hàng 20 kg (yêu cầu khách hàng), tổng 75 kg.
- Bán kính bánh: 0,08 m, khoảng cách tâm hai bánh: 0,45 m (giả định).
- Quán tính yaw: xấp xỉ thân hình hộp, bánh quay có quán tính `0.012 kg.m^2` (giả định).
- PI/PID: `Kp = 1.1`, `Ki = 3.0`, `Kd = 0.02`, lọc đạo hàm `0.04 s`, giới hạn mô-men `8 N.m/bánh` — **giả định minh họa, chưa tuning**.
- Kết quả PID có thể xấu hơn PI nếu hệ số D chưa phù hợp. Không được tự kết luận PID tốt hơn từ ví dụ này.
- Tọa độ waypoint chỉ minh họa: A(0,0), B(3,0), C(3,2), D(6,2) mét.
- Ngưỡng đến waypoint 0,12 m **không tương đương** với docking ±30 mm.

## Lưu ý kỹ thuật

Mô hình dựa trên năng lượng động học:

\[
T=\frac12m v^2+\frac12 I_z\dot\theta^2+\frac12J_w(\dot\phi_R^2+\dot\phi_L^2)
\]

Lực cản lăn xấp xỉ được phân bố cho hai bánh, cản nhớt được giả định tuyến tính. Đây là mô hình giản lược không xét động lực học hệ treo, trượt bánh, sai số cảm biến, va chạm, tải phân bố lệch hay giới hạn an toàn. Không suy luận đạt tiêu chuẩn an toàn hoặc yêu cầu dừng ±30 mm từ kết quả này.

**Trạng thái kiểm tra:** Mã MATLAB đã được rà soát tĩnh và đối chiếu với phiên bản Python hiện có, nhưng **chưa chạy trong MATLAB** vì môi trường biên soạn không cài MATLAB. Nhóm cần chạy `run_all` trên máy cá nhân và kiểm tra đồ thị trước khi đưa kết quả MATLAB vào báo cáo.
