function p = amr_params()
%AMR_PARAMS Giả định thiết kế dùng trong mô phỏng SỐ robot vi sai.
% Không phải thông số chốt từ URDF/động cơ và KHÔNG phải Gazebo hay robot thật.
% Cần đối chiếu lại từng giá trị trước khi áp dụng cho hệ thống thực tế.
p.dt = 0.005;               % s, bước tích phân
p.r = 0.08;                 % m, bán kính bánh chủ động
p.b = 0.45;                 % m, khoảng cách tâm hai bánh chủ động
p.mass = 75;               % kg, robot 55 kg (giả định) + khay 20 kg
p.length = 0.70;           % m, kích thước tham chiếu
p.width = 0.55;            % m
p.Iz = p.mass * (p.length^2 + p.width^2) / 12; % kg*m^2, xấp xỉ
p.Jw = 0.012;              % kg*m^2, quán tính mỗi bánh giả định
p.crr = 0.025;             % hệ số cản lăn giả định
p.viscous = 0.015;         % N*m*s/rad, cản nhớt bánh giả định
p.tau_limit = 8.0;         % N*m tại bánh, giới hạn GIẢ ĐỊNH mô phỏng
p.kp = 1.1;                % N*m/(rad/s), giả định, chưa hiệu chỉnh
p.ki = 3.0;                % N*m/rad, giả định, chưa hiệu chỉnh
p.kd = 0.02;              % N*m*s^2/rad, ví dụ hệ số D minh họa
p.derivative_filter_s = 0.04;  % s, thời gian lọc đạo hàm

% Ma trận khối lượng tương đương khi tọa độ bánh là [omega_R;omega_L].
common = p.mass*p.r^2/4;
rot = p.Iz*p.r^2/p.b^2;
p.M = [common+rot+p.Jw, common-rot;
       common-rot, common+rot+p.Jw];
end
