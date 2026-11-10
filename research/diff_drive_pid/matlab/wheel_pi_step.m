function [omega_next, integ_next, desired, tau] = wheel_pi_step( ...
    omega, integ, vref, wref, p, kp, ki, extra_load)
%WHEEL_PI_STEP Một bước mô phỏng động lực học + PI của HAI bánh.
% M*dot(omega) = tau - c*omega - tau_roll - extra_load.
% M là ma trận khối lượng quán tính tương đương có liên kết hai bánh.
% omega, integ, extra_load: vector [bánh phải; bánh trái].
if nargin < 6 || isempty(kp), kp = p.kp; end
if nargin < 7 || isempty(ki), ki = p.ki; end
if nargin < 8 || isempty(extra_load), extra_load = [0;0]; end

desired = diffdrive_inverse(vref, wref, p);
error = desired - omega;
raw = kp*error + ki*integ;
% Chống tích phân tích lũy khi bão hòa (conditional integration).
eligible = (abs(raw) < p.tau_limit) | (sign(error) ~= sign(raw));
integ_next = integ + p.dt*(error .* double(eligible));
tau = max(-p.tau_limit, min(p.tau_limit, kp*error + ki*integ_next));

% Cản lăn được làm mượt quanh tốc độ 0 để tránh chuyển dấu đột ngột.
tau_roll = (p.crr*p.mass*9.81*p.r/2) .* tanh(omega/0.2);
domega = p.M \ (tau - p.viscous*omega - tau_roll - extra_load);
omega_next = omega + p.dt*domega;   % Euler tiến (mô hình minh họa)
end
