function [omega_next, integ_next, deriv_next, desired, tau] = wheel_pid_step( ...
  omega, integ, deriv_state, omega_prev, vref, wref, p, kp, ki, kd)
%WHEEL_PID_STEP Mot buoc mo phong PID toc do 2 banh vi sai.
% D tren phan hoi toc do do (derivative on measurement), loc thong thap.
% Nhu vay tranh xung D khi tin hieu dat thay doi dot ngot.
% Kp, Ki, Kd khong phai he so da tune tren robot that.
if nargin < 8 || isempty(kp), kp=p.kp; end
if nargin < 9 || isempty(ki), ki=p.ki; end
if nargin < 10 || isempty(kd), kd=p.kd; end

desired=diffdrive_inverse(vref,wref,p);
error=desired-omega;
raw_d=(omega-omega_prev)/p.dt;
alpha=p.derivative_filter_s/(p.derivative_filter_s+p.dt);
deriv_next=alpha*deriv_state+(1-alpha)*raw_d;
raw=kp*error+ki*integ-kd*deriv_next;
% Conditional integration chi danh cho thanh phan I.
eligible=(abs(raw)<p.tau_limit) | (sign(error)~=sign(raw));
integ_next=integ+p.dt*(error.*double(eligible));
tau=max(-p.tau_limit,min(p.tau_limit,kp*error+ki*integ_next-kd*deriv_next));
tau_roll=(p.crr*p.mass*9.81*p.r/2).*tanh(omega/0.2);
domega=p.M\(tau-p.viscous*omega-tau_roll);
omega_next=omega+p.dt*domega;
end
