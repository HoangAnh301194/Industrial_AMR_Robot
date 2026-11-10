function [v, yaw_rate] = diffdrive_forward(omega, p)
%DIFFDRIVE_FORWARD Động học thuận từ [omega_R; omega_L] (rad/s).
v = 0.5*p.r*(omega(1)+omega(2));
yaw_rate = p.r*(omega(1)-omega(2))/p.b;
end
