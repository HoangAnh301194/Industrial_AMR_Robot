function omega = diffdrive_inverse(v, yaw_rate, p)
%DIFFDRIVE_INVERSE Động học nghịch: [omega_R;omega_L] (rad/s).
% v: vận tốc tiến (m/s), yaw_rate: vận tốc quay (rad/s).
omega = [(v + 0.5*p.b*yaw_rate)/p.r;
         (v - 0.5*p.b*yaw_rate)/p.r];
end
