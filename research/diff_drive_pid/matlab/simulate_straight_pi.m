function d = simulate_straight_pi(p)
%SIMULATE_STRAIGHT_PI Chạy thẳng, tăng/giảm tốc và bám 1,2 m/s.
% Dữ liệu mô phỏng số đơn giản, không chứng minh docking hoặc an toàn.
t = (0:p.dt:6.2).'; n = numel(t);
d = zeros(n,12);
omega = zeros(2,1); integ = zeros(2,1); pose = zeros(3,1);
for k = 1:n
    tk = t(k);
    if tk < 0.5
        vref = 0;
    elseif tk < 2.0
        vref = min(1.2, 0.8*(tk-0.5));
    elseif tk < 4.0
        vref = 1.2;
    elseif tk < 5.5
        vref = max(0, 1.2-0.8*(tk-4.0));
    else
        vref = 0;
    end
    [omega,integ,desired,tau] = wheel_pi_step(omega,integ,vref,0,p);
    [v,w] = diffdrive_forward(omega,p);
    middle = pose(3) + 0.5*w*p.dt;
    pose(1) = pose(1) + v*cos(middle)*p.dt;
    pose(2) = pose(2) + v*sin(middle)*p.dt;
    pose(3) = pose(3) + w*p.dt;
    % cột: t,vref,v,wr_ref,wl_ref,wr,wl,tauR,tauL,x,y,theta
    d(k,:) = [tk,vref,v,desired(1),desired(2),omega(1),omega(2), ...
              tau(1),tau(2),pose(1),pose(2),pose(3)];
end
end
