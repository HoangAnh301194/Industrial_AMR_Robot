function d = simulate_turning(p)
%SIMULATE_TURNING Đi thẳng, rẽ cong và quay tại chỗ.
t = (0:p.dt:8.0).'; n = numel(t);
d = zeros(n,17);
omega = zeros(2,1); integ = zeros(2,1);
pose = zeros(3,1); ideal = zeros(3,1);
for k = 1:n
    tk = t(k);
    if tk < 0.6
        vr = 0; wr = 0;
    elseif tk < 1.8
        vr = 0.60; wr = 0;
    elseif tk < 5.2
        vr = 0.60; wr = 0.50;
    elseif tk < 6.3
        vr = 0; wr = -0.65;
    else
        vr = 0; wr = 0;
    end
    [omega,integ,desired,tau] = wheel_pi_step(omega,integ,vr,wr,p);
    [v,w] = diffdrive_forward(omega,p);
    mid = pose(3)+0.5*w*p.dt;
    pose(1:2) = pose(1:2)+[v*cos(mid);v*sin(mid)]*p.dt;
    pose(3) = pose(3)+w*p.dt;
    mid_ref = ideal(3)+0.5*wr*p.dt;
    ideal(1:2) = ideal(1:2)+[vr*cos(mid_ref);vr*sin(mid_ref)]*p.dt;
    ideal(3) = ideal(3)+wr*p.dt;
    % t,vr,wr,v,w,wr_ref,wl_ref,wr,wl,tauR,tauL,x,y,theta,xref,yref,thetaRef
    d(k,:)=[tk,vr,wr,v,w,desired(1),desired(2),omega(1),omega(2), ...
            tau(1),tau(2),pose.',ideal.'];
end
end
