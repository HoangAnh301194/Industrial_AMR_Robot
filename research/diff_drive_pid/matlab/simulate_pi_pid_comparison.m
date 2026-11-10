function d=simulate_pi_pid_comparison(p)
%SIMULATE_PI_PID_COMPARISON Minh hoa PI vs PID cho lenh toc do thay doi.
% Gia dinh PI (Kp,Ki) va them Kd; khong khang dinh PID luon tot hon.
t=(0:p.dt:5).';n=numel(t);d=zeros(n,6);
wheel_pi=zeros(2,1);int_pi=zeros(2,1);
wheel_pid=zeros(2,1);int_pid=zeros(2,1);d_filt=zeros(2,1);wheel_prev=zeros(2,1);
for k=1:n
    tk=t(k);
    if tk < 0.5
        vref=0;
    elseif tk<1.8
        vref=0.85;
    elseif tk<2.6
        vref=0.25;
    else
        vref=0.85;
    end
    [wheel_pi,int_pi,~,tau_pi]=wheel_pi_step(wheel_pi,int_pi,vref,0,p);
    % Luu gia tri truoc buoc tich phan: D dung sai phan omega(k)-omega(k-1).
    % NEU luu wheel_prev=wheel_pid sau buoc cap nhat, dao ham luon = 0.
    old_wheel=wheel_pid;
    [wheel_pid,int_pid,d_filt,~,tau_pid]=wheel_pid_step( ...
      wheel_pid,int_pid,d_filt,wheel_prev,vref,0,p);
    wheel_prev=old_wheel;
    [v_pi,~]=diffdrive_forward(wheel_pi,p);
    [v_pid,~]=diffdrive_forward(wheel_pid,p);
    d(k,:)=[tk,vref,v_pi,v_pid,tau_pi(1),tau_pid(1)];
end
end
