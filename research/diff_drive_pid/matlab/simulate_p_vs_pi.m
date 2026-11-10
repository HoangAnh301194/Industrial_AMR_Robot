function d = simulate_p_vs_pi(p, ki)
%SIMULATE_P_VS_PI So sánh vòng P / PI với nhiễu mô-men một bánh.
% Nhiễu 1,7 N*m tại bánh trái từ 3,0 đến 4,5 s là GIẢ ĐỊNH.
t = (0:p.dt:7.0).'; n = numel(t);
d = zeros(n,6); omega=zeros(2,1); integ=zeros(2,1);
for k=1:n
    tk=t(k);
    if tk < 0.5, vref=0; else, vref=0.85; end
    if tk >= 3.0 && tk < 4.5
        ext=[0;1.7];
    else
        ext=[0;0];
    end
    [omega,integ,~,tau] = wheel_pi_step(omega,integ,vref,0,p,p.kp,ki,ext);
    [v,w] = diffdrive_forward(omega,p);
    d(k,:)=[tk,vref,v,w,tau(1),tau(2)];
end
end
