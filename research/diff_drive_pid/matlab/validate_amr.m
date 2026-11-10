function validate_amr(p, straight, turn, route, events)
%VALIDATE_AMR Smoke tests nhất quán toán học và ổn định số; KHÔNG phải nghiệm thu.
for k=1:6
    v=0.1*k; w=0.07*(k-3);
    [v2,w2]=diffdrive_forward(diffdrive_inverse(v,w,p),p);
    assert(abs(v-v2)<1e-11 && abs(w-w2)<1e-11, 'Dong hoc khong nhat quan');
end
assert(all(isfinite(straight(:))) && all(isfinite(turn(:))), 'Gia tri khong huu han');
assert(max(max(abs(straight(:,8:9)))) <= p.tau_limit + 1e-8, 'Vuot torque limit');
assert(max(max(abs(turn(:,10:11)))) <= p.tau_limit + 1e-8, 'Vuot torque limit');
cruise=straight(:,1)>3.0 & straight(:,1)<3.8;
meanV=mean(straight(cruise,3));
assert(abs(meanV-1.2)<0.07, 'Speed tracking khong dat smoke test');
assert(~isempty(events) && any(events(:,2)==4), 'Chua den D');
assert(~isempty(route) && all(isfinite(route(:))), 'Waypoint du lieu khong hop le');
fprintf('PASS smoke tests. Van toc tb mo phong doan di thang = %.4f m/s\n',meanV);
fprintf('So lan chuyen diem = %d; ket thuc t=%.3f s\n',size(events,1),route(end,1));
fprintf('CHU Y: Khong phai Gazebo/Nav2, khong chung minh docking 30 mm.\n');
end
