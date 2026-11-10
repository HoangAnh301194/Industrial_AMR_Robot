function amr_plot_all(p, straight, turn, cmpP, cmpPI, wp, names, route, events, assets, pidcmp)
%AMR_PLOT_ALL Tạo 14 biểu đồ từ dữ liệu mô phỏng do MATLAB tự tính.
% Tên có tiền tố matlab_ để KHÔNG ghi đè hình Python của nhóm.
if ~exist(assets,'dir'), mkdir(assets); end
% 1 - tốc độ đi thẳng
f=newfig();plot(straight(:,1),straight(:,2),'--','LineWidth',1.4);hold on;
plot(straight(:,1),straight(:,3),'LineWidth',1.6);grid on;
title('PI - Bam van toc tien khi tang/giam toc');xlabel('Thoi gian (s)');ylabel('v (m/s)');
legend('Toc do dat','Toc do mo phong','Location','best');
amr_save_png(f,fullfile(assets,'matlab_straight_speed.png'));
% 2 - mô-men đi thẳng
f=newfig();plot(straight(:,1),straight(:,8),'LineWidth',1.4);hold on;
plot(straight(:,1),straight(:,9),'--','LineWidth',1.3);
plot(xlim,[p.tau_limit,p.tau_limit],':k');plot(xlim,[-p.tau_limit,-p.tau_limit],':k');grid on;
title('Mo-men hai banh khi chay thang');xlabel('Thoi gian (s)');ylabel('Mo-men tai banh (N.m)');
legend('Banh phai','Banh trai','Gioi han +','Gioi han -','Location','best');
amr_save_png(f,fullfile(assets,'matlab_straight_torque.png'));
% 3 - quỹ đạo thẳng
f=newfig();plot(straight(:,10),straight(:,11),'LineWidth',1.7);grid on;axis equal;
title('Quy dao XY - chay thang');xlabel('X (m)');ylabel('Y (m)');
amr_save_png(f,fullfile(assets,'matlab_straight_xy.png'));
% 4 - tốc độ bánh khi rẽ
f=newfig();plot(turn(:,1),turn(:,6),'--');hold on;
plot(turn(:,1),turn(:,8),'LineWidth',1.4);
plot(turn(:,1),turn(:,7),'--');plot(turn(:,1),turn(:,9),'LineWidth',1.4);grid on;
title('PI - Bam toc do hai banh khi re');xlabel('Thoi gian (s)');ylabel('Toc do banh (rad/s)');
legend('Phai - dat','Phai - phan hoi','Trai - dat','Trai - phan hoi','Location','best');
amr_save_png(f,fullfile(assets,'matlab_wheel_speed_tracking.png'));
% 5 - v(t)
f=newfig();plot(turn(:,1),turn(:,2),'--');hold on;plot(turn(:,1),turn(:,4),'LineWidth',1.5);grid on;
title('Toc do tien than robot');xlabel('Thoi gian (s)');ylabel('v (m/s)');
legend('v dat','v mo phong','Location','best');
amr_save_png(f,fullfile(assets,'matlab_body_linear_speed.png'));
% 6 - yaw(t)
f=newfig();plot(turn(:,1),turn(:,3),'--');hold on;plot(turn(:,1),turn(:,5),'LineWidth',1.5);grid on;
title('Toc do quay than robot');xlabel('Thoi gian (s)');ylabel('omega (rad/s)');
legend('Omega dat','Omega mo phong','Location','best');
amr_save_png(f,fullfile(assets,'matlab_body_yaw_rate.png'));
% 7 - sai số tốc độ hai bánh
f=newfig();plot(turn(:,1),turn(:,6)-turn(:,8),'LineWidth',1.4);hold on;
plot(turn(:,1),turn(:,7)-turn(:,9),'LineWidth',1.4);grid on;
title('Sai so toc do hai banh');xlabel('Thoi gian (s)');ylabel('Sai so toc do (rad/s)');
legend('Phai','Trai','Location','best');
amr_save_png(f,fullfile(assets,'matlab_wheel_speed_error.png'));
% 8 - quỹ đạo rẽ
f=newfig();plot(turn(:,15),turn(:,16),'--','LineWidth',1.3);hold on;
plot(turn(:,12),turn(:,13),'LineWidth',1.7);grid on;axis equal;
title('Quy dao di chuyen cong');xlabel('X (m)');ylabel('Y (m)');
legend('Tich phan lenh ly tuong','Mo phong dong luc + PI','Location','best');
amr_save_png(f,fullfile(assets,'matlab_turning_xy.png'));
% 9 - So sánh P/PI
f=newfig();plot(cmpP(:,1),cmpP(:,2),'--','LineWidth',1.3);hold on;
plot(cmpP(:,1),cmpP(:,3),'LineWidth',1.3);
plot(cmpPI(:,1),cmpPI(:,3),'LineWidth',1.3);grid on;
title('So sanh P va PI khi thay doi tai 3.0-4.5 s');xlabel('Thoi gian (s)');ylabel('v (m/s)');
legend('Toc do dat','P','PI','Location','best');
amr_save_png(f,fullfile(assets,'matlab_p_vs_pi_disturbance.png'));
% 10 - torque during turning
f=newfig();plot(turn(:,1),turn(:,10),'LineWidth',1.4);hold on;
plot(turn(:,1),turn(:,11),'LineWidth',1.4);
plot(xlim,[p.tau_limit,p.tau_limit],':k');plot(xlim,[-p.tau_limit,-p.tau_limit],':k');grid on;
title('Mo-men hai banh khi doi huong');xlabel('Thoi gian (s)');ylabel('Mo-men tai banh (N.m)');
legend('Phai','Trai','Gioi han +','Gioi han -','Location','best');
amr_save_png(f,fullfile(assets,'matlab_turning_torque.png'));
% 11 - minh họa A-B-C-D
f=newfig();plot(route(:,2),route(:,3),'LineWidth',1.6);hold on;
plot(wp(:,1),wp(:,2),'--');plot(wp(:,1),wp(:,2),'o');
for k=1:numel(names),text(wp(k,1)+0.08,wp(k,2)+0.08,names{k},'FontSize',11);end
grid on;axis equal;
title('A -> B -> C -> D: Toa do MINH HOA, khong phai Nav2');
xlabel('X (m)');ylabel('Y (m)');
legend('Quy dao mo phong','Noi cac diem','Waypoint','Location','best');
amr_save_png(f,fullfile(assets,'matlab_waypoint_abcd.png'));
% 12 - khoảng cách tới waypoint
f=newfig();plot(route(:,1),route(:,8),'LineWidth',1.4);hold on;
plot(xlim,[0.12,0.12],':k');
for k=1:size(events,1)
    x=events(k,1);
    yl=ylim;
    plot([x,x],yl,':');
end
grid on;
title('Khoang cach den waypoint dang phuc vu');xlabel('Thoi gian (s)');ylabel('Khoang cach (m)');
legend('Khoang cach','Nguong chuyen 0.12m','Location','best');
amr_save_png(f,fullfile(assets,'matlab_waypoint_distance.png'));
% 13 - PI/PID speed illustration
f=newfig();plot(pidcmp(:,1),pidcmp(:,2),'--','LineWidth',1.2);hold on;
plot(pidcmp(:,1),pidcmp(:,3),'LineWidth',1.3);
plot(pidcmp(:,1),pidcmp(:,4),'LineWidth',1.3);grid on;
title('So sanh PI va PID voi bo tham so minh hoa');
xlabel('Thoi gian (s)');ylabel('v (m/s)');
legend('Dat','PI','PID','Location','best');
amr_save_png(f,fullfile(assets,'matlab_pi_vs_pid_speed.png'));
% 14 - PI vs PID torque (right wheel)
f=newfig();plot(pidcmp(:,1),pidcmp(:,5),'LineWidth',1.3);hold on;
plot(pidcmp(:,1),pidcmp(:,6),'LineWidth',1.3);
plot(xlim,[p.tau_limit,p.tau_limit],':k');plot(xlim,[-p.tau_limit,-p.tau_limit],':k');grid on;
title('Mo-men banh phai PI va PID');xlabel('Thoi gian (s)');ylabel('Mo-men (N.m)');
legend('PI','PID','Gioi han +','Gioi han -','Location','best');
amr_save_png(f,fullfile(assets,'matlab_pi_vs_pid_torque.png'));
end

function f = newfig()
f=figure('Visible','off','Color','w','Position',[100,100,900,420]);
set(f,'DefaultAxesFontSize',11);
end
