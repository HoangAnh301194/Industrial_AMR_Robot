function plot_from_python_csv()
%PLOT_FROM_PYTHON_CSV Vẽ lại 9 đồ thị trực tiếp từ CSV Python đã có.
% Hữu ích để đối chiếu hai nền tảng mà KHÔNG cần chạy lại mô hình MATLAB.
here=fileparts(mfilename('fullpath'));
root=fileparts(here); data_dir=fullfile(root,'examples','data');
assets=fullfile(root,'docs','assets');
turn=readmatrix(fullfile(data_dir,'turning_scenario.csv'));
pdata=readmatrix(fullfile(data_dir,'compare_p.csv'));
pidata=readmatrix(fullfile(data_dir,'compare_pi.csv'));
route=readmatrix(fullfile(data_dir,'waypoint_scenario.csv'));
if ~exist(assets,'dir'),mkdir(assets);end
p=amr_params();
wp=[0 0;3 0;3 2;6 2];names={'A','B','C','D'};
% Các sự kiện chỉ dùng để đánh dấu thời gian, truy ngược từ goal_index.
events=zeros(0,6);
lastGoal=2;
for k=2:size(route,1)
    goal=route(k,7)+1; % CSV gốc Python lưu chỉ số 0-based
    if goal > lastGoal
        events(end+1,:)=[route(k,1),lastGoal,route(k,2),route(k,3),0.12,1]; %#ok<AGROW>
        lastGoal=goal;
    end
end
if ~isempty(route) && route(end,7)==3
    events(end+1,:)=[route(end,1),4,route(end,2),route(end,3),route(end,8),1];
end
% Chỉ xuất 9 biểu đồ dựa trên cột CSV: không dùng tiện ích plot_all cho kịch bản straight.
matlab_plot_csv_only(turn,pdata,pidata,route,events,wp,names,assets,p);
fprintf('Da doc CSV Python va ve lai 9 bieu do co tien to matlab_csv_.\n');
end

function matlab_plot_csv_only(d,p,pi,route,events,wp,names,assets,cfg)
% CSV gốc dùng cùng thứ tự cột với file MATLAB turning, trừ tọa độ route.
styles = {
 'wheel_speed_tracking', [6,8,7,9]; ...
 'body_linear_speed', [2,4,0,0]; ...
 'body_yaw_rate', [3,5,0,0]; ...
 'wheel_speed_error', [6,8,7,9]; ...
 'wheel_torque', [10,11,0,0]};
for i=1:size(styles,1)
   f=figure('Visible','off','Color','w');
   cols=styles{i,2};kind=styles{i,1};
   switch kind
    case 'wheel_speed_tracking'
     plot(d(:,1),d(:,cols(1)),'--',d(:,1),d(:,cols(2)), ...
       d(:,1),d(:,cols(3)),'--',d(:,1),d(:,cols(4))); legend('R dat','R do','L dat','L do');
    case 'wheel_speed_error'
     plot(d(:,1),d(:,cols(1))-d(:,cols(2)),d(:,1),d(:,cols(3))-d(:,cols(4))); legend('R','L');
    otherwise
     plot(d(:,1),d(:,cols(1)),'--',d(:,1),d(:,cols(2)));legend('1','2');
   end
   grid on;xlabel('Thoi gian (s)');title(strrep(kind,'_',' '));
   amr_save_png(f,fullfile(assets,['matlab_csv_',kind,'.png']));
end
f=figure('Visible','off','Color','w');plot(d(:,15),d(:,16),'--');hold on;plot(d(:,12),d(:,13));grid on;axis equal;title('Turning XY from Python CSV');legend('Ideal','Simulated');
amr_save_png(f,fullfile(assets,'matlab_csv_turning_xy.png'));
f=figure('Visible','off','Color','w');plot(p(:,1),p(:,2),'--');hold on;plot(p(:,1),p(:,3));plot(pi(:,1),pi(:,3));grid on;title('P vs PI - Python CSV');legend('Ref','P','PI');
amr_save_png(f,fullfile(assets,'matlab_csv_p_vs_pi.png'));
f=figure('Visible','off','Color','w');plot(route(:,2),route(:,3));hold on;plot(wp(:,1),wp(:,2),'o--');grid on;axis equal;title('A-B-C-D - Python CSV');
amr_save_png(f,fullfile(assets,'matlab_csv_waypoints.png'));
f=figure('Visible','off','Color','w');plot(route(:,1),route(:,8));hold on;plot(xlim,[0.12 0.12],':k');grid on;title('Distance to active goal - Python CSV');
amr_save_png(f,fullfile(assets,'matlab_csv_waypoint_distance.png'));
end
