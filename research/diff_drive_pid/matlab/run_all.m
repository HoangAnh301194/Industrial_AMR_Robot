function run_all()
%RUN_ALL Tạo toàn bộ dữ liệu và 12 đồ thị MATLAB (không cần Simulink).
% Trong MATLAB: cd('.../research/diff_drive_pid/matlab'); run_all
% Sử dụng MATLAB cơ bản; các thông số PI đều là giả định minh họa.
here=fileparts(mfilename('fullpath'));
root=fileparts(here);  % research/diff_drive_pid
data_dir=fullfile(root,'examples','data');
assets=fullfile(root,'docs','assets');
if ~exist(data_dir,'dir'), mkdir(data_dir); end
if ~exist(assets,'dir'), mkdir(assets); end
addpath(here);
p=amr_params();
straight=simulate_straight_pi(p);
turn=simulate_turning(p);
compare_p=simulate_p_vs_pi(p,0);
compare_pi=simulate_p_vs_pi(p,p.ki);
[wp,names,route,events]=simulate_waypoints(p);
pidcmp=simulate_pi_pid_comparison(p);
validate_amr(p,straight,turn,route,events);
assert(all(isfinite(pidcmp(:))), 'PID mo phong co gia tri khong huu han');

amr_write_csv(fullfile(data_dir,'matlab_straight.csv'), ...
 {'t_s','v_ref_mps','v_mps','right_ref_rps','left_ref_rps','right_rps','left_rps', ...
  'right_Nm','left_Nm','x_m','y_m','theta_rad'},straight);
amr_write_csv(fullfile(data_dir,'matlab_turning.csv'), ...
 {'t_s','v_ref_mps','yaw_ref_rps','v_mps','yaw_rps','right_ref_rps','left_ref_rps', ...
  'right_rps','left_rps','right_Nm','left_Nm','x_m','y_m','theta_rad', ...
  'x_ref_m','y_ref_m','theta_ref_rad'},turn);
amr_write_csv(fullfile(data_dir,'matlab_compare_p.csv'), ...
 {'t_s','v_ref_mps','v_mps','yaw_rps','right_Nm','left_Nm'},compare_p);
amr_write_csv(fullfile(data_dir,'matlab_compare_pi.csv'), ...
 {'t_s','v_ref_mps','v_mps','yaw_rps','right_Nm','left_Nm'},compare_pi);
amr_write_csv(fullfile(data_dir,'matlab_waypoints.csv'), ...
 {'t_s','x_m','y_m','theta_rad','v_ref_mps','w_ref_rps', ...
 'active_goal_index','distance_to_goal_m','right_rps','left_rps'},route);
amr_write_csv(fullfile(data_dir,'matlab_waypoint_events.csv'), ...
 {'t_s','goal_index','x_m','y_m','distance_at_switch_m','success'},events);

amr_write_csv(fullfile(data_dir,'matlab_pi_pid_comparison.csv'), ...
 {'t_s','v_ref_mps','v_pi_mps','v_pid_mps','tau_r_pi_Nm','tau_r_pid_Nm'},pidcmp);

amr_plot_all(p,straight,turn,compare_p,compare_pi,wp,names,route,events,assets,pidcmp);
fprintf('Da xuat 14 anh PNG (tien to matlab_) vao: %s\n',assets);
fprintf('Da xuat 7 file CSV (tien to matlab_) vao: %s\n',data_dir);
fprintf('Chi mo phong so MATLAB voi gia dinh; khong phai ket qua robot that.\n');
end
