function [wp, names, d, events] = simulate_waypoints(p)
%SIMULATE_WAYPOINTS Điều khiển vị trí đơn giản A->B->C->D.
% Điểm là TỌA ĐỘ MINH HỌA, không phải bản đồ kho/SLAM/Nav2.
% Ngưỡng chuyển trạm 0,12 m, KHÔNG chứng minh docking 30 mm.
wp=[0,0;3,0;3,2;6,2]; names={'A','B','C','D'};
idx=2; omega=zeros(2,1); integ=zeros(2,1); pose=zeros(3,1);
t=(0:p.dt:70).'; d=zeros(numel(t),10);
events=zeros(0,6); % [time, goal_idx, x, y, distance, success]
actualN=0;
for k=1:numel(t)
    tk=t(k); target=wp(idx,:).';
    dx=target(1)-pose(1); dy=target(2)-pose(2);
    dist=hypot(dx,dy);
    if dist < 0.12
        events(end+1,:)=[tk,idx,pose(1),pose(2),dist,1]; %#ok<AGROW>
        if idx == size(wp,1)
            actualN=actualN+1;
            d(actualN,:)=[tk,pose.',0,0,idx,dist,omega.'];
            break;
        end
        idx=idx+1;
        target=wp(idx,:).';
        dx=target(1)-pose(1);dy=target(2)-pose(2);dist=hypot(dx,dy);
    end
    ang_err = atan2(sin(atan2(dy,dx)-pose(3)),cos(atan2(dy,dx)-pose(3)));
    vcmd = min(0.80,max(0,0.95*dist))*max(0,cos(ang_err));
    wcmd = max(-0.9,min(0.9,1.7*ang_err));
    [omega,integ] = wheel_pi_step(omega,integ,vcmd,wcmd,p);
    [v,w] = diffdrive_forward(omega,p);
    mid=pose(3)+0.5*w*p.dt;
    pose(1:2) = pose(1:2)+v*[cos(mid);sin(mid)]*p.dt;
    pose(3) = pose(3)+w*p.dt;
    actualN=actualN+1;
    d(actualN,:)=[tk,pose.',vcmd,wcmd,idx,dist,omega.'];
end
d=d(1:actualN,:);
end
