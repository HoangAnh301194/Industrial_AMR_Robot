#!/usr/bin/env python3
"""Generate reproducible explanatory plots for a hypothetical differential-drive AMR.

All parameters are assumptions based on the group's preliminary sizing. These are
NOT ROS/Gazebo results, not real robot measurements, and not PID gains to deploy.
Requires: numpy, matplotlib; run from any directory.
"""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from simulate_diff_drive_pi import (
    DT, R, TRACK, MASS, IZ, JW, CRR, VISCOUS, TAU_LIMIT,
    KP, KI, MINV, inv_kin, forward_kin,
)

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'assets'
DATA = ROOT / 'examples' / 'data'

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                     'axes.titlesize': 12, 'figure.dpi': 100})


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def step_wheels(state, integral, vref, wref, dt=DT, kp=KP, ki=KI, extra_load=(0,0)):
    desired = inv_kin(vref, wref)
    e = desired - state
    raw = kp*e + ki*integral
    eligible = (np.abs(raw) < TAU_LIMIT) | (np.sign(e) != np.sign(raw))
    integral = integral + dt*e*eligible
    torque = np.clip(kp*e + ki*integral, -TAU_LIMIT, TAU_LIMIT)
    rolling = CRR*MASS*9.81*R/2.0 * np.tanh(state/0.2)
    load = np.array(extra_load, dtype=float)
    accel = MINV @ (torque - VISCOUS*state - rolling - load)
    next_wheels = state + dt*accel
    return next_wheels, integral, desired, torque


def motion_profile(t):
    # Command only, contains sharp transitions intentionally for response assessment.
    if t < 0.6: return 0.0, 0.0
    if t < 1.8: return 0.60, 0.0
    if t < 5.2: return 0.60, 0.50
    if t < 6.3: return 0.0, -0.65
    return 0.0, 0.0


def simulate_motion(duration=8.0):
    time = np.arange(0, duration+DT/2, DT)
    wheel = np.zeros(2)
    integ = np.zeros(2)
    pose = np.zeros(3)
    ideal = np.zeros(3)
    rows=[]
    for t in time:
        vr, wr = motion_profile(t)
        wheel, integ, des, torque = step_wheels(wheel, integ, vr, wr)
        v, w = forward_kin(wheel)
        theta_mid = pose[2] + 0.5*w*DT
        pose[0] += v*np.cos(theta_mid)*DT
        pose[1] += v*np.sin(theta_mid)*DT
        pose[2] += w*DT
        ideal_mid=ideal[2]+0.5*wr*DT
        ideal[0]+=vr*np.cos(ideal_mid)*DT
        ideal[1]+=vr*np.sin(ideal_mid)*DT
        ideal[2]+=wr*DT
        rows.append([t,vr,wr,v,w,des[0],des[1],wheel[0],wheel[1],
                     torque[0],torque[1],*pose,*ideal])
    return np.array(rows)


def sim_compare(ki, duration=7.0):
    time=np.arange(0,duration+DT/2,DT)
    wheels=np.zeros(2);integ=np.zeros(2);rows=[]
    for t in time:
        v_ref=0.0 if t < .5 else .85
        ext = (0.0, 1.7) if 3.0 <= t < 4.5 else (0.0,0.0)
        wheels,integ,_,torque=step_wheels(wheels,integ,v_ref,0.0,ki=ki,extra_load=ext)
        v,w=forward_kin(wheels)
        rows.append([t,v_ref,v,w,torque[0],torque[1]])
    return np.asarray(rows)


def simulate_waypoints(duration=70):
    # Hypothetical XY positions; not the user's factory, not Nav2, no obstacles.
    waypoints=np.array([[0.0,0.0],[3.0,0.0],[3.0,2.0],[6.0,2.0]])
    names=['A','B','C','D']
    goal_idx=1
    wheels=np.zeros(2);integ=np.zeros(2);pose=np.zeros(3)
    rows=[];events=[]
    for t in np.arange(0, duration+DT/2,DT):
        target = waypoints[goal_idx]
        dx,dy = target[0]-pose[0], target[1]-pose[1]
        dist=np.hypot(dx,dy)
        if dist < 0.12:
            events.append((t,names[goal_idx],pose[0],pose[1],dist))
            if goal_idx==len(waypoints)-1:
                rows.append([t,*pose,0,0,goal_idx,dist,*wheels]);break
            goal_idx += 1
            target=waypoints[goal_idx]
            dx,dy=target[0]-pose[0],target[1]-pose[1]
            dist=np.hypot(dx,dy)
        err=wrap_angle(np.arctan2(dy,dx)-pose[2])
        v_cmd = np.clip(0.95*dist,0.0,0.80)*max(0,np.cos(err))
        w_cmd = np.clip(1.7*err,-0.9,0.9)
        wheels,integ,_,_=step_wheels(wheels,integ,v_cmd,w_cmd)
        v,w=forward_kin(wheels)
        mid=pose[2]+0.5*w*DT
        pose[0]+=v*np.cos(mid)*DT;pose[1]+=v*np.sin(mid)*DT;pose[2]+=w*DT
        rows.append([t,*pose,v_cmd,w_cmd,goal_idx,dist,*wheels])
    return waypoints,names,np.asarray(rows),events


def csv_out(path, cols, arr):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(cols);writer.writerows(np.round(arr,6))


def make_plots():
    ASSETS.mkdir(exist_ok=True,parents=True);DATA.mkdir(exist_ok=True,parents=True)
    dat=simulate_motion();t=dat[:,0]
    csv_out(DATA/'turning_scenario.csv',
        ['t_s','v_ref_mps','yaw_ref_rps','v_mps','yaw_rps','right_ref_rps','left_ref_rps',
         'right_rps','left_rps','right_Nm','left_Nm','x_m','y_m','theta_rad','x_ref_m','y_ref_m','theta_ref_rad'],dat)
    # 1 wheel speed tracking
    fig,ax=plt.subplots(figsize=(10,4.4));
    ax.plot(t,dat[:,5],ls='--',label='Bánh phải - đặt')
    ax.plot(t,dat[:,7],label='Bánh phải - mô phỏng')
    ax.plot(t,dat[:,6],ls='--',label='Bánh trái - đặt')
    ax.plot(t,dat[:,8],label='Bánh trái - mô phỏng')
    ax.set(xlabel='Thời gian (s)',ylabel='Tốc độ góc bánh (rad/s)',title='PI: bám tốc độ bánh khi đi thẳng, vào cua và quay tại chỗ')
    ax.grid(alpha=.3);ax.legend(ncol=2,fontsize=9);fig.tight_layout()
    fig.savefig(ASSETS/'extended_wheel_speed_tracking.png',dpi=190);plt.close(fig)
    # 2 body speed
    fig,ax=plt.subplots(figsize=(10,4.2));ax.plot(t,dat[:,1],'--',label='Lệnh v');ax.plot(t,dat[:,3],label='Mô phỏng v');
    ax.set(xlabel='Thời gian (s)',ylabel='Tốc độ tiến (m/s)',title='Tốc độ tiến: yêu cầu và giá trị phản hồi');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_body_linear_speed.png',dpi=190);plt.close(fig)
    # 3 yaw
    fig,ax=plt.subplots(figsize=(10,4.2));ax.plot(t,dat[:,2],'--',label='Lệnh ω');ax.plot(t,dat[:,4],label='Mô phỏng ω');
    ax.set(xlabel='Thời gian (s)',ylabel='Tốc độ quay (rad/s)',title='Tốc độ góc thân robot trong tình huống đổi hướng');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_body_yaw_rate.png',dpi=190);plt.close(fig)
    # 4 error 
    fig,ax=plt.subplots(figsize=(10,4.2));ax.plot(t,dat[:,5]-dat[:,7],label='Sai số bánh phải');ax.plot(t,dat[:,6]-dat[:,8],label='Sai số bánh trái');
    ax.axhline(0,linewidth=0.9,color='gray');ax.set(xlabel='Thời gian (s)',ylabel='Sai số tốc độ bánh (rad/s)',title='Sai số bám tốc độ; tăng khi có thay đổi lệnh');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_wheel_error.png',dpi=190);plt.close(fig)
    # 5 turning XY
    fig,ax=plt.subplots(figsize=(7,5.3));ax.plot(dat[:,14],dat[:,15],label='Tích phân lệnh (lý tưởng)',ls='--');ax.plot(dat[:,11],dat[:,12],label='Đường đi theo tốc độ bánh (PI)');
    ax.plot(dat[0,11],dat[0,12],marker='o',label='Bắt đầu');ax.plot(dat[-1,11],dat[-1,12],marker='s',label='Kết thúc');
    ax.set(xlabel='X (m)',ylabel='Y (m)',title='Quỹ đạo XY: chuyển động cong từ mô hình vi sai');ax.axis('equal');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_turning_trajectory.png',dpi=190);plt.close(fig)
    # 6 compare control P vs PI with external load
    p=sim_compare(0.0);pi=sim_compare(KI)
    csv_out(DATA/'compare_p.csv',['t_s','v_ref_mps','v_mps','yaw_rps','torque_r_Nm','torque_l_Nm'],p)
    csv_out(DATA/'compare_pi.csv',['t_s','v_ref_mps','v_mps','yaw_rps','torque_r_Nm','torque_l_Nm'],pi)
    fig,ax=plt.subplots(figsize=(10,4.4));ax.plot(p[:,0],p[:,1],'--',label='Tốc độ đặt');ax.plot(p[:,0],p[:,2],label='Chỉ P');ax.plot(pi[:,0],pi[:,2],label='PI');
    ax.axvspan(3,4.5,alpha=.12,color='gray',label='Nhiễu tải mô phỏng');
    ax.set(xlabel='Thời gian (s)',ylabel='Tốc độ tiến (m/s)',title='So sánh P và PI dưới tác động tăng tải một bánh');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_p_vs_pi_disturbance.png',dpi=190);plt.close(fig)
    # 7 torque during turn
    fig,ax=plt.subplots(figsize=(10,4.2));ax.plot(t,dat[:,9],label='Mô-men bánh phải');ax.plot(t,dat[:,10],label='Mô-men bánh trái');
    ax.axhline(TAU_LIMIT,ls=':',color='gray',label='Giới hạn ±8 N·m (giả định)');ax.axhline(-TAU_LIMIT,ls=':',color='gray');
    ax.set(xlabel='Thời gian (s)',ylabel='Mô-men bánh (N·m)',title='Mô-men đầu ra PI khi thay đổi chuyển động');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_wheel_torque.png',dpi=190);plt.close(fig)
    # 8 simulated A→B→C→D path (custom proportional outer loop + PI wheel loops)
    wp,names,route,events=simulate_waypoints()
    csv_out(DATA/'waypoint_scenario.csv',['t_s','x_m','y_m','theta_rad','v_ref_mps','w_ref_rps','active_goal_index','distance_to_goal_m','right_rps','left_rps'],route)
    fig,ax=plt.subplots(figsize=(9,5));ax.plot(route[:,1],route[:,2],label='Quỹ đạo robot (mô hình số)');
    for (x,y),nm in zip(wp,names):
        ax.scatter([x],[y],s=56);ax.annotate(nm,(x,y),xytext=(8,8),textcoords='offset points',fontsize=13,weight='bold')
    ax.plot(wp[:,0],wp[:,1],ls='--',linewidth=1,alpha=.45,label='Nối thẳng các điểm nhiệm vụ')
    ax.set(xlabel='X (m)',ylabel='Y (m)',title='Minh họa quỹ đạo A → B → C → D (không phải Gazebo/Nav2)')
    ax.axis('equal');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_waypoint_abcd.png',dpi=190);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4.2));ax.plot(route[:,0],route[:,7],label='Khoảng cách tới trạm đang hướng tới');
    for tim,nm,x,y,d in events: ax.axvline(tim,ls='--',alpha=.55);ax.text(tim+.15,.65,f'Đến {nm}',rotation=90,va='bottom',fontsize=9)
    ax.axhline(.12,ls=':',color='gray',label='Ngưỡng đổi điểm 0,12 m (giả định)');ax.set(xlabel='Thời gian (s)',ylabel='Khoảng cách tới mục tiêu (m)',title='Khoảng cách robot tới waypoint hiện tại');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(ASSETS/'extended_waypoint_distance.png',dpi=190);plt.close(fig)
    # capture params and numerical results
    stats={
      'warning':'ILLUSTRATIVE PYTHON SIMULATION ONLY; no Gazebo/ROS/hardware verification',
      'parameters':{'wheel_radius_m':R,'wheel_separation_m':TRACK,'total_mass_kg':MASS,'I_z_kgm2':IZ,
                    'wheel_inertia_kgm2':JW,'rolling_resistance':CRR,'viscous_Nm_s_per_rad':VISCOUS,
                    'torque_limit_Nm_per_wheel':TAU_LIMIT,'kp_Nm_per_rad_s':KP,'ki_Nm_per_rad':KI,
                    'time_step_s':DT},
      'maneuver':{'max_wheel_speed_error_rps':float(np.max(np.abs(dat[:,5:7]-dat[:,7:9]))),
                  'total_path_distance_m':float(np.sum(np.hypot(np.diff(dat[:,11]),np.diff(dat[:,12])))),
                  'final_pose_m_rad':dat[-1,11:14].round(5).tolist()},
      'waypoint':{'simulated_points':{nm:wp[i].tolist() for i,nm in enumerate(names)},
                  'events':[{ 'time_s':round(t,3),'point':n,'distance_at_switch_m':round(d,5)} for t,n,x,y,d in events],
                  'completed_all':bool(any(ev[1]=='D' for ev in events)),
                  'end_time_s':round(float(route[-1,0]),3),
                  'note':'Tolerance 0.12m only indicates illustrative switch criterion; it does NOT prove docking accuracy 30mm'},
    }
    (DATA/'simulation_summary.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(stats,ensure_ascii=False,indent=2))
    assert np.allclose(np.array(forward_kin(inv_kin(.5,.3))),np.array([.5,.3]))
    assert stats['waypoint']['completed_all'],'waypoint controller did not reach D'
    assert np.max(np.abs(dat[:,9:11])) <= TAU_LIMIT+1e-6
    print('Saved', len(list(ASSETS.glob('extended_*.png'))),'plots')

if __name__=='__main__':
    make_plots()
