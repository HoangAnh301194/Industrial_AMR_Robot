#!/usr/bin/env python3
"""Python cross-check of MATLAB's wheel_pid_step.m illustrative two-wheel PID.

This is a synthetic numerical model, not a MATLAB run, not Gazebo and not hardware.
Uses only numpy and matplotlib. Produces PNG, CSV and machine-readable metrics.
"""
from __future__ import annotations
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ASSETS = ROOT / 'docs' / 'assets'
DATA = HERE / 'data'
ASSETS.mkdir(parents=True, exist_ok=True)
DATA.mkdir(parents=True, exist_ok=True)
DT = .005
R, B, MASS = .08, .45, 75.0
IZ = MASS*(.7**2+.55**2)/12
JW=.012
CRR=.025
VISCOUS=.015
TORQUE_LIMIT=8.
KP, KI, KD = 1.1, 3., .02
FILTER=.04
c=MASS*R**2/4
rot=IZ*R**2/B**2
M=np.array([[c+rot+JW, c-rot],[c-rot,c+rot+JW]])
INV=np.linalg.inv(M)

def setpoint(t):
    if t < .5: return 0.
    if t < 1.8: return .85
    if t < 2.6: return .25
    return .85

def simulate(pid: bool):
    time=np.arange(0,5+DT/2,DT)
    omega=np.zeros(2); i_state=np.zeros(2); d_state=np.zeros(2)
    omega_prev=np.zeros(2)
    rows=[]
    for t in time:
        vref=setpoint(t)
        wheel_ref=np.array([vref/R,vref/R])
        e=wheel_ref-omega
        if pid:
            # Same derivative on measurement used in corrected MATLAB version.
            d_raw=(omega-omega_prev)/DT
            a=FILTER/(FILTER+DT)
            d_state=a*d_state+(1-a)*d_raw
            d_term=KD*d_state
        else:
            d_term=np.zeros(2)
        raw=KP*e+KI*i_state-d_term
        eligible=(np.abs(raw)<TORQUE_LIMIT)|(np.sign(e)!=np.sign(raw))
        i_state=i_state+DT*e*eligible
        torque=np.clip(KP*e+KI*i_state-d_term,-TORQUE_LIMIT,TORQUE_LIMIT)
        roll=(CRR*MASS*9.81*R/2)*np.tanh(omega/.2)
        acceleration=INV@(torque-VISCOUS*omega-roll)
        old_omega=omega.copy()
        omega=omega+DT*acceleration
        omega_prev=old_omega
        v=R*(omega[0]+omega[1])/2
        rows.append([t,vref,v,torque[0],torque[1]])
    return np.array(rows)

if __name__ == '__main__':
    pi=simulate(False); pid=simulate(True)
    assert np.all(np.isfinite(pi)) and np.all(np.isfinite(pid))
    assert np.max(np.abs(pi[:,3:5]))<=TORQUE_LIMIT+1e-10
    assert np.max(np.abs(pid[:,3:5]))<=TORQUE_LIMIT+1e-10
    data=np.column_stack([pi[:,0],pi[:,1],pi[:,2],pid[:,2],pi[:,3],pid[:,3]])
    with (DATA/'python_pi_pid_comparison.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['time_s','v_reference_mps','v_PI_mps','v_PID_mps','tau_R_PI_Nm','tau_R_PID_Nm'])
        writer.writerows(data)
    fig,ax=plt.subplots(figsize=(9,4.2))
    ax.plot(pi[:,0],pi[:,1],linestyle='--',label='Tốc độ đặt')
    ax.plot(pi[:,0],pi[:,2],label='PI (mô hình số)')
    ax.plot(pid[:,0],pid[:,2],label='PID lọc D (mô hình số)')
    ax.set(xlabel='Thời gian (s)', ylabel='Tốc độ robot (m/s)', title='So sánh PI và PID với bộ tham số minh họa')
    ax.grid(True,alpha=.3);ax.legend(loc='best');fig.tight_layout()
    fig.savefig(ASSETS/'python_pi_vs_pid_speed.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4.2))
    ax.plot(pi[:,0],pi[:,3],label='PI: mô-men bánh phải')
    ax.plot(pid[:,0],pid[:,3],label='PID: mô-men bánh phải')
    ax.axhline(TORQUE_LIMIT,linestyle=':',label='Giới hạn mô phỏng')
    ax.axhline(-TORQUE_LIMIT,linestyle=':')
    ax.set(xlabel='Thời gian (s)', ylabel='Mô-men tại bánh (N.m)', title='Điều khiển PI / PID: mô-men giới hạn ±8 N.m')
    ax.grid(True,alpha=.3);ax.legend(loc='best');fig.tight_layout()
    fig.savefig(ASSETS/'python_pi_vs_pid_torque.png',dpi=180);plt.close(fig)
    # Compare on the intervals with changing command, not tuned control performance acceptance.
    active=pi[:,0]>.5
    info={
        'warning':'Illustrative Python cross-check only, NOT MATLAB/Gazebo/hardware result',
        'Kp':KP,'Ki':KI,'Kd':KD,'derivative_filter_seconds':FILTER,
        'torque_limit_Nm_at_wheel_assumed':TORQUE_LIMIT,
        'RMSE_PI_mps':float(np.sqrt(np.mean((pi[active,1]-pi[active,2])**2))),
        'RMSE_PID_mps':float(np.sqrt(np.mean((pid[active,1]-pid[active,2])**2))),
        'max_absolute_PI_minus_PID_speed_mps':float(np.max(np.abs(pi[:,2]-pid[:,2])))
    }
    (DATA/'python_pi_pid_summary.json').write_text(json.dumps(info,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(info,indent=2,ensure_ascii=False))
