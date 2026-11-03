"""Generated attitude aircraft: preserved GA seed and adapted outer-loop trials."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from python.simulation.attitude_fleet import AttitudeFleet
from python.controller import HeadingCommand,LoiterCommand,PolylinePathFollower

OUT=Path(__file__).resolve().parent

def ga_config():
    return {'outer':json.loads((ROOT/'experiments/wp2a8/tuning_04_multitask_ga/result.json').read_text())['outer_cfg']}

def figure8():
    a=np.linspace(0,2*np.pi,100,endpoint=False)
    b=np.linspace(np.pi,-np.pi,100,endpoint=False)
    return np.column_stack([np.r_[120+80*np.cos(a),280+80*np.cos(b)],
                            np.r_[80*np.sin(a),80*np.sin(b)],np.full(200,100.)])

def simulate(case,cfg,seconds,workers=1):
    rows=[]; path=None; follower=None
    if case=='figure8':path=figure8()
    if case.startswith('loiter'):
        a=np.linspace(0,2*np.pi,721)
        path=np.column_stack([200+60*np.cos(a),60*np.sin(a),np.full(721,100.)])
    started=time.perf_counter()
    with AttitudeFleet(cfg,workers=workers) as fleet:
        fc=fleet.controller
        if case.startswith('loiter'):fc.command_loiter(LoiterCommand((200.,0.,100.),16.,60.))
        else:fc.command_heading(HeadingCommand(16.,0.,0.))
        if case=='figure8':
            follower=PolylinePathFollower(path,16,lookahead_min_m=35,lookahead_max_m=55)
            follower.reset(initial_chi=0)
        previous_segment=-1
        for k in range(round(seconds/.02)):
            now=fleet.time
            if case=='heading':
                segment=np.searchsorted([80,140,240,300],now,side='right')
                if segment!=previous_segment:
                    fc.command_heading(HeadingCommand(16.,np.deg2rad([0,3,0,-3,0][segment]),
                        np.deg2rad([0,0,30,30,0][segment]),timestamp=now))
                    previous_segment=segment
            if follower:
                obs=fleet.observation()
                fc.command_heading(follower.update(obs.neu_pos[0],now,np.linalg.norm(obs.neu_vel[0,:2])))
            d=fleet.step(wind_flat=(0,3,0) if case=='loiter_wind' else None)
            row={key:float(value[0]) for key,value in d.items()}
            if follower:
                status=follower.status()
                row.update(path_cross_track=status.signed_cross_track_m,path_progress=status.progress_m,
                    path_lap_count=status.lap_count,path_lookahead=status.lookahead_m)
            rows.append(row)
        gains=fleet.gains
    log={key:np.array([r[key] for r in rows]) for key in rows[0]}
    return log,path,gains,time.perf_counter()-started

def metrics(case,d,elapsed):
    t=d['t']; valid=t>30
    m={'case':case,'seconds':len(t)*.02,'wall_seconds':elapsed,
        'finite_aircraft_output':bool(all(np.isfinite(d[k]).all() for k in ['alt','ias_est','phi','theta','p','q','r'])),
        'ias_rmse_mps':float(np.sqrt(np.mean((d['ias_est'][valid]-d['ias_cmd'][valid])**2))),
        'alt_rmse_m':float(np.sqrt(np.mean((d['alt'][valid]-d['altitude_cmd'][valid])**2))),
        'roll_mae_deg':float(np.mean(abs(d['phi'][valid]-d['phi_cmd'][valid]))*180/np.pi),
        'pitch_mae_deg':float(np.mean(abs(d['theta'][valid]-d['theta_cmd'][valid]))*180/np.pi),
        'pitch_command_step_rms_deg':float(np.sqrt(np.mean(np.diff(d['theta_cmd'][valid])**2))*180/np.pi),
        'throttle_limit_fraction':float(np.mean((d['throttle_cmd'][valid]<=.001)|(d['throttle_cmd'][valid]>=.999))),
        'max_bank_deg':float(max(abs(d['phi']))*180/np.pi),
        'max_beta_deg':float(max(abs(d['beta'][valid]))*180/np.pi),
        'surface_saturation':None,'surface_telemetry_available':False}
    if case.startswith('loiter'):
        held=np.flatnonzero(d['loiter_phase']==2)
        m['first_hold_time_s']=float(t[held[0]]) if len(held) else None
        m['entered_circle_at_m']=float(d['radial_m'][np.flatnonzero(d['loiter_phase']>0)[0]]) if np.any(d['loiter_phase']>0) else None
        steady=t>max(60.,float(t[held[0]])+20 if len(held) else 60.)
        if np.any(steady):
            err=abs(d['cross_track'][steady]);m.update(radial_mae_m=float(np.mean(err)),radial_p95_m=float(np.percentile(err,95)))
    if case=='heading':
        changes=np.r_[0,np.flatnonzero(np.diff(d['gamma_cmd'])!=0)+1,np.flatnonzero(np.diff(d['chi_ground_cmd'])!=0)+1]
        steady=valid.copy()
        for i in changes:steady&=~((t>=t[i])&(t<t[i]+10))
        e=(d['chi']-d['chi_ground_cmd']+np.pi)%(2*np.pi)-np.pi
        m['course_rmse_deg']=float(np.sqrt(np.mean(e[steady]**2))*180/np.pi)
        m['gamma_rmse_deg']=float(np.sqrt(np.mean((d['gamma'][steady]-d['gamma_cmd'][steady])**2))*180/np.pi)
    if case=='figure8':
        valid=t>60
        m.update(path_cross_mae_m=float(np.mean(abs(d['path_cross_track'][valid]))),
            path_cross_p95_m=float(np.percentile(abs(d['path_cross_track'][valid]),95)),
            completed_laps=int(d['path_lap_count'][-1]),progress_monotonic=bool(np.all(np.diff(d['path_progress'])>=-1e-9)))
    return m

def plot(case,d,path,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,3,figsize=(16,11),constrained_layout=True)
    a=axes.ravel();t=d['t'];deg=180/np.pi
    a[0].plot(d['east'],d['north'],label='actual')
    if path is not None:a[0].plot(path[:,1],path[:,0],'k--',lw=.8,label='reference')
    a[0].set(title='Ground trajectory',xlabel='East (m)',ylabel='North (m)');a[0].axis('equal')
    a[1].plot(t,d['alt'],label='actual');a[1].plot(t,d['altitude_cmd'],'k--',label='trajectory reference');a[1].set(title='Altitude',ylabel='m')
    a[2].plot(t,d['ias_est'],label='estimated IAS');a[2].plot(t,d['ias_cmd'],'k--',label='IAS command');a[2].set(title='Airspeed',ylabel='m/s')
    for ax,key,title in [(a[3],'phi','Roll'),(a[4],'theta','Pitch'),(a[5],'gamma','Ground flight-path inclination')]:
        ax.plot(t,d[key]*deg,label='actual');ax.plot(t,d[key+'_cmd']*deg,'k--',lw=.8,label='command');ax.set(title=title,ylabel='deg')
    a[6].plot(t,np.unwrap(d['chi'])*deg,label='actual ground course');a[6].plot(t,np.unwrap(d['chi_ground_cmd'])*deg,'k--',lw=.8,label='guidance reference');a[6].set(title='Ground course',ylabel='deg')
    a[7].plot(t,d['throttle_cmd'],label='throttle');a[7].set(title='Throttle (surfaces not exposed by ABI)',ylim=(-.05,1.05))
    for key in ['p','q','r']:a[8].plot(t,d[key]*deg,lw=.7,label=key)
    a[8].set(title='Body angular rates',ylabel='deg/s')
    for ax in a:ax.grid(alpha=.25);ax.legend(fontsize=7)
    for ax in a[1:]:ax.set_xlabel('Time (s)')
    fig.suptitle(f'WP2A-8b | {case} | generated attitude loop + Python NPFG/TECS | dt=0.02s')
    fig.savefig(out,dpi=140,bbox_inches='tight');plt.close(fig)

def main():
    p=argparse.ArgumentParser();p.add_argument('--profile',choices=['ga','adapted'],default='adapted')
    p.add_argument('--seconds',type=float,default=400);p.add_argument('--trial',choices=['all','heading','loiter','loiter_wind','figure8'],default='all')
    args=p.parse_args();out=OUT/('ga_baseline' if args.profile=='ga' else 'quality_trials');out.mkdir(exist_ok=True)
    cfg=ga_config() if args.profile=='ga' else json.loads((OUT/'candidate_config.json').read_text())
    (out/'config.json').write_text(json.dumps(cfg,indent=2))
    selected=['heading','loiter','loiter_wind','figure8'] if args.trial=='all' else [args.trial]
    summaries=[]
    for case in selected:
        print('RUNNING',args.profile,case,args.seconds,flush=True)
        d,path,gains,elapsed=simulate(case,cfg,args.seconds)
        np.savez_compressed(out/(case+'.npz'),**d)
        plot(case,d,path,out/(case+'.png'))
        m=metrics(case,d,elapsed);summaries.append(m)
        (out/(case+'_metrics.json')).write_text(json.dumps(m,indent=2));print(json.dumps(m),flush=True)
    (out/'summary.json').write_text(json.dumps({'profile':args.profile,'library_gain_values':gains,'trials':summaries},indent=2))

if __name__=='__main__':main()
