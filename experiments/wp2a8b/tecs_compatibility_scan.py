"""Reproduce the bounded 12-case straight-flight compatibility screen.

Short-window winners need long-sequence validation. Historical results are
preserved; a rerun writes tecs_compatibility_rescan.json.
"""
import copy
import itertools
import json
import numpy as np
from quality_trials import OUT, ga_config, simulate


def main():
    results=[]
    for index,(trim,balance,throttle) in enumerate(itertools.product([-2,0,2],[.3,.6],[.06,.15])):
        cfg=copy.deepcopy(ga_config())
        cfg['outer']['tecs'].update(theta_trim_deg=trim,balance_kp=balance,
            balance_ki=.05,throttle_kp=throttle,throttle_ki=.04,energy_kp=.3,
            airspeed_kp=.4,airspeed_ki=.03,height_kp=.15,bank_comp_gain=.1)
        d,_,_,_=simulate('heading',cfg,45)
        mask=d['t']>20
        alt=float(np.sqrt(np.mean((d['alt'][mask]-d['altitude_cmd'][mask])**2)))
        ias=float(np.sqrt(np.mean((d['ias_est'][mask]-d['ias_cmd'][mask])**2)))
        std=float(np.std(np.rad2deg(d['theta'][mask])))
        step=float(np.sqrt(np.mean(np.diff(np.rad2deg(d['theta_cmd'][mask]))**2)))
        m=dict(index=index,trim=trim,balance_kp=balance,throttle_kp=throttle,
            alt_rmse=alt,ias_rmse=ias,pitch_std_deg=std,cmd_step_deg=step,
            cost=alt+3*ias+.5*std+step)
        results.append(dict(metrics=m,config=cfg))
        print(json.dumps(m),flush=True)
    results.sort(key=lambda row:row['metrics']['cost'])
    output=OUT/'tecs_compatibility_rescan.json'
    output.write_text(json.dumps(dict(budget=12,seconds_per_trial=45,results=results),indent=2),encoding='utf-8')
    print(output)


if __name__=='__main__':main()
