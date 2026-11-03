"""Compare four independent aircraft in serial and persistent-thread execution."""
import json
import sys
import time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from python.simulation.attitude_fleet import AttitudeFleet
from python.controller import HeadingCommand,LoiterCommand
OUT=Path(__file__).resolve().parent

def rollout(workers,cfg):
    data=[];start=time.perf_counter()
    with AttitudeFleet(cfg,batch=4,workers=workers) as fleet:
        for i in range(3):fleet.controller.command_heading(HeadingCommand(16,[0,.03,-.02][i],[-.2,0,.25][i]),[i])
        fleet.controller.command_loiter(LoiterCommand((200,0,100),16,60,direction=-1),[3])
        for _ in range(1500):
            d=fleet.step(np.array([[0,0,0],[0,2,0],[0,-1,0],[0,0,0]]))
            data.append(np.stack([d[k] for k in ['north','east','alt','ias_est','phi','theta','p','q','r']],axis=-1))
    return np.stack(data),time.perf_counter()-start

if __name__=='__main__':
    cfg=json.loads((OUT/'candidate_config.json').read_text())
    serial,ts=rollout(1,cfg);parallel,tp=rollout(4,cfg)
    error=float(np.max(abs(serial-parallel)))
    assert np.isfinite(serial).all() and error<1e-12
    assert np.max(np.ptp(serial[-1,:,:3],axis=0))>1
    result={'instances':4,'seconds':30,'workers':4,'max_abs_difference':error,
        'serial_wall_s':ts,'parallel_wall_s':tp,'speedup':ts/tp,
        'heterogeneous_commands':True,'independent_outer_states':True,'native_gain_writes':0,
        'mechanism':'batched NumPy outer loop + persistent ThreadPoolExecutor native ticks'}
    np.savez_compressed(OUT/'parallel_traces.npz',serial=serial,parallel=parallel)
    (OUT/'parallel_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
