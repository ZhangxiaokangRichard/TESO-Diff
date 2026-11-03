"""Record current trial provenance after checking config and duration."""
import hashlib
import json
import platform
import sys
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent


def main():
    selected=json.loads((OUT/'candidate_config.json').read_text())
    active=yaml.safe_load((ROOT/'config/controller/attitude_runtime.yaml').read_text(encoding='utf-8'))
    assert active['outer']==selected['outer']
    assert json.loads((OUT/'quality_trials/config.json').read_text())==selected
    summary=json.loads((OUT/'quality_trials/summary.json').read_text())
    assert len(summary['trials'])==4 and all(t['seconds']==400 and t['finite_aircraft_output'] for t in summary['trials'])
    paths=[
        'runtime/fixedwing_att/lib/avanti2_att.dll',
        'runtime/fixedwing_att/generated/PX4Controller_ertmodel0827.cpp',
        'runtime/fixedwing_att/generated/PX4Controller_ertmodel0827_data.cpp',
        'runtime/fixedwing_att/wrapper/avanti_att_c_api.cpp',
        'python/bindings/avanti_att.py','python/controller/controllerAPI.py',
        'python/controller/l1.py','python/controller/tecs.py','python/controller/types.py',
        'python/simulation/attitude_fleet.py','config/controller/attitude_runtime.yaml',
        'experiments/wp2a8/tuning_04_multitask_ga/result.json',
    ]
    paths.extend(str(p.relative_to(ROOT)).replace('\\','/') for p in OUT.rglob('*')
                 if p.is_file() and p.suffix in {'.json','.npz','.png','.py','.md'}
                 and p.name!='provenance.json')
    digest={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(set(paths))}
    record=dict(date='2026-11-03',platform=platform.platform(),python=sys.version,
        executed_library='runtime/fixedwing_att/lib/avanti2_att.dll',
        linux_runtime_executed=False,dt_s=.02,trial_seconds=400,official_ticks_per_trial=20000,
        bootstrap_ticks_per_instance=1,controller_observation_delay_ticks=1,
        native_gain_writes=0,native_gains=summary['library_gain_values'],
        assumptions=['deterministic initial state','known wind; no estimator noise',
                     'IAS approximated by TAS*sqrt(ISA density ratio)',
                     'no surface telemetry or native full-state snapshot'],sha256=digest)
    (OUT/'provenance.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print('Provenance recorded:',len(digest),'files; active and trial configurations match')


if __name__=='__main__':main()
