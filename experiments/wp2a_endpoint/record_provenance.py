"""Record wp2a_endpoint provenance after checking summary completeness."""
import hashlib
import json
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def main():
    summary = json.loads((OUT / 'metrics/summary.json').read_text(encoding='utf-8'))
    runs = summary['runs']
    assert len(runs) == 12, 'expect 4 subjects x 3 winds'
    assert all(r['channels_complete'] and r['all_finite'] and r['time_uniform_0p02s']
               and r['progress_monotonic'] and r['seconds'] == 300.0 for r in runs)
    assert all(r['zero_wind_zero_energy_ok'] for r in runs if r['wind'] == 'W0')
    assert all(r['p_env_core_positive'] for r in runs if r['wind'] == 'W1')
    paths = [
        'runtime/fixedwing_att/lib/avanti2_att.dll',
        'python/bindings/avanti_att.py', 'python/controller/controllerAPI.py',
        'python/controller/l1.py', 'python/controller/tecs.py',
        'python/controller/types.py', 'python/simulation/attitude_fleet.py',
        'python/simulation/task_route.py', 'python/tests/test_controller_interfaces.py',
        'config/controller/attitude_runtime.yaml',
        'experiments/wp2a8b/candidate_config.json',
    ]
    paths.extend(str(p.relative_to(ROOT)).replace('\\', '/') for p in OUT.rglob('*')
                 if p.is_file() and p.suffix in {'.json', '.npz', '.png', '.py', '.md', '.txt'}
                 and p.name != 'provenance.json' and '__pycache__' not in p.parts)
    digest = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
              for p in sorted(set(paths))}
    record = dict(
        date='2026-11-04', platform=platform.platform(), python=sys.version,
        executed_library='runtime/fixedwing_att/lib/avanti2_att.dll',
        linux_runtime_executed=False, dt_s=.02, seconds_per_run=300.0,
        runs=12, batch_per_subject=3, workers=1,
        controller_config='config/controller/attitude_runtime.yaml (== wp2a8b candidate_config)',
        native_gains=summary['native_gain_values'],
        path_follower_fix=('PolylinePathFollower._project_local lap-boundary '
                           'deadlock fixed before this collection; regression '
                           'test test_polyline_path_follower_lap_boundary_no_deadlock'),
        assumptions=['deterministic initial state; no randomization',
                     'known wind truth fed to controller; no estimator noise',
                     'wind applied with one-tick transport delay (recorded)',
                     'IAS approximated by TAS*sqrt(ISA density ratio)',
                     'p_env derived from wind truth only; no aero/thrust telemetry',
                     'no surface telemetry or native full-state snapshot'],
        sha256=digest)
    (OUT / 'provenance.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    print('Provenance recorded:', len(digest), 'files; 12 runs all valid')


if __name__ == '__main__':
    main()
