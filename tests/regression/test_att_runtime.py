"""ABI/time/reset/shared-gain/dual-runtime checks; no full path-control claim."""
from __future__ import annotations
import argparse
import ctypes as ct
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'python'))
from bindings.avanti_att import AvantiAttInput, AvantiAttLibrary, AvantiAttParams
from bindings.avanti import AvantiInput, AvantiLibrary

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--library',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--skip-old-runtime',action='store_true')
    args=parser.parse_args()
    lib=AvantiAttLibrary(args.library)
    original=lib.get_params()
    try:
        defaults=lib.get_default_params()
        assert all(math.isfinite(x) for x in defaults.as_dict().values())
        # Compare old plant ticks before/after the new library's simulations.
        try: old=None if args.skip_old_runtime else AvantiLibrary()
        except FileNotFoundError: old=None
        def old_trace():
            with old.model() as m:
                return [bytes(m.tick(AvantiInput(throttle_01=.5))) for _ in range(100)]
        reference=old_trace() if old is not None else None
        with lib.model(100) as a,lib.model(100) as b:
            assert a.fixed_step_s==.02
            initial=None
            for k in range(1000):
                t=k*.02
                phi=math.radians(10)*max(0,min((t-1)/2,1)) if t<9 else 0
                u=AvantiAttInput(phi_sp_rad=phi,theta_sp_rad=math.radians(3),throttle_01=.7)
                y=a.tick(u); z=b.tick(u)
                assert y.finite() and bytes(y)==bytes(z)
                assert abs(y.sample_time_s-(100+t))<1e-10
                if initial is None: initial={"position":list(y.position_ne_alt_m),"euler_deg":list(y.euler_rpy_deg),"body_velocity":list(y.velocity_body_mps)}
            assert abs(a.next_tick_time_s-120)<1e-10
            before=a.next_tick_time_s
            bad=AvantiAttInput(phi_sp_rad=float('nan'))
            assert lib.lib.avanti_att_tick(a.handle,before,ct.byref(bad),ct.byref(y))==3
            assert a.next_tick_time_s==before
            assert lib.lib.avanti_att_tick(a.handle,119,ct.byref(u),ct.byref(y))==5
            # Atomic rejection of a nonfinite gain set.
            bad_params=AvantiAttParams.from_dict(original.as_dict())
            bad_params.omega_sp_i_phi=float('nan')
            assert lib.lib.avanti_att_set_params(ct.byref(bad_params))==3
            assert lib.get_params().as_dict()==original.as_dict()
            a.reset(200); b.reset(200)
            a0=a.tick(u); b0=b.tick(u); assert bytes(a0)==bytes(b0)
            # Runtime-wide gains affect *both existing* model handles.
            changed=AvantiAttParams.from_dict(original.as_dict()); changed.p_phi_sp*=.5
            lib.set_params(changed)
            assert lib.get_default_params().as_dict()==defaults.as_dict()
            a.reset(300); b.reset(300)
            cmd=AvantiAttInput(phi_sp_rad=.15,throttle_01=.7)
            changed_trace=[bytes(a.tick(cmd)) for _ in range(30)]
            assert changed_trace==[bytes(b.tick(cmd)) for _ in range(30)]
            lib.set_params(original); a.reset(300)
            assert changed_trace!=[bytes(a.tick(cmd)) for _ in range(30)]
            a.reset(400)
            y=a.advance_to(cmd,400.04,3)
            assert y.sample_time_s==400.04 and abs(a.next_tick_time_s-400.06)<1e-10
        if old is not None:
            assert reference==old_trace(),'New runtime changed old runtime state/symbols'
        report={'library':str(lib.path),'ticks':1000,'two_instance_determinism':True,
                'time_reset_advance':True,'invalid_inputs_atomic':True,
                'process_shared_params':True,'old_runtime_coexistence':old is not None,
                'compiled_default_params':defaults.as_dict(),'initial_state':initial}
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
    finally: lib.set_params(original)

if __name__=='__main__': main()
