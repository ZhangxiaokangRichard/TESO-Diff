"""Action timing, mixed-batch state, capture latch and reference continuity."""
import copy
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from python.controller import FlightController,HeadingCommand,LoiterCommand,AttitudeCommand
from python.tests.test_controller_interfaces import _observation

def obs(batch=1):
    o=_observation(batch);o.neu_pos[:,2]=100;o.neu_vel[:,0]=16
    return o

def test_action_timing_reference_snapshot():
    c=FlightController();c.command_heading(HeadingCommand(16,0,0))
    c.command_heading(HeadingCommand(16,.05,1,timestamp=.04))
    o=obs();assert isinstance(c.step(o,0),AttitudeCommand)
    assert c.intermediates()['gamma_cmd'][0]==0
    o.neu_pos[:,2]=110;c.step(o,.02)
    assert c.intermediates()['altitude_cmd'][0]==100
    snap=c.get_snapshot();expected=c.step(o,.04);diag=c.intermediates()
    c.set_snapshot(snap);actual=c.step(o,.04)
    np.testing.assert_array_equal(actual.theta,expected.theta)
    assert c.intermediates()['gamma_cmd'][0]==.05
    assert diag['altitude_cmd'][0]==100
    c.step(o,.06)
    np.testing.assert_allclose(c.intermediates()['altitude_cmd'],100+.02*16*np.tan(.05))
    # A held path-follower action keeps its original timestamp and is safe to resend.
    c.command_heading(HeadingCommand(16,.05,1,timestamp=.04))
    try:c.step(o,.10)
    except ValueError:pass
    else:raise AssertionError('skipped control tick accepted')

def test_loiter_capture_and_reverse_heading():
    c=FlightController();c.command_loiter(LoiterCommand((200,0,100),16,60));o=obs()
    c.step(o,0);assert c.intermediates()['loiter_phase'][0]==c.APPROACH
    o.neu_pos[:,0]=90;c.step(o,.02);assert c.intermediates()['loiter_phase'][0]>c.APPROACH
    o.neu_pos[:,0]=0;c.step(o,.04);assert c.intermediates()['loiter_phase'][0]>c.APPROACH
    c.reset();c.command_heading(HeadingCommand(16,0,np.pi));cmd=c.step(obs())
    assert abs(cmd.phi[0])>0,'180 degree course change must produce a turn'
    c.reset();c.command_loiter(LoiterCommand((200,0,100),16,1));c.step(obs())
    assert c.intermediates()['radius_m'][0]>30

def test_mixed_batch_matches_independent():
    commands=[HeadingCommand(16,.03,.2),LoiterCommand((200,0,100),15,60),HeadingCommand(17,0,-.3)]
    batched=FlightController();batched.reset(3)
    singles=[FlightController() for _ in commands]
    for i,command in enumerate(commands):
        method='command_loiter' if isinstance(command,LoiterCommand) else 'command_heading'
        getattr(batched,method)(command,indices=[i]);getattr(singles[i],method)(command)
    for k in range(15):
        o=obs(3);o.neu_pos[:,0]=k*2
        group=batched.step(o,k*.02)
        for i,c in enumerate(singles):
            individual=obs();individual.neu_pos[:,0]=k*2
            single=c.step(individual,k*.02)
            for name in ['phi','theta','throttle']:
                np.testing.assert_allclose(getattr(group,name)[i],getattr(single,name)[0],rtol=0,atol=1e-12)
    assert batched.intermediates()['mode'].tolist()==[batched.HEADING,batched.LOITER,batched.HEADING]

if __name__=='__main__':
    test_action_timing_reference_snapshot()
    test_loiter_capture_and_reverse_heading()
    test_mixed_batch_matches_independent()
    print('Generated attitude action API tests passed')
