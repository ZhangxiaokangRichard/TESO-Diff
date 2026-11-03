"""Batched outer loop and persistent, optional parallel native aircraft handles."""
from concurrent.futures import ThreadPoolExecutor
import numpy as np

from ..bindings.avanti_att import AvantiAttInput, AvantiAttLibrary
from ..controller.controllerAPI import FlightController, density_ratio
from ..controller.types import ControlObservation


class AttitudeFleet:
    """One native handle per environment; shared, frozen gains per process.

    Python NPFG/TECS computation is batched. Native ticks can run in threads
    (ctypes releases the GIL), using the same executor throughout a rollout.
    This is CPU parallelism, independent of Isaac/GPU availability.
    """
    def __init__(self,cfg=None,batch=1,workers=1,library=None,epoch_s=0.):
        if not isinstance(workers,int) or workers<1:raise ValueError('workers must be a positive integer')
        self.library=library or AvantiAttLibrary()
        self.controller=FlightController(cfg)
        self.controller.reset(batch)
        self.batch=batch
        self.models=[]
        self.executor=ThreadPoolExecutor(max_workers=min(batch,workers)) if workers>1 else None
        try:
            self.models=[self.library.model(epoch_s) for _ in range(batch)]
            assert all(np.isclose(m.fixed_step_s,.02) for m in self.models)
            initial=AvantiAttInput(phi_sp_rad=0,theta_sp_rad=np.deg2rad(.39280840518578974),throttle_01=.7)
            self.outputs=[m.tick(initial) for m in self.models]
        except BaseException:
            self.close();raise
        self._last_wind=np.zeros((batch,3))
        self.gains=self.library.get_params().as_dict()

    @property
    def time(self):
        return self.models[0].next_tick_time_s

    def observation(self):
        obs=ControlObservation.from_fw_outputs(self.outputs)
        wind_neu=self._last_wind.copy();wind_neu[:,2]*=-1
        obs.tas=np.linalg.norm(obs.neu_vel-wind_neu,axis=1)
        obs.wind_neu=wind_neu[:,:2]
        return obs

    @staticmethod
    def _tick(pair):
        model,inputs=pair
        output=model.tick(inputs)
        if not output.finite():raise RuntimeError('native attitude aircraft returned NaN/Inf')
        return output

    def step(self,wind_flat=None):
        wind=np.zeros((self.batch,3)) if wind_flat is None else np.broadcast_to(wind_flat,(self.batch,3)).copy()
        if not np.isfinite(wind).all():raise ValueError('wind must be finite')
        now=self.time
        command=self.controller.step(self.observation(),timestamp=now)
        inputs=command.to_inputs(wind)
        pairs=list(zip(self.models,inputs))
        self.outputs=list(self.executor.map(self._tick,pairs)) if self.executor else list(map(self._tick,pairs))
        self._last_wind=wind
        obs=self.observation()
        diag=self.controller.intermediates()
        record=dict(diag)
        record.update(t=np.array([y.sample_time_s for y in self.outputs]),
            north=obs.neu_pos[:,0],east=obs.neu_pos[:,1],alt=obs.neu_pos[:,2],
            vn=obs.neu_vel[:,0],ve=obs.neu_vel[:,1],vu=obs.neu_vel[:,2],
            phi=obs.phi,theta=obs.theta,psi=obs.psi,p=obs.p,q=obs.q,r=obs.r,
            tas=obs.tas,ias_est=obs.tas*np.sqrt(density_ratio(obs.neu_pos[:,2])),
            alpha=obs.alpha,beta=obs.beta,
            chi=np.arctan2(obs.neu_vel[:,1],obs.neu_vel[:,0]),
            gamma=np.arctan2(obs.neu_vel[:,2],np.linalg.norm(obs.neu_vel[:,:2],axis=1)))
        return record

    def close(self):
        if self.executor:
            self.executor.shutdown(wait=True);self.executor=None
        for model in self.models:model.close()
        self.models=[]

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
