"""Batched outer-loop guidance and two explicitly different inner-loop backends.

FlightController emits AttitudeCommand to the generated attitude aircraft.
LegacySurfaceFlightController / AvantiController retain surface-input tests.
NPFG/TECS math is batched; native aircraft handles are stepped by AttitudeFleet.
Python outer snapshots do not include the generated C++ plant/controller state.
"""

from .types import (  # noqa: F401
    ActuatorCommand,
    AttitudeCommand,
    ControlObservation,
    ControlReference,
)
from .pid import PID  # noqa: F401
from .avanti_controller import AvantiController  # noqa: F401
from .controllerAPI import (  # noqa: F401
    FlightController,
    LegacySurfaceFlightController,
    HeadingCommand,
    LoiterCommand,
    PathFollowerStatus,
    PolylinePathFollower,
)

__all__ = [
    "ActuatorCommand",
    "AttitudeCommand",
    "ControlObservation",
    "ControlReference",
    "PID",
    "AvantiController",
    "FlightController",
    "LegacySurfaceFlightController",
    "LoiterCommand",
    "HeadingCommand",
    "PathFollowerStatus",
    "PolylinePathFollower",
]
