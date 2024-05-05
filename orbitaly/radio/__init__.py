"""Radio control: keep a rig tuned through a pass despite doppler.

Orbitaly does not implement CAT protocols itself. It speaks to Hamlib's
``rigctld`` over TCP, which already supports a few hundred radios and can be
faked convincingly in tests.
"""
from .base import Rig, RigError, RigStatus
from .simulated import SimulatedRig
from .tuner import DopplerTuner

__all__ = ["Rig", "RigError", "RigStatus", "SimulatedRig", "DopplerTuner", "make_rig"]


def make_rig(config, *, clock=None) -> Rig | None:
    """Build the rig backend named in the config, or None when disabled."""
    backend = (config.backend or "none").lower()
    if backend in ("none", "", "off"):
        return None
    if backend == "simulated":
        return SimulatedRig(config)
    if backend == "rigctld":
        from .rigctld import RigctldRig

        return RigctldRig(config)
    raise ValueError(f"Unknown rig backend {config.backend!r}: expected none|rigctld|simulated")
