"""A rig that exists only in memory.

Useful for two things: developing the tuning loop without a radio on the
bench, and giving tests something to assert against that behaves like a
transceiver — including the annoying part, tuning resolution.
"""
from __future__ import annotations

from ..config import RigConfig
from .base import Rig, RigStatus


class SimulatedRig(Rig):
    def __init__(self, config: RigConfig | None = None, *, resolution_hz: int = 1):
        self.config = config
        self._open = False
        self.rx_hz = 145_800_000.0
        self.tx_hz = 145_990_000.0
        self.split = False
        self.ptt = False
        self.mode = "USB"
        self.resolution_hz = resolution_hz
        #: every frequency this rig was ever told to use, for assertions
        self.rx_history: list[float] = []
        self.tx_history: list[float] = []

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def connected(self) -> bool:
        return self._open

    def _quantise(self, hz: float) -> float:
        step = max(1, self.resolution_hz)
        return round(hz / step) * step

    def set_rx_freq(self, hz: float) -> None:
        self.rx_hz = self._quantise(hz)
        self.rx_history.append(self.rx_hz)

    def set_tx_freq(self, hz: float) -> None:
        self.tx_hz = self._quantise(hz)
        self.tx_history.append(self.tx_hz)

    def get_rx_freq(self) -> float:
        return self.rx_hz

    def get_tx_freq(self) -> float:
        return self.tx_hz

    def set_split(self, enabled: bool, tx_vfo: str = "VFOB") -> None:
        self.split = enabled

    def get_ptt(self) -> bool:
        return self.ptt

    def get_mode(self) -> str:
        return self.mode

    def status(self) -> RigStatus:
        return RigStatus(
            connected=self._open,
            rx_hz=self.rx_hz,
            tx_hz=self.tx_hz,
            split=self.split,
            ptt=self.ptt,
            mode=self.mode,
        )
