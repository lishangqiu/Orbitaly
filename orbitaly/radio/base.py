"""Rig abstraction: the handful of operations doppler tuning actually needs."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


class RigError(RuntimeError):
    """The radio (or the daemon in front of it) refused or went away."""


@dataclass
class RigStatus:
    connected: bool = False
    error: str = ""
    rx_hz: float = 0.0
    tx_hz: float = 0.0
    split: bool = False
    ptt: bool = False
    mode: str = ""


class Rig(ABC):
    """A transceiver.

    Satellite work is full duplex: receive on one frequency while transmitting
    on another, in a different band. Hamlib models that as *split* — the RX
    frequency on the current VFO and the TX frequency as the split frequency —
    and so does this interface.
    """

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def connected(self) -> bool: ...

    @abstractmethod
    def set_rx_freq(self, hz: float) -> None: ...

    @abstractmethod
    def set_tx_freq(self, hz: float) -> None: ...

    @abstractmethod
    def get_rx_freq(self) -> float: ...

    @abstractmethod
    def get_tx_freq(self) -> float: ...

    def set_split(self, enabled: bool, tx_vfo: str = "VFOB") -> None:
        pass

    def get_ptt(self) -> bool:
        return False

    def get_mode(self) -> str:
        return ""

    def status(self) -> RigStatus:
        return RigStatus(connected=self.connected)
