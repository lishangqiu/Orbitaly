"""Rotator hardware abstraction layer."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class RotatorState:
    azimuth: float
    elevation: float
    target_azimuth: float
    target_elevation: float
    moving: bool
    homed: bool
    homing: bool = False
    fault: str = ""


class Rotator(ABC):
    """An az/el positioner. All angles are degrees in the rotator's own frame.

    Azimuth may exceed [0, 360) on rotators with extended travel; callers
    (the tracker) are responsible for mapping sky azimuth into rotator travel.
    """

    @abstractmethod
    def goto(self, azimuth: float, elevation: float) -> None:
        """Command a move. Returns immediately; motion happens in background."""

    @abstractmethod
    def jog(self, d_azimuth: float, d_elevation: float) -> None:
        """Nudge relative to the current *target* position."""

    @abstractmethod
    def stop(self) -> None:
        """Decelerate to a stop and hold position."""

    @abstractmethod
    def home(self) -> None:
        """Run the homing routine (blocking backends may do this async)."""

    @abstractmethod
    def state(self) -> RotatorState:
        """Snapshot of current position and motion status."""

    def close(self) -> None:  # optional cleanup
        pass

    def heartbeat(self) -> None:
        """Called every cycle by whatever is actively steering the antenna.

        Backends with a watchdog use this to notice that the thing driving
        them has died mid-pass and stop rather than sit on a stale target.
        """

    def release(self) -> None:
        """Steering has stopped deliberately — disarm any watchdog."""

    def emergency_stop(self, reason: str = "emergency stop") -> None:
        """Cut motion immediately. Defaults to a controlled stop."""
        self.stop()

    def clear_fault(self) -> bool:
        """Clear a latched fault if its cause is gone. True if now clear."""
        return True

    @property
    @abstractmethod
    def azimuth_travel(self) -> tuple[float, float]:
        """(min, max) commandable azimuth in rotator frame."""

    @property
    @abstractmethod
    def elevation_travel(self) -> tuple[float, float]:
        """(min, max) commandable elevation."""
