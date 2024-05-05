"""The unit of work handed to hardware: a run of steps at one rate."""
from __future__ import annotations

import math
from dataclasses import dataclass

MIN_PERIOD_US = 2  # nothing sane asks a step/dir driver for faster than 500 kHz


@dataclass(frozen=True)
class Segment:
    """``steps`` pulses in ``direction`` at one pulse every ``period_us``.

    ``counts`` is False for backlash take-up: those pulses turn the motor but
    only wind up mechanical slack, so they must not change logical position.
    """

    direction: int  # +1 or -1
    steps: int
    period_us: int
    counts: bool = True

    def __post_init__(self) -> None:
        if self.direction not in (1, -1):
            raise ValueError(f"direction must be +1 or -1, got {self.direction}")
        if self.steps <= 0:
            raise ValueError(f"steps must be positive, got {self.steps}")
        if self.period_us < MIN_PERIOD_US:
            raise ValueError(f"period_us must be >= {MIN_PERIOD_US}, got {self.period_us}")

    @property
    def duration_s(self) -> float:
        return self.steps * self.period_us / 1e6

    @property
    def speed_sps(self) -> float:
        return 1e6 / self.period_us

    @property
    def delta(self) -> int:
        """Signed change in logical position (zero for backlash take-up)."""
        return self.direction * self.steps if self.counts else 0


@dataclass(frozen=True)
class AxisKinematics:
    """Everything the planner needs. Derived from AxisConfig, in step space."""

    max_speed_sps: float
    accel_sps2: float
    #: start/stop rate — the speed a loaded stepper can jump to from rest
    min_speed_sps: float = 8.0
    backlash_steps: int = 0
    segment_max_s: float = 0.02
    #: largest velocity discontinuity allowed between consecutive segments.
    #: A ramp is a staircase, not a curve; this is the height of one stair.
    max_jump_sps: float = 0.0
    #: hard cap on stairs per ramp, so a pathological config cannot explode
    max_ramp_levels: int = 512

    def __post_init__(self) -> None:
        if self.max_speed_sps < self.min_speed_sps:
            raise ValueError("max_speed_sps below min_speed_sps")
        if self.accel_sps2 <= 0:
            raise ValueError("accel_sps2 must be positive")
        if self.max_jump_sps <= 0:
            # One segment's worth of the ideal ramp: by the time hardware has
            # finished a segment the true profile has moved on by this much, so
            # matching it keeps the staircase within a step of the curve.
            object.__setattr__(
                self, "max_jump_sps", max(self.min_speed_sps, self.accel_sps2 * self.segment_max_s)
            )

    def period_us(self, speed_sps: float) -> int:
        speed = min(max(speed_sps, self.min_speed_sps), self.max_speed_sps)
        return max(MIN_PERIOD_US, int(round(1e6 / speed)))

    def stop_steps(self, speed_sps: float) -> int:
        """Steps needed to decelerate from ``speed_sps`` to the crawl speed."""
        if speed_sps <= self.min_speed_sps:
            return 0
        return int(math.ceil((speed_sps**2 - self.min_speed_sps**2) / (2.0 * self.accel_sps2)))
