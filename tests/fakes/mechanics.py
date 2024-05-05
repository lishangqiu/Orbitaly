"""A rotator axis that can actually misbehave.

A test double that always does what it is told proves nothing. This one has
mechanical slack, a limit switch at a real angle, a hard stop past it, and a
stall speed — so a test can prove the planner never asks for motion the
mechanism cannot deliver, and that the supervisor notices when it does.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orbitaly.motion.segment import Segment


@dataclass
class VirtualAxis:
    """One geared axis with a switch on it.

    Angles are in the rotator's own frame — the same frame the supervisor
    commands in — so a test can compare commanded and physical degrees
    directly.
    """

    steps_per_deg: float
    position_deg: float = 0.0
    #: switch is closed at or below this angle (None = no switch fitted)
    endstop_deg: float | None = None
    #: mechanical hard stops; motion past them is lost, not travelled
    hard_min_deg: float | None = None
    hard_max_deg: float | None = None
    backlash_deg: float = 0.0
    #: pulse rate above which the motor stalls instead of stepping
    stall_speed_sps: float = float("inf")

    steps_lost: int = 0
    stalls: int = 0
    pulses: int = 0
    trace: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._position_steps = int(round(self.position_deg * self.steps_per_deg))
        self._engaged_dir = 0
        self._slack_remaining = 0
        self.trace.append(self.position_deg)

    # -- Mechanics protocol -------------------------------------------------

    def apply(self, segment: Segment) -> None:
        self.pulses += segment.steps

        if segment.speed_sps > self.stall_speed_sps:
            # Commanded faster than the motor can follow: it buzzes and stays put.
            self.stalls += 1
            self.steps_lost += segment.steps
            self.trace.append(self.position_deg)
            return

        remaining = segment.steps
        if self._engaged_dir == 0:
            self._engaged_dir = segment.direction
        elif segment.direction != self._engaged_dir:
            self._slack_remaining = self._backlash_steps
            self._engaged_dir = segment.direction

        taken = min(remaining, self._slack_remaining)
        self._slack_remaining -= taken
        remaining -= taken

        target = self._position_steps + segment.direction * remaining
        clamped = self._clamp(target)
        if clamped != target:
            self.steps_lost += abs(target - clamped)
        self._position_steps = clamped
        self.position_deg = self._position_steps / self.steps_per_deg
        self.trace.append(self.position_deg)

    def has_endstop(self) -> bool:
        return self.endstop_deg is not None

    def endstop_triggered(self) -> bool:
        return self.endstop_deg is not None and self.position_deg <= self.endstop_deg + 1e-9

    # -- internals ----------------------------------------------------------

    @property
    def _backlash_steps(self) -> int:
        return int(round(self.backlash_deg * self.steps_per_deg))

    def _clamp(self, steps: int) -> int:
        if self.hard_min_deg is not None:
            steps = max(steps, int(round(self.hard_min_deg * self.steps_per_deg)))
        if self.hard_max_deg is not None:
            steps = min(steps, int(round(self.hard_max_deg * self.steps_per_deg)))
        return steps
