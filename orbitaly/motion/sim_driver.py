"""Simulated axis backend.

Executes segments as pure time bookkeeping — no pin toggling, no per-step
work — and optionally pushes them through a mechanical model so tests can
watch a real rotator misbehave (backlash, endstops, hard stops, lost steps).

This is the default backend, which means developing without hardware still
exercises the same planner, the same supervisor and the same safety logic
that run on the Pi.
"""
from __future__ import annotations

from typing import Callable, Protocol

from .driver import BufferedAxisDriver
from .segment import Segment


class Mechanics(Protocol):
    """A physical axis the simulated driver drives."""

    def apply(self, segment: Segment) -> None: ...

    def has_endstop(self) -> bool: ...

    def endstop_triggered(self) -> bool: ...


class NullMechanics:
    """No mechanism attached: pulses vanish into the void, no endstop."""

    def apply(self, segment: Segment) -> None:
        pass

    def has_endstop(self) -> bool:
        return False

    def endstop_triggered(self) -> bool:
        return False


class SimulatedAxisDriver(BufferedAxisDriver):
    def __init__(self, *, mechanics: Mechanics | None = None, dir_setup_s: float = 20e-6, clock=None):
        super().__init__(dir_setup_s=dir_setup_s, clock=clock)
        self.mechanics: Mechanics = mechanics or NullMechanics()
        self.enabled = False
        self.direction_forward = True
        self.pulses = 0
        self._endstop_callback: Callable[[], None] | None = None
        self._endstop_latched = False

    # -- backend primitives -------------------------------------------------

    def _hand_off(self, segment: Segment) -> None:
        self.pulses += segment.steps

    def _set_dir(self, forward: bool) -> None:
        self.direction_forward = forward

    def _stop_pulses(self) -> None:
        pass

    def _on_complete(self, segment: Segment) -> None:
        # Mechanics see a segment when the motor has actually turned through
        # it, not when it was queued — so endstop timing stays honest.
        self.mechanics.apply(segment)
        self._poll_endstop()

    # -- endstop ------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def has_endstop(self) -> bool:
        return self.mechanics.has_endstop()

    def endstop_triggered(self) -> bool:
        return self.mechanics.endstop_triggered()

    def arm_endstop(self, callback: Callable[[], None] | None) -> None:
        self._endstop_callback = callback
        self._endstop_latched = False

    def _poll_endstop(self) -> None:
        triggered = self.mechanics.endstop_triggered()
        if triggered and not self._endstop_latched:
            self._endstop_latched = True
            if self._endstop_callback is not None:
                self._endstop_callback()
        elif not triggered:
            self._endstop_latched = False
