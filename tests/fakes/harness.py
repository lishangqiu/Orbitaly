"""Deterministic pump for the motion stack.

No threads and no wall-clock sleeps: time only moves when the test says so, so
a twelve-minute pass runs in milliseconds and fails the same way every time.
"""
from __future__ import annotations

from orbitaly.config import AxisConfig, RotatorConfig
from orbitaly.motion.axis import StepperAxis
from orbitaly.motion.backend import SimulatedBackend
from orbitaly.motion.clock import VirtualClock
from orbitaly.motion.rotator import StepperRotator
from orbitaly.motion.sim_driver import SimulatedAxisDriver

from .mechanics import VirtualAxis

TICK_S = 0.002


class AxisHarness:
    """One supervised axis over a virtual mechanism, pumped by hand."""

    def __init__(
        self,
        config: AxisConfig,
        mechanics: VirtualAxis | None = None,
        *,
        require_homing: bool = False,
        idle_disable_s: float = 0.0,
    ):
        self.clock = VirtualClock()
        self.mechanics = mechanics
        self.driver = SimulatedAxisDriver(
            mechanics=mechanics, dir_setup_s=config.dir_setup_us / 1e6, clock=self.clock
        )
        self.axis = StepperAxis(
            config,
            self.driver,
            name="test",
            clock=self.clock,
            require_homing=require_homing,
            idle_disable_s=idle_disable_s,
        )
        self.axis._set_enabled(True)

    def run(self, seconds: float, tick: float = TICK_S) -> None:
        steps = max(1, int(seconds / tick))
        for _ in range(steps):
            self.clock.advance(tick)
            self.axis.tick()

    def run_until(self, predicate, timeout_s: float = 120.0, tick: float = TICK_S) -> bool:
        elapsed = 0.0
        while elapsed < timeout_s:
            self.clock.advance(tick)
            self.axis.tick()
            elapsed += tick
            if predicate():
                return True
        return False

    def settle(self, timeout_s: float = 120.0) -> bool:
        """Run until the axis is at rest with nothing queued."""
        return self.run_until(lambda: not self.axis.moving, timeout_s)


class RotatorHarness:
    """Both axes, as the application sees them."""

    def __init__(
        self,
        config: RotatorConfig,
        mechanics: dict[str, VirtualAxis] | None = None,
        *,
        require_homing: bool = False,
    ):
        self.clock = VirtualClock()
        self.mechanics = mechanics or {}
        self.backend = SimulatedBackend(mechanics=self.mechanics, clock=self.clock)
        self.rotator = StepperRotator(
            config, self.backend, clock=self.clock, require_homing=require_homing
        )
        for axis in self.rotator.axes:
            axis._set_enabled(True)

    def run(self, seconds: float, tick: float = TICK_S) -> None:
        steps = max(1, int(seconds / tick))
        for _ in range(steps):
            self.clock.advance(tick)
            self.rotator.tick()

    def run_until(self, predicate, timeout_s: float = 300.0, tick: float = TICK_S) -> bool:
        elapsed = 0.0
        while elapsed < timeout_s:
            self.clock.advance(tick)
            self.rotator.tick()
            elapsed += tick
            if predicate():
                return True
        return False

    def settle(self, timeout_s: float = 300.0) -> bool:
        return self.run_until(lambda: not self.rotator.state().moving, timeout_s)
