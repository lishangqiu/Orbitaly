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


class SerialAxisHarness:
    """One supervised axis whose pulses come out of a (fake) Arduino.

    Same shape as :class:`AxisHarness`, and deliberately so: every behavioural
    test that holds for the simulated driver should hold here, because the
    supervisor above the driver is unchanged. The differences are all below the
    `AxisDriver` line — real framing, flow control, and completion reports
    instead of clock arithmetic.
    """

    def __init__(
        self,
        config: AxisConfig,
        mechanics: VirtualAxis | None = None,
        *,
        require_homing: bool = False,
        idle_disable_s: float = 0.0,
        wrap_transport=None,
    ):
        from orbitaly.config import RotatorConfig as _RotatorConfig
        from orbitaly.motion.serial_driver import SerialBackend

        from .fake_firmware import FakeFirmware, FirmwareTransport

        self.clock = VirtualClock()
        self.mechanics = mechanics
        self.firmware = FakeFirmware(mechanics={"az": mechanics}, clock=self.clock)
        transport = FirmwareTransport(self.firmware)
        self.transport = wrap_transport(transport) if wrap_transport else transport
        self.backend = SerialBackend(
            _RotatorConfig(backend="serial", azimuth=config, elevation=config),
            transport=self.transport,
            clock=self.clock,
        )
        self.driver = self.backend.axis_driver(config, "az")
        self.axis = StepperAxis(
            config,
            self.driver,
            name="test",
            clock=self.clock,
            require_homing=require_homing,
            idle_disable_s=idle_disable_s,
        )
        # StepperRotator wires these two for the real application; a bare axis
        # has to do it itself or a backend-level fault (unplugged cable, board
        # reset) would have nowhere to land and the test would be measuring the
        # harness rather than the code.
        self.backend.arm_estop(lambda: self.axis.emergency_stop("E-stop asserted"))
        self.backend.arm_fault(self.axis.emergency_stop)
        self.axis._set_enabled(True)

    def run(self, seconds: float, tick: float = TICK_S) -> None:
        steps = max(1, int(seconds / tick))
        for _ in range(steps):
            self.clock.advance(tick)
            self.firmware.advance(self.clock.monotonic())
            self.axis.tick()

    def run_until(self, predicate, timeout_s: float = 120.0, tick: float = TICK_S) -> bool:
        elapsed = 0.0
        while elapsed < timeout_s:
            self.clock.advance(tick)
            self.firmware.advance(self.clock.monotonic())
            self.axis.tick()
            elapsed += tick
            if predicate():
                return True
        return False

    def settle(self, timeout_s: float = 120.0) -> bool:
        return self.run_until(lambda: not self.axis.moving, timeout_s)


class SerialRotatorHarness:
    """Both axes over one fake Arduino — what the application sees on the Pi."""

    def __init__(
        self,
        config: RotatorConfig,
        mechanics: dict[str, VirtualAxis] | None = None,
        *,
        require_homing: bool = False,
        wrap_transport=None,
    ):
        from orbitaly.motion.serial_driver import SerialBackend

        from .fake_firmware import FakeFirmware, FirmwareTransport

        self.clock = VirtualClock()
        self.mechanics = mechanics or {}
        self.firmware = FakeFirmware(mechanics=self.mechanics, clock=self.clock)
        transport = FirmwareTransport(self.firmware)
        self.transport = wrap_transport(transport) if wrap_transport else transport
        self.backend = SerialBackend(config, transport=self.transport, clock=self.clock)
        self.rotator = StepperRotator(
            config, self.backend, clock=self.clock, require_homing=require_homing
        )
        for axis in self.rotator.axes:
            axis._set_enabled(True)

    def run(self, seconds: float, tick: float = TICK_S) -> None:
        steps = max(1, int(seconds / tick))
        for _ in range(steps):
            self.clock.advance(tick)
            self.firmware.advance(self.clock.monotonic())
            self.rotator.tick()

    def run_until(self, predicate, timeout_s: float = 300.0, tick: float = TICK_S) -> bool:
        elapsed = 0.0
        while elapsed < timeout_s:
            self.clock.advance(tick)
            self.firmware.advance(self.clock.monotonic())
            self.rotator.tick()
            elapsed += tick
            if predicate():
                return True
        return False

    def settle(self, timeout_s: float = 300.0) -> bool:
        return self.run_until(lambda: not self.rotator.state().moving, timeout_s)
