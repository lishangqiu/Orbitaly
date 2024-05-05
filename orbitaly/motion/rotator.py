"""Two supervised stepper axes presented as a :class:`Rotator`."""
from __future__ import annotations

import logging
import threading

from ..config import RotatorConfig
from ..hardware.base import Rotator, RotatorState
from .axis import StepperAxis
from .backend import Backend

log = logging.getLogger(__name__)


class StepperRotator(Rotator):
    """Az/el rotator driven by two :class:`StepperAxis` supervisors.

    Also owns the two station-wide interlocks that are not per-axis: the
    E-stop input, and a heartbeat watchdog that stops the antenna if the
    tracker that was steering it goes away mid-pass.
    """

    WATCHDOG_TICK_S = 0.25

    def __init__(
        self,
        config: RotatorConfig,
        backend: Backend,
        *,
        clock=None,
        require_homing: bool | None = None,
    ):
        from .clock import RealClock

        self.config = config
        self.backend = backend
        self.clock = clock or RealClock()
        if require_homing is None:
            require_homing = bool(config.require_homing)

        self.az = StepperAxis(
            config.azimuth,
            backend.axis_driver(config.azimuth, "az"),
            name="az",
            clock=self.clock,
            require_homing=require_homing,
            idle_disable_s=config.idle_disable_s,
            realtime_priority=config.realtime_priority,
        )
        self.el = StepperAxis(
            config.elevation,
            backend.axis_driver(config.elevation, "el"),
            name="el",
            clock=self.clock,
            require_homing=require_homing,
            idle_disable_s=config.idle_disable_s,
            realtime_priority=config.realtime_priority,
        )
        self.axes = (self.az, self.el)

        self._hb_lock = threading.Lock()
        self._hb_armed = False
        self._hb_time = self.clock.monotonic()
        self._stop_flag = threading.Event()
        self._watchdog: threading.Thread | None = None

        backend.arm_estop(self._on_estop)

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        for axis in self.axes:
            axis.start()
        if self.config.watchdog_s > 0:
            self._watchdog = threading.Thread(
                target=self._watchdog_run, daemon=True, name="rotator-watchdog"
            )
            self._watchdog.start()

    def close(self) -> None:
        self._stop_flag.set()
        if self._watchdog:
            self._watchdog.join(timeout=2.0)
        for axis in self.axes:
            axis.close()
        self.backend.close()

    # -- Rotator ------------------------------------------------------------

    def goto(self, azimuth: float, elevation: float) -> None:
        self.az.goto(azimuth)
        self.el.goto(elevation)

    def jog(self, d_azimuth: float, d_elevation: float) -> None:
        if d_azimuth:
            self.az.jog(d_azimuth)
        if d_elevation:
            self.el.jog(d_elevation)

    def stop(self) -> None:
        for axis in self.axes:
            axis.stop()

    def home(self) -> None:
        for axis in self.axes:
            axis.home()

    def emergency_stop(self, reason: str = "emergency stop") -> None:
        for axis in self.axes:
            axis.emergency_stop(reason)

    def clear_fault(self) -> bool:
        return all([axis.clear_fault() for axis in self.axes])

    def state(self) -> RotatorState:
        faults = [f"{axis.name}: {axis.fault}" for axis in self.axes if axis.fault]
        if self.backend.estop_triggered():
            faults.insert(0, "E-stop active")
        return RotatorState(
            azimuth=self.az.position_deg,
            elevation=self.el.position_deg,
            target_azimuth=self.az.target_deg,
            target_elevation=self.el.target_deg,
            moving=any(axis.moving for axis in self.axes),
            homed=all(axis.homed for axis in self.axes),
            homing=any(axis.homing for axis in self.axes),
            fault="; ".join(faults),
        )

    @property
    def azimuth_travel(self) -> tuple[float, float]:
        return (self.config.azimuth.min_deg, self.config.azimuth.max_deg)

    @property
    def elevation_travel(self) -> tuple[float, float]:
        return (self.config.elevation.min_deg, self.config.elevation.max_deg)

    # -- interlocks ---------------------------------------------------------

    def heartbeat(self) -> None:
        """Called by whatever is actively steering the antenna, every cycle."""
        with self._hb_lock:
            self._hb_armed = True
            self._hb_time = self.clock.monotonic()

    def release(self) -> None:
        """Steering stopped on purpose — disarm the watchdog."""
        with self._hb_lock:
            self._hb_armed = False

    def _watchdog_run(self) -> None:
        while not self._stop_flag.is_set():
            self._watchdog_once()
            self.clock.sleep(self.WATCHDOG_TICK_S)

    def _watchdog_once(self) -> None:
        with self._hb_lock:
            armed = self._hb_armed
            age = self.clock.monotonic() - self._hb_time
        if armed and age > self.config.watchdog_s:
            log.error("Steering heartbeat lost after %.1fs — stopping rotator", age)
            with self._hb_lock:
                self._hb_armed = False
            self.stop()

    def _on_estop(self) -> None:
        self.emergency_stop("E-stop asserted")

    # -- test seam ----------------------------------------------------------

    def tick(self) -> None:
        """Advance both supervisors once, without threads (used by tests)."""
        for axis in self.axes:
            axis.tick()
