"""Software stepper-motor control: one axis, direct step/dir pulse generation.

This is the piece that replaces gcode/motion-controller firmware: Orbitaly
itself generates step pulses with trapezoidal acceleration ramps, enforces
soft travel limits, performs homing against an endstop, and compensates
backlash. Pulses go through a pluggable :class:`PulseDriver`, so the same
logic runs against real GPIO pins or a test double.

Antenna rotators move slowly (a few deg/s) through large gear reductions, so
software-timed pulses from Python are adequate; a pigpio waveform driver can
be slotted in for high-microstep setups without touching this module.
"""
from __future__ import annotations

import math
import threading
import time
from abc import ABC, abstractmethod

from ..config import AxisConfig


class PulseDriver(ABC):
    """Minimal electrical interface to one stepper driver (step/dir/enable)."""

    @abstractmethod
    def set_direction(self, forward: bool) -> None: ...

    @abstractmethod
    def pulse(self) -> None:
        """Emit one step pulse (leading + trailing edge)."""

    def set_enabled(self, enabled: bool) -> None:  # optional
        pass

    def endstop_triggered(self) -> bool:  # optional; no endstop = never triggered
        return False

    def close(self) -> None:
        pass


class StepperAxis:
    """Position-controlled axis with a trapezoidal velocity profile.

    Runs its own thread. ``goto()`` retargets at any time, including
    mid-move; the loop decelerates and reverses as needed.
    """

    MIN_SPEED_SPS = 8.0  # floor so step delays stay bounded

    def __init__(
        self,
        config: AxisConfig,
        driver: PulseDriver,
        name: str = "axis",
        sleep_fn=time.sleep,
    ):
        self.config = config
        self.driver = driver
        self.name = name
        self._sleep = sleep_fn
        self._steps_per_deg = config.steps_per_deg
        self._max_speed_sps = max(config.max_speed_dps * self._steps_per_deg, self.MIN_SPEED_SPS)
        self._accel_sps2 = max(config.accel_dps2 * self._steps_per_deg, 1.0)
        self._backlash_steps = int(round(config.backlash_deg * self._steps_per_deg))

        self._lock = threading.Lock()
        self._position_steps = int(round(config.home_position_deg * self._steps_per_deg))
        self._target_steps = self._position_steps
        self._velocity_sps = 0.0  # magnitude; direction tracked separately
        self._direction = 1
        self._last_motion_dir = 0
        self._homed = False
        self._homing = False
        self._fault = ""
        self._wake = threading.Event()
        self._stop_flag = threading.Event()
        self._thread: threading.Thread | None = None

    # -- public API ---------------------------------------------------------

    def start(self) -> None:
        self.driver.set_enabled(True)
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"stepper-{self.name}")
        self._thread.start()

    def close(self) -> None:
        self._stop_flag.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.driver.set_enabled(False)
        self.driver.close()

    @property
    def position_deg(self) -> float:
        return self._position_steps / self._steps_per_deg

    @property
    def target_deg(self) -> float:
        return self._target_steps / self._steps_per_deg

    @property
    def moving(self) -> bool:
        return self._position_steps != self._target_steps or self._homing

    @property
    def homed(self) -> bool:
        return self._homed

    @property
    def fault(self) -> str:
        return self._fault

    def goto(self, degrees: float) -> None:
        clamped = min(max(degrees, self.config.min_deg), self.config.max_deg)
        with self._lock:
            self._target_steps = int(round(clamped * self._steps_per_deg))
        self._wake.set()

    def stop(self) -> None:
        """Retarget to wherever the current ramp can stop."""
        with self._lock:
            stop_steps = int(self._velocity_sps**2 / (2.0 * self._accel_sps2)) + 1
            self._target_steps = self._position_steps + self._direction * stop_steps
            self._homing = False

    def home(self) -> None:
        self._homing = True
        self._wake.set()

    # -- motion loop --------------------------------------------------------

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            if self._homing:
                self._do_home()
                continue
            with self._lock:
                delta = self._target_steps - self._position_steps
            if delta == 0:
                self._velocity_sps = 0.0
                self._wake.wait(0.05)
                self._wake.clear()
                continue
            self._step_once(delta)

    def _step_once(self, delta: int) -> None:
        direction = 1 if delta > 0 else -1
        if direction != self._direction and self._velocity_sps > self.MIN_SPEED_SPS:
            # Moving the wrong way: decelerate before reversing.
            self._velocity_sps = self._ramp_down(self._velocity_sps)
            self._emit_step(self._direction)
            return
        if direction != self._direction:
            self._direction = direction
            self.driver.set_direction((direction > 0) != self.config.invert_dir)
            self._take_up_backlash(direction)

        remaining = abs(delta)
        stop_steps = self._velocity_sps**2 / (2.0 * self._accel_sps2)
        if remaining <= stop_steps:
            self._velocity_sps = self._ramp_down(self._velocity_sps)
        else:
            self._velocity_sps = min(
                math.sqrt(self._velocity_sps**2 + 2.0 * self._accel_sps2),
                self._max_speed_sps,
            )
        self._emit_step(direction)

    def _ramp_down(self, velocity: float) -> float:
        return max(
            math.sqrt(max(velocity**2 - 2.0 * self._accel_sps2, 0.0)),
            self.MIN_SPEED_SPS,
        )

    def _emit_step(self, direction: int) -> None:
        self.driver.pulse()
        with self._lock:
            self._position_steps += direction
        self._last_motion_dir = direction
        self._sleep(1.0 / max(self._velocity_sps, self.MIN_SPEED_SPS))

    def _take_up_backlash(self, direction: int) -> None:
        """Extra pulses on reversal that move the mechanism, not the count."""
        if self._backlash_steps and self._last_motion_dir and direction != self._last_motion_dir:
            period = 1.0 / self.MIN_SPEED_SPS
            for _ in range(self._backlash_steps):
                self.driver.pulse()
                self._sleep(period)

    def _do_home(self) -> None:
        """Drive toward min travel until the endstop closes, then zero there."""
        if not self._has_endstop():
            with self._lock:
                self._position_steps = int(round(self.config.home_position_deg * self._steps_per_deg))
                self._target_steps = self._position_steps
            self._homed = True
            self._homing = False
            return
        self.driver.set_direction((False) != self.config.invert_dir)  # toward min
        self._direction = -1
        travel_steps = int((self.config.max_deg - self.config.min_deg) * self._steps_per_deg) + 100
        period = 1.0 / max(self._max_speed_sps * 0.25, self.MIN_SPEED_SPS)
        for _ in range(travel_steps):
            if self._stop_flag.is_set() or not self._homing:
                return
            if self.driver.endstop_triggered():
                with self._lock:
                    self._position_steps = int(round(self.config.min_deg * self._steps_per_deg))
                    self._target_steps = self._position_steps
                self._homed = True
                self._homing = False
                self._velocity_sps = 0.0
                return
            self.driver.pulse()
            with self._lock:
                self._position_steps -= 1
            self._sleep(period)
        self._fault = f"{self.name}: homing failed, endstop never triggered"
        self._homing = False

    def _has_endstop(self) -> bool:
        return self.config.pins.endstop >= 0
