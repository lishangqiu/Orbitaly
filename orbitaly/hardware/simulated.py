"""Simulated rotator: full kinematic model, zero hardware.

The default backend — lets the whole application (tracking, UI, API) run and
be developed on any machine. Honors per-axis max speed and acceleration so
the UI shows realistic slewing behavior.
"""
from __future__ import annotations

import threading
import time

from ..config import RotatorConfig
from .base import Rotator, RotatorState


class _SimAxis:
    def __init__(self, min_deg: float, max_deg: float, max_speed: float, accel: float, start: float):
        self.min_deg, self.max_deg = min_deg, max_deg
        self.max_speed, self.accel = max_speed, accel
        self.position = start
        self.velocity = 0.0
        self.target = start

    def tick(self, dt: float) -> None:
        error = self.target - self.position
        if abs(error) < 1e-4 and abs(self.velocity) < 1e-3:
            self.position, self.velocity = self.target, 0.0
            return
        stop_dist = self.velocity**2 / (2.0 * self.accel)
        direction = 1.0 if error > 0 else -1.0
        if self.velocity * direction < 0 or abs(error) <= stop_dist:
            accel = -self.accel if self.velocity > 0 else self.accel
        else:
            accel = direction * self.accel
        self.velocity = max(-self.max_speed, min(self.max_speed, self.velocity + accel * dt))
        self.position += self.velocity * dt
        # Snap when overshooting near the target at low speed
        if (self.target - self.position) * direction < 0 and abs(self.velocity) <= self.accel * dt * 2:
            self.position, self.velocity = self.target, 0.0

    @property
    def moving(self) -> bool:
        return abs(self.target - self.position) > 1e-3 or abs(self.velocity) > 1e-3


class SimulatedRotator(Rotator):
    TICK_S = 0.05

    def __init__(self, config: RotatorConfig):
        self.config = config
        az, el = config.azimuth, config.elevation
        self._az = _SimAxis(az.min_deg, az.max_deg, az.max_speed_dps, az.accel_dps2, az.home_position_deg)
        self._el = _SimAxis(el.min_deg, el.max_deg, el.max_speed_dps, el.accel_dps2, el.home_position_deg)
        self._homed = True
        self._lock = threading.Lock()
        self._stop_flag = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="sim-rotator")
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            with self._lock:
                self._az.tick(self.TICK_S)
                self._el.tick(self.TICK_S)
            time.sleep(self.TICK_S)

    def goto(self, azimuth: float, elevation: float) -> None:
        with self._lock:
            self._az.target = min(max(azimuth, self._az.min_deg), self._az.max_deg)
            self._el.target = min(max(elevation, self._el.min_deg), self._el.max_deg)

    def jog(self, d_azimuth: float, d_elevation: float) -> None:
        with self._lock:
            az_t, el_t = self._az.target, self._el.target
        self.goto(az_t + d_azimuth, el_t + d_elevation)

    def stop(self) -> None:
        with self._lock:
            for axis in (self._az, self._el):
                stop_dist = axis.velocity**2 / (2.0 * axis.accel)
                axis.target = axis.position + (stop_dist if axis.velocity > 0 else -stop_dist)

    def home(self) -> None:
        self.goto(self.config.azimuth.home_position_deg, self.config.elevation.home_position_deg)
        self._homed = True

    def state(self) -> RotatorState:
        with self._lock:
            return RotatorState(
                azimuth=self._az.position,
                elevation=self._el.position,
                target_azimuth=self._az.target,
                target_elevation=self._el.target,
                moving=self._az.moving or self._el.moving,
                homed=self._homed,
            )

    def close(self) -> None:
        self._stop_flag.set()
        self._thread.join(timeout=1.0)

    @property
    def azimuth_travel(self) -> tuple[float, float]:
        return (self._az.min_deg, self._az.max_deg)

    @property
    def elevation_travel(self) -> tuple[float, float]:
        return (self._el.min_deg, self._el.max_deg)
