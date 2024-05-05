"""Tracking engine: drives the rotator to follow a satellite.

State machine: IDLE -> ACQUIRING (pre-position at AOS azimuth) -> TRACKING
(follow at update_rate) -> back to ACQUIRING/IDLE at LOS. MANUAL suspends
tracking for jog/goto commands from the UI.

Azimuth unwrapping: sky azimuth wraps 359->0 on north-crossing passes. The
tracker converts each pass to a *continuous* azimuth profile and picks a
360-degree offset so the whole pass fits inside the rotator's travel
(e.g. -90..450), avoiding a mid-pass 360-degree slew whenever the hardware
allows it.
"""
from __future__ import annotations

import logging
import threading
import time
from enum import Enum

from ..config import TrackerConfig
from ..hardware.base import Rotator
from ..motion.errors import MotionBlocked
from .predictor import Pass, Predictor
from .tle import TleManager

log = logging.getLogger(__name__)


class TrackerState(str, Enum):
    IDLE = "idle"
    ACQUIRING = "acquiring"
    TRACKING = "tracking"
    MANUAL = "manual"


def unwrap_azimuths(azimuths: list[float]) -> list[float]:
    """Remove 360-degree wraps so the sequence is continuous."""
    if not azimuths:
        return []
    result = [azimuths[0]]
    for az in azimuths[1:]:
        prev = result[-1]
        delta = (az - prev + 180.0) % 360.0 - 180.0
        result.append(prev + delta)
    return result


def map_into_travel(azimuth: float, current: float, az_min: float, az_max: float) -> float:
    """Pick the representation of a sky azimuth that is nearest where we are.

    An extended-travel rotator can point at 10 degrees as 10, 370 or -350. Any
    of them is correct; the one to command is whichever is inside the travel
    limits and closest to the current position, so an external client sending
    plain 0..360 azimuths never provokes a needless full-circle slew.
    """
    candidates = [
        azimuth + 360.0 * k
        for k in (-2, -1, 0, 1, 2)
        if az_min - 1e-9 <= azimuth + 360.0 * k <= az_max + 1e-9
    ]
    if not candidates:
        return min(max(azimuth, az_min), az_max)
    return min(candidates, key=lambda c: abs(c - current))


def choose_az_offset(lo: float, hi: float, az_min: float, az_max: float) -> float | None:
    """Pick k*360 so [lo, hi] + offset fits in [az_min, az_max], else None."""
    for k in (0, -1, 1, -2, 2):
        offset = 360.0 * k
        if lo + offset >= az_min - 1e-9 and hi + offset <= az_max + 1e-9:
            return offset
    return None


class Tracker:
    def __init__(
        self,
        config: TrackerConfig,
        predictor: Predictor,
        tle_manager: TleManager,
        rotator: Rotator,
        time_fn=time.time,
    ):
        self.config = config
        self.predictor = predictor
        self.tle_manager = tle_manager
        self.rotator = rotator
        self._now = time_fn

        self._lock = threading.Lock()
        self._state = TrackerState.IDLE
        self._norad_id: int | None = None
        self._current_pass: Pass | None = None
        self._az_offset = 0.0
        self._prev_continuous_az: float | None = None
        self._wrap_warning = ""
        self._blocked = ""
        self._stop_flag = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="tracker")
        self._thread.start()

    def close(self) -> None:
        self._stop_flag.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # -- commands -----------------------------------------------------------

    def track(self, norad_id: int) -> None:
        if self.tle_manager.get(norad_id) is None:
            raise KeyError(f"Unknown satellite {norad_id}")
        with self._lock:
            self._norad_id = norad_id
            self._state = TrackerState.ACQUIRING
            self._current_pass = None
            self._prev_continuous_az = None
            self._wrap_warning = ""
        log.info("Tracking engaged for NORAD %d", norad_id)

    def stop_tracking(self) -> None:
        with self._lock:
            self._state = TrackerState.IDLE
            self._norad_id = None
            self._current_pass = None
        self.rotator.release()

    def manual_goto(self, azimuth: float, elevation: float) -> None:
        with self._lock:
            self._state = TrackerState.MANUAL
            self._norad_id = None
        self.rotator.release()
        self.rotator.goto(azimuth, elevation)

    def manual_jog(self, d_az: float, d_el: float) -> None:
        with self._lock:
            self._state = TrackerState.MANUAL
            self._norad_id = None
        self.rotator.release()
        self.rotator.jog(d_az, d_el)

    def manual_stop(self) -> None:
        with self._lock:
            if self._state != TrackerState.MANUAL:
                self._state = TrackerState.IDLE
                self._norad_id = None
        self.rotator.release()
        self.rotator.stop()

    def park(self) -> None:
        with self._lock:
            self._state = TrackerState.IDLE
            self._norad_id = None
        self.rotator.release()
        self.rotator.goto(self.config.park.az, self.config.park.el)

    # -- introspection ------------------------------------------------------

    @property
    def state(self) -> TrackerState:
        return self._state

    @property
    def tracked_norad_id(self) -> int | None:
        return self._norad_id

    @property
    def current_pass(self) -> Pass | None:
        return self._current_pass

    @property
    def wrap_warning(self) -> str:
        return self._wrap_warning

    @property
    def blocked(self) -> str:
        """Why the rotator is refusing commands, if it is."""
        return self._blocked

    # -- loop ---------------------------------------------------------------

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                self._tick()
            except Exception:
                log.exception("Tracker tick failed")
            self._stop_flag.wait(self.config.update_rate_s)

    def _tick(self) -> None:
        with self._lock:
            state, norad_id = self._state, self._norad_id
        if state not in (TrackerState.ACQUIRING, TrackerState.TRACKING) or norad_id is None:
            return
        sat = self.tle_manager.get(norad_id)
        if sat is None:
            self.stop_tracking()
            return

        now = self._now()
        # Tell the rotator we are still here. If this stops arriving mid-pass
        # the rotator stops rather than carry on toward a stale target.
        self.rotator.heartbeat()
        self._ensure_pass(sat, now)
        obs = self.predictor.observe(sat, now)

        if obs.elevation >= 0.0:
            with self._lock:
                self._state = TrackerState.TRACKING
            self._command(obs.azimuth, max(obs.elevation, 0.0))
        else:
            with self._lock:
                if self._state == TrackerState.TRACKING:
                    # LOS just happened: plan for the next pass
                    self._current_pass = None
                    self._prev_continuous_az = None
                self._state = TrackerState.ACQUIRING
            if self._current_pass is not None:
                aos_az = self._current_pass.profile[0].azimuth if self._current_pass.profile else self._current_pass.aos_azimuth
                self._command(aos_az, 0.0)

    def _ensure_pass(self, sat, now: float) -> None:
        """Keep a current/next pass planned, with its azimuth offset chosen."""
        p = self._current_pass
        if p is not None and p.los > now:
            return
        passes = self.predictor.next_passes(
            sat, now, hours=self.config.pass_lookahead_hours,
            min_elevation=0.0,
        )
        self._current_pass = passes[0] if passes else None
        self._prev_continuous_az = None
        self._plan_azimuth()

    def _plan_azimuth(self) -> None:
        self._az_offset = 0.0
        self._wrap_warning = ""
        p = self._current_pass
        if p is None or not p.profile:
            return
        continuous = unwrap_azimuths([point.azimuth for point in p.profile])
        az_min, az_max = self.rotator.azimuth_travel
        offset = choose_az_offset(min(continuous), max(continuous), az_min, az_max)
        if offset is None:
            self._wrap_warning = (
                "Pass does not fit rotator azimuth travel; a mid-pass wrap slew may occur"
            )
            # Fall back: at least start inside travel
            offset = choose_az_offset(continuous[0], continuous[0], az_min, az_max) or 0.0
        self._az_offset = offset
        # Seed continuity from the profile start so live azimuths unwrap the same way
        self._prev_continuous_az = continuous[0]

    def _command(self, sky_azimuth: float, elevation: float) -> None:
        try:
            self._command_unchecked(sky_azimuth, elevation)
        except MotionBlocked as exc:
            # An interlock said no — usually "not homed yet". Report it once in
            # the status feed instead of logging a traceback every second.
            if self._blocked != str(exc):
                log.warning("Tracking blocked: %s", exc)
            self._blocked = str(exc)
            return
        self._blocked = ""

    def _command_unchecked(self, sky_azimuth: float, elevation: float) -> None:
        continuous = self._continuous(sky_azimuth)
        commanded = continuous + self._az_offset
        az_min, az_max = self.rotator.azimuth_travel
        if commanded < az_min or commanded > az_max:
            # Out of planned travel (fallback path): re-fit this single angle
            refit = choose_az_offset(continuous, continuous, az_min, az_max)
            if refit is not None:
                self._az_offset = refit
                commanded = continuous + refit
        self.rotator.goto(commanded, elevation)

    def _continuous(self, azimuth: float) -> float:
        prev = self._prev_continuous_az
        if prev is None:
            self._prev_continuous_az = azimuth
            return azimuth
        delta = (azimuth - prev + 180.0) % 360.0 - 180.0
        value = prev + delta
        self._prev_continuous_az = value
        return value
