"""Whole-stack passes: predictor to tracker to planner to hardware to gearbox.

Everything except the pulse hardware and the mechanism is the real code. A ten
minute pass runs in a fraction of a second on a virtual clock, so these can ask
awkward questions — what is the worst pointing error at zenith, does the
antenna ever unwind 360 degrees mid-pass, does the motor ever lose a step.
"""
import math

import pytest

from orbitaly.config import AxisConfig, PinConfig, RotatorConfig, StationConfig, TrackerConfig
from orbitaly.core.predictor import Predictor
from orbitaly.core.tle import Satellite
from orbitaly.core.tracker import Tracker, TrackerState
from orbitaly.motion.backend import SimulatedBackend
from orbitaly.motion.clock import VirtualClock
from orbitaly.motion.rotator import StepperRotator

from fakes.mechanics import VirtualAxis

ISS = Satellite(
    25544,
    "ISS (ZARYA)",
    "1 25544U 98067A   20045.18587073  .00000950  00000-0  25302-4 0  9990",
    "2 25544  51.6443 242.0161 0004885 264.6060 207.3845 15.49165514212791",
)
STATION = StationConfig(latitude=40.44, longitude=-79.99, altitude_m=300)

# Passes picked from this TLE, by shape rather than by luck:
MODERATE_PASS_AOS = 1581686587  # 20.6 deg peak, no north crossing
NORTH_CROSSING_AOS = 1581692357  # 45.3 deg peak, azimuth wraps 247 -> 50
OVERHEAD_PASS_AOS = 1581775896  # 75.1 deg peak: the azimuth rate stress case


class FakeTle:
    def __init__(self, sat):
        self.satellites = {sat.norad_id: sat}

    def get(self, norad_id):
        return self.satellites.get(norad_id)


def rotator_config(**axis_overrides) -> RotatorConfig:
    """The reference hardware from config.example.yaml."""
    az = AxisConfig(
        min_deg=-90.0,
        max_deg=450.0,
        max_speed_dps=6.0,
        accel_dps2=4.0,
        gear_ratio=60.0,
        pins=PinConfig(step=17, dir=27, enable=22, endstop=-1),
    )
    el = AxisConfig(
        min_deg=0.0,
        max_deg=180.0,
        max_speed_dps=4.0,
        accel_dps2=4.0,
        gear_ratio=40.0,
        pins=PinConfig(step=23, dir=24, enable=25, endstop=-1),
    )
    for key, value in axis_overrides.items():
        setattr(az, key, value)
        setattr(el, key, value)
    return RotatorConfig(backend="simulated", azimuth=az, elevation=el, watchdog_s=0.0)


def separation_deg(az1: float, el1: float, az2: float, el2: float) -> float:
    """True angular separation on the sky — what a beam actually cares about."""
    a1, e1, a2, e2 = map(math.radians, (az1, el1, az2, el2))
    cos_sep = math.sin(e1) * math.sin(e2) + math.cos(e1) * math.cos(e2) * math.cos(a1 - a2)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_sep))))


class PassRun:
    """Runs one pass and records what the antenna actually did."""

    def __init__(
        self,
        aos: float,
        config: RotatorConfig | None = None,
        lead_s: float = 120.0,
        backend_factory=None,
    ):
        config = config or rotator_config()
        self.clock = VirtualClock()
        self.mechanics = {
            "az": VirtualAxis(steps_per_deg=config.azimuth.steps_per_deg),
            "el": VirtualAxis(steps_per_deg=config.elevation.steps_per_deg),
        }
        # `backend_factory` exists so the identical pass can be flown against a
        # different pulse source — see test_serial_passes.py, which re-runs this
        # accuracy table through the Arduino backend. Returns (backend, hook),
        # where the hook runs immediately before each motion tick so a backend
        # with its own hardware model can be stepped in lockstep.
        if backend_factory is None:
            backend = SimulatedBackend(mechanics=self.mechanics, clock=self.clock)
            self.before_motion_tick = lambda: None
        else:
            backend, self.before_motion_tick = backend_factory(config, self.mechanics, self.clock)
        self.backend = backend
        self.rotator = StepperRotator(config, backend, clock=self.clock, require_homing=False)
        for axis in self.rotator.axes:
            axis._set_enabled(True)

        self.predictor = Predictor(STATION)
        self.sim_time = aos - lead_s
        self.tracker = Tracker(
            TrackerConfig(update_rate_s=1.0),
            self.predictor,
            FakeTle(ISS),
            self.rotator,
            time_fn=lambda: self.sim_time,
        )
        self.tracker.track(ISS.norad_id)
        self.samples: list[dict] = []

    def run(self, seconds: float, motion_tick: float = 0.01) -> None:
        slices = int(1.0 / motion_tick)
        for _ in range(int(seconds)):
            self.tracker._tick()
            for _ in range(slices):
                self.clock.advance(motion_tick)
                self.before_motion_tick()
                self.rotator.tick()
            self.sim_time += 1.0
            self._sample()

    def _sample(self) -> None:
        obs = self.predictor.observe(ISS, self.sim_time)
        az_mech = self.mechanics["az"].position_deg
        el_mech = self.mechanics["el"].position_deg
        self.samples.append(
            {
                "t": self.sim_time,
                "sat_az": obs.azimuth,
                "sat_el": obs.elevation,
                "az": az_mech,
                "el": el_mech,
                "error": separation_deg(az_mech % 360.0, el_mech, obs.azimuth, obs.elevation),
                "state": self.tracker.state,
            }
        )

    # -- queries ------------------------------------------------------------

    def during_pass(self, min_el: float = 5.0) -> list[dict]:
        return [s for s in self.samples if s["sat_el"] >= min_el]

    def worst_error(self, min_el: float = 5.0) -> float:
        tracking = [s for s in self.during_pass(min_el) if s["state"] is TrackerState.TRACKING]
        assert tracking, "no tracking samples — the run never engaged the tracker"
        return max(s["error"] for s in tracking)

    def azimuth_travelled(self) -> float:
        azs = [s["az"] for s in self.during_pass(0.0)]
        return sum(abs(b - a) for a, b in zip(azs, azs[1:]))


# -- accuracy ---------------------------------------------------------------


def test_a_normal_pass_is_tracked_to_well_inside_a_beamwidth():
    run = PassRun(MODERATE_PASS_AOS)
    run.run(120 + 601 + 30)
    # Measured worst case is ~0.4 deg, which is dominated by the 1 Hz update
    # rate rather than by anything in the motion stack.
    assert run.worst_error() < 0.6
    assert run.mechanics["az"].steps_lost == 0
    assert run.mechanics["el"].steps_lost == 0
    assert run.mechanics["az"].stalls == 0


def test_the_antenna_is_pre_positioned_before_the_satellite_rises():
    run = PassRun(MODERATE_PASS_AOS, lead_s=180.0)
    run.run(180)
    first_pass_sample = next(s for s in run.samples if s["sat_el"] > 0)
    assert first_pass_sample["error"] < 5.0, "should already be looking at the AOS azimuth"


def test_a_steep_pass_is_still_tracked_accurately():
    """75 degrees elevation is where azimuth rate blows up: it peaks near
    5 deg/s here, just inside what a 6 deg/s azimuth drive can follow."""
    run = PassRun(OVERHEAD_PASS_AOS)
    run.run(120 + 657 + 30)
    assert run.worst_error() < 2.0
    assert run.rotator.state().fault == ""
    assert run.mechanics["az"].steps_lost == 0


def test_a_rotator_too_slow_for_the_pass_lags_and_then_recovers():
    """Halve the azimuth speed and the mount genuinely cannot keep up at TCA.

    The point is what happens next: the error must peak and come back, with no
    lost steps and no fault. A mount that cannot follow is a hardware fact; a
    mount that desyncs its position because of it is a software bug.
    """
    config = rotator_config()
    config.azimuth.max_speed_dps = 2.0
    run = PassRun(OVERHEAD_PASS_AOS, config=config)
    run.run(120 + 657 + 30)
    peak = run.worst_error()
    assert peak > 3.0, "expected this mount to fall behind — otherwise the test proves nothing"
    late = run.during_pass(8.0)[-45:]
    assert max(s["error"] for s in late) < peak / 2.0, "should have caught back up"
    assert run.mechanics["az"].steps_lost == 0
    assert run.rotator.state().fault == ""


# -- azimuth unwrapping -----------------------------------------------------


def test_a_north_crossing_pass_never_unwinds_the_cable():
    """247 -> 50 degrees the short way is straight through north.

    With -90..450 of travel the planner should map the whole pass into one
    continuous sweep instead of slewing the long way round mid-pass.
    """
    run = PassRun(NORTH_CROSSING_AOS)
    run.run(120 + 651 + 30)
    travelled = run.azimuth_travelled()
    assert travelled < 200.0, f"antenna swept {travelled:.0f} deg — that is a wrap slew"
    assert run.tracker.wrap_warning == ""
    assert run.worst_error() < 1.0


def test_a_rotator_without_extended_travel_is_told_it_cannot_fit_the_pass():
    """A plain 0..360 rotator physically cannot do a north crossing cleanly.

    The right behaviour is to say so, not to pretend.
    """
    config = rotator_config()
    config.azimuth.min_deg = 0.0
    config.azimuth.max_deg = 360.0
    run = PassRun(NORTH_CROSSING_AOS, config=config)
    run.run(120 + 300)
    assert "does not fit" in run.tracker.wrap_warning


# -- limits -----------------------------------------------------------------


def test_the_antenna_never_leaves_its_travel_limits():
    for aos in (MODERATE_PASS_AOS, NORTH_CROSSING_AOS, OVERHEAD_PASS_AOS):
        run = PassRun(aos)
        run.run(120 + 400)
        az_min, az_max = run.rotator.azimuth_travel
        el_min, el_max = run.rotator.elevation_travel
        for sample in run.samples:
            assert az_min - 0.1 <= sample["az"] <= az_max + 0.1
            assert el_min - 0.1 <= sample["el"] <= el_max + 0.1


def test_elevation_never_dips_below_the_horizon_stop():
    run = PassRun(MODERATE_PASS_AOS)
    run.run(120 + 601 + 60)
    assert min(s["el"] for s in run.samples) >= -0.01


# -- watchdog ---------------------------------------------------------------


def test_the_rotator_stops_if_the_tracker_stops_heartbeating():
    config = rotator_config()
    config.watchdog_s = 3.0
    run = PassRun(MODERATE_PASS_AOS, config=config)
    run.run(130)
    run.rotator.goto(300.0, 40.0)  # a long slew, then the steering vanishes
    run.rotator.heartbeat()

    # No more heartbeats: the watchdog should intervene rather than let the
    # antenna carry on toward a target nobody is updating.
    for _ in range(600):
        run.clock.advance(0.01)
        run.rotator.tick()
        run.rotator._watchdog_once()
    assert not run.rotator.state().moving
    assert run.rotator.az.position_deg < 300.0
