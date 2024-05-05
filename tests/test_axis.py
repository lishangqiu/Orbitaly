"""Supervisor behaviour against a virtual mechanism, on a virtual clock.

These run the real planner, the real supervisor and the real safety logic —
only the pulse hardware and the gearbox are simulated.
"""
import pytest

from orbitaly.config import AxisConfig, PinConfig
from orbitaly.motion.errors import MotionBlocked

from fakes.harness import AxisHarness
from fakes.mechanics import VirtualAxis

STEPS_PER_DEG = 200 * 1 * 10 / 360.0  # 5.5556


def make_config(**overrides) -> AxisConfig:
    config = AxisConfig(
        min_deg=0.0,
        max_deg=360.0,
        max_speed_dps=90.0,
        accel_dps2=180.0,
        steps_per_rev=200,
        microsteps=1,
        gear_ratio=10.0,
        pins=PinConfig(step=17, dir=27, enable=22, endstop=-1),
    )
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


def make_mechanics(**overrides) -> VirtualAxis:
    params = {"steps_per_deg": STEPS_PER_DEG}
    params.update(overrides)
    return VirtualAxis(**params)


# -- basic motion -----------------------------------------------------------


def test_goto_lands_on_target_to_the_step():
    harness = AxisHarness(make_config(), make_mechanics())
    harness.axis.goto(90.0)
    assert harness.settle()
    assert harness.axis.position_deg == pytest.approx(90.0, abs=1 / STEPS_PER_DEG)


def test_the_mechanism_ends_up_where_the_axis_thinks_it_is():
    mechanics = make_mechanics()
    harness = AxisHarness(make_config(), mechanics)
    harness.axis.goto(120.0)
    assert harness.settle()
    assert mechanics.steps_lost == 0
    assert mechanics.stalls == 0
    assert mechanics.position_deg == pytest.approx(harness.axis.position_deg, abs=1e-9)


def test_goto_clamps_to_soft_limits():
    harness = AxisHarness(make_config(), make_mechanics())
    harness.axis.goto(9999.0)
    assert harness.axis.target_deg == pytest.approx(360.0)
    harness.axis.goto(-45.0)
    assert harness.axis.target_deg == pytest.approx(0.0)


def test_retarget_mid_move_ends_on_the_new_target():
    mechanics = make_mechanics()
    harness = AxisHarness(make_config(), mechanics)
    harness.axis.goto(180.0)
    assert harness.run_until(lambda: harness.axis.position_deg > 30.0)
    harness.axis.goto(10.0)
    assert harness.settle()
    assert harness.axis.position_deg == pytest.approx(10.0, abs=1 / STEPS_PER_DEG)
    assert mechanics.steps_lost == 0


def test_retarget_never_needs_an_abort_so_position_stays_exact():
    """Replanning happens from the end of what hardware already has.

    That is the whole reason tracking can retarget every second without
    accumulating error: no in-flight pulses are ever thrown away.
    """
    mechanics = make_mechanics()
    harness = AxisHarness(make_config(), mechanics)
    for target in (40.0, 55.0, 20.0, 60.0, 35.0):
        harness.axis.goto(target)
        harness.run(0.4)
    assert harness.settle()
    assert harness.axis.position_deg == pytest.approx(35.0, abs=1 / STEPS_PER_DEG)
    assert mechanics.position_deg == pytest.approx(35.0, abs=1 / STEPS_PER_DEG)
    assert harness.axis.homed is False or harness.axis.fault == ""


def test_stop_ramps_down_and_holds():
    harness = AxisHarness(make_config(), make_mechanics())
    harness.axis.goto(300.0)
    assert harness.run_until(lambda: harness.axis.position_deg > 20.0)
    harness.axis.stop()
    assert harness.settle()
    stopped_at = harness.axis.position_deg
    harness.run(0.5)
    assert harness.axis.position_deg == pytest.approx(stopped_at, abs=1e-9)
    assert stopped_at < 300.0


# -- backlash ---------------------------------------------------------------


def test_backlash_take_up_moves_the_motor_but_not_the_position():
    mechanics = make_mechanics(backlash_deg=1.0)
    harness = AxisHarness(make_config(backlash_deg=1.0), mechanics)
    harness.axis.goto(20.0)
    assert harness.settle()
    pulses_out = mechanics.pulses

    harness.axis.goto(10.0)
    assert harness.settle()
    extra = mechanics.pulses - pulses_out
    lash = round(1.0 * STEPS_PER_DEG)
    assert extra == pytest.approx(10.0 * STEPS_PER_DEG + lash, abs=2)
    # Compensation means the mechanism really is where the axis says it is.
    assert mechanics.position_deg == pytest.approx(10.0, abs=2 / STEPS_PER_DEG)


def test_uncompensated_backlash_shows_up_as_position_error():
    """Sanity check on the model itself: with lash and no compensation the
    mechanism lags, which is exactly what the previous test proves we fix."""
    mechanics = make_mechanics(backlash_deg=1.0)
    harness = AxisHarness(make_config(backlash_deg=0.0), mechanics)
    harness.axis.goto(20.0)
    assert harness.settle()
    harness.axis.goto(10.0)
    assert harness.settle()
    assert mechanics.position_deg == pytest.approx(11.0, abs=2 / STEPS_PER_DEG)


# -- homing -----------------------------------------------------------------


def test_homing_without_an_endstop_accepts_the_current_position():
    harness = AxisHarness(make_config(home_position_deg=0.0), make_mechanics())
    harness.axis.home()
    harness.run(0.05)
    assert harness.axis.homed
    assert harness.axis.position_deg == pytest.approx(0.0)
    assert not harness.axis.moving


def test_homing_finds_the_switch_and_zeroes_there():
    mechanics = make_mechanics(position_deg=30.0, endstop_deg=0.0, hard_min_deg=-3.0)
    harness = AxisHarness(make_config(min_deg=0.0, home_backoff_deg=1.0), mechanics)
    harness.axis.home()
    assert harness.run_until(lambda: harness.axis.homed, timeout_s=60.0)
    assert harness.axis.position_deg == pytest.approx(0.0)
    # And the antenna really is at the switch, not just bookkept there.
    assert mechanics.position_deg == pytest.approx(0.0, abs=0.2)
    assert harness.axis.fault == ""


def test_homing_approaches_the_switch_twice_for_repeatability():
    """Fast seek, back off, crawl back on. The slow approach sets the datum."""
    mechanics = make_mechanics(position_deg=20.0, endstop_deg=0.0, hard_min_deg=-3.0)
    harness = AxisHarness(make_config(min_deg=0.0, home_backoff_deg=1.0), mechanics)
    harness.axis.home()
    assert harness.run_until(lambda: harness.axis.homed, timeout_s=60.0)
    # The trace must leave the switch and come back to it.
    released = [p for p in mechanics.trace if p > 0.5]
    assert released, "expected a back-off away from the switch"
    assert mechanics.trace[-1] <= 0.1


def test_homing_faults_when_the_switch_never_closes():
    mechanics = make_mechanics(position_deg=30.0, endstop_deg=-9999.0)
    harness = AxisHarness(make_config(min_deg=0.0, max_deg=40.0), mechanics)
    harness.axis.home()
    assert harness.run_until(lambda: harness.axis.fault != "", timeout_s=120.0)
    assert "homing failed" in harness.axis.fault
    assert not harness.axis.homed


# -- interlocks -------------------------------------------------------------


def test_unhomed_axis_refuses_to_move():
    harness = AxisHarness(make_config(), make_mechanics(), require_homing=True)
    with pytest.raises(MotionBlocked, match="not homed"):
        harness.axis.goto(45.0)


def test_homing_lifts_the_interlock():
    mechanics = make_mechanics(position_deg=10.0, endstop_deg=0.0, hard_min_deg=-3.0)
    harness = AxisHarness(make_config(min_deg=0.0), mechanics, require_homing=True)
    harness.axis.home()
    assert harness.run_until(lambda: harness.axis.homed, timeout_s=60.0)
    harness.axis.goto(20.0)
    assert harness.settle()
    assert harness.axis.position_deg == pytest.approx(20.0, abs=1 / STEPS_PER_DEG)


def test_endstop_during_normal_motion_stops_the_axis():
    """The switch is a limit at all times, not just a homing aid.

    v0.1 only read it while homing, so a bad target could drive the antenna
    straight through the stop with the switch closed the whole way.
    """
    mechanics = make_mechanics(endstop_deg=-5.0, hard_min_deg=-8.0)
    harness = AxisHarness(make_config(min_deg=-10.0), mechanics)
    harness.axis.goto(-9.0)
    assert harness.run_until(lambda: harness.axis.fault != "", timeout_s=60.0)
    assert "endstop" in harness.axis.fault
    harness.run(0.5)
    # It stopped at the switch rather than driving on to the commanded angle.
    assert mechanics.position_deg > -6.5
    assert mechanics.steps_lost == 0, "must not have run into the hard stop"


def test_a_tripped_limit_blocks_further_motion_into_it_but_not_away():
    mechanics = make_mechanics(endstop_deg=-5.0, hard_min_deg=-8.0)
    harness = AxisHarness(make_config(min_deg=-10.0), mechanics)
    harness.axis.goto(-9.0)
    assert harness.run_until(lambda: harness.axis.fault != "", timeout_s=60.0)

    with pytest.raises(MotionBlocked, match="limit"):
        harness.axis.goto(-9.0)

    harness.axis.goto(5.0)  # away from the limit: allowed
    assert harness.settle()
    assert mechanics.position_deg > 3.0


def test_emergency_stop_mid_segment_forces_a_re_home():
    """Cut pulses mid-segment and the step count is no longer trustworthy.

    Rather than quietly carry on with a position that may be wrong, the axis
    drops its homed flag — soft limits are only as good as the datum.
    """
    harness = AxisHarness(make_config(), make_mechanics(), require_homing=True)
    harness.axis.home()
    harness.run(0.05)
    harness.axis.goto(200.0)
    harness.run(0.3)
    harness.axis.emergency_stop("test")
    assert not harness.axis.homed
    assert "position reference lost" in harness.axis.fault
    with pytest.raises(MotionBlocked):
        harness.axis.goto(10.0)


def test_idle_disable_releases_the_motor_current():
    harness = AxisHarness(make_config(), make_mechanics(), idle_disable_s=0.5)
    harness.axis.goto(5.0)
    assert harness.settle()
    assert harness.driver.enabled
    harness.run(1.0)
    assert not harness.driver.enabled


# -- the model has teeth ----------------------------------------------------


def test_the_virtual_motor_stalls_if_the_plan_asks_too_much():
    """Guards the guard: if the planner ever exceeded the motor's capability
    the harness would notice, so a clean run in the tests above means something."""
    mechanics = make_mechanics(stall_speed_sps=50.0)  # far below the 500 sps plan
    harness = AxisHarness(make_config(), mechanics)
    harness.axis.goto(90.0)
    harness.settle()
    assert mechanics.stalls > 0
    assert mechanics.steps_lost > 0
    assert mechanics.position_deg < 90.0
