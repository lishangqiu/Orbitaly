"""The planner: exact step counts, honest ramps, no clock involved."""
import pytest

from orbitaly.motion import AxisKinematics, plan_move
from orbitaly.motion.planner import plan_constant, plan_decel

# Azimuth at the reference config: 200 steps/rev * 8 microsteps * 60:1 / 360
AZ = AxisKinematics(max_speed_sps=1600.0, accel_sps2=1066.7, min_speed_sps=8.0)


def net(segments):
    return sum(s.delta for s in segments)


def pulses(segments):
    return sum(s.steps for s in segments)


@pytest.mark.parametrize("distance", [1, 2, 3, 17, 100, 999, 2667, 100_000])
@pytest.mark.parametrize("sign", [1, -1])
def test_plan_lands_exactly_on_target(distance, sign):
    segments = plan_move(0, sign * distance, AZ)
    assert net(segments) == sign * distance


@pytest.mark.parametrize("distance", [50, 2667, 100_000])
def test_plan_never_exceeds_max_speed(distance):
    for segment in plan_move(0, distance, AZ):
        assert segment.speed_sps <= AZ.max_speed_sps + 1e-6


@pytest.mark.parametrize("distance", [50, 2667, 100_000])
def test_plan_ends_at_rest(distance):
    last = plan_move(0, distance, AZ)[-1]
    assert last.speed_sps <= AZ.min_speed_sps + 1e-6


@pytest.mark.parametrize("distance", [200, 2667, 20_000])
def test_no_segment_boundary_jumps_more_than_the_motor_can_swallow(distance):
    """A ramp is a staircase; this is the invariant that keeps the stairs low.

    Every boundary is a step change in rate that the motor absorbs instantly,
    so the height of each one — not the average acceleration — is what decides
    whether it keeps sync.
    """
    import math

    segments = plan_move(0, distance, AZ)
    for previous, current in zip(segments, segments[1:]):
        # A single step of the ideal ramp changes velocity by this much; no
        # plan can be smoother than that without violating the accel limit.
        v = previous.speed_sps
        one_step = math.sqrt(v**2 + 2.0 * AZ.accel_sps2) - v
        # Periods are whole microseconds, so each speed lands a little off the
        # one the planner asked for; at 1600 sps that is ~2.6 sps per segment.
        quantisation = 2.0 * v**2 / 1e6
        jump = abs(current.speed_sps - previous.speed_sps)
        assert jump <= AZ.max_jump_sps + one_step + quantisation + 1e-6


@pytest.mark.parametrize("distance", [200, 2667, 20_000])
def test_starting_and_stopping_happen_at_the_pull_in_rate(distance):
    segments = plan_move(0, distance, AZ)
    assert segments[0].speed_sps <= AZ.min_speed_sps + 1e-6
    assert segments[-1].speed_sps <= AZ.min_speed_sps + 1e-6


def test_segments_are_short_enough_to_retarget_promptly():
    for segment in plan_move(0, 100_000, AZ):
        # A single step at the crawl rate is longer than the cap and cannot be
        # divided; everything else must respect it.
        assert segment.steps == 1 or segment.duration_s <= AZ.segment_max_s + 1e-9


def test_backlash_pulses_turn_the_motor_without_moving_the_position():
    kin = AxisKinematics(max_speed_sps=1600.0, accel_sps2=1066.7, backlash_steps=267)
    segments = plan_move(0, 500, kin, last_dir=-1)
    assert net(segments) == 500
    assert pulses(segments) == 500 + 267
    assert sum(s.steps for s in segments if not s.counts) == 267


def test_no_backlash_without_a_reversal():
    kin = AxisKinematics(max_speed_sps=1600.0, accel_sps2=1066.7, backlash_steps=267)
    segments = plan_move(0, 500, kin, last_dir=1)
    assert pulses(segments) == 500


def test_reversal_from_full_speed_decelerates_first():
    segments = plan_move(1000, 0, AZ, speed_sps=1600.0, moving_dir=1)
    # It must run on before it can turn around, and still land on target.
    assert net(segments) == -1000
    forward = [s for s in segments if s.direction == 1]
    assert pulses(forward) == AZ.stop_steps(1600.0)


def test_target_too_close_to_stop_in_overshoots_then_returns():
    segments = plan_move(0, 50, AZ, speed_sps=1600.0, moving_dir=1)
    assert net(segments) == 50
    assert any(s.direction == -1 for s in segments), "must come back from the overshoot"


def test_decel_matches_the_advertised_stopping_distance():
    segments = plan_decel(1600.0, 1, AZ)
    assert net(segments) == AZ.stop_steps(1600.0)
    assert segments[-1].speed_sps <= AZ.min_speed_sps + 1e-6


def test_decel_from_rest_is_a_no_op():
    assert plan_decel(AZ.min_speed_sps, 1, AZ) == []
    assert plan_decel(500.0, 0, AZ) == []


def test_plan_to_current_position_does_nothing():
    assert plan_move(500, 500, AZ) == []


def test_constant_speed_run_is_flat():
    segments = plan_constant(1000, 400.0, -1, AZ)
    assert net(segments) == -1000
    assert {s.period_us for s in segments} == {AZ.period_us(400.0)}
