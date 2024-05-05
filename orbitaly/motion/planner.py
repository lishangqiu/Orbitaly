"""Trapezoidal motion planning in step space. Pure functions, no I/O, no clock.

The planner is deliberately total: given where an axis is, how fast it is
already moving, and where it must end up, it returns a complete list of
segments that ends **at rest on the target**. Because every plan terminates at
rest, an axis can always stop safely by simply letting its queue drain.

Ramps are emitted as a bounded number of constant-rate steps rather than one
segment per pulse: a 100 degree slew is a few dozen segments, not 27,000.
"""
from __future__ import annotations

import math

from .segment import AxisKinematics, Segment

__all__ = ["plan_move", "plan_decel", "plan_constant"]


def plan_move(
    current_steps: int,
    target_steps: int,
    kin: AxisKinematics,
    speed_sps: float | None = None,
    moving_dir: int = 0,
    last_dir: int = 0,
) -> list[Segment]:
    """Plan a move that ends at rest on ``target_steps``.

    ``speed_sps`` / ``moving_dir`` describe motion already underway (the axis
    replans from the end of what it has already committed to hardware, so a
    retarget never requires aborting pulses that are in flight).
    ``last_dir`` is the last direction physically moved, for backlash take-up.
    """
    speed = kin.min_speed_sps if speed_sps is None else max(speed_sps, kin.min_speed_sps)
    if moving_dir == 0:
        speed = kin.min_speed_sps

    segments: list[Segment] = []
    position = current_steps
    delta = target_steps - position
    direction = _sign(delta)

    # Already moving the wrong way: come to rest first, wherever that lands us.
    if moving_dir != 0 and direction != moving_dir and speed > kin.min_speed_sps:
        stop_segments = plan_decel(speed, moving_dir, kin)
        segments += stop_segments
        position += _net(stop_segments)
        last_dir = moving_dir
        speed, moving_dir = kin.min_speed_sps, 0
        delta = target_steps - position
        direction = _sign(delta)

    if direction == 0:
        return segments

    # Moving too fast to stop in the distance available: overshoot, then come back.
    if moving_dir == direction and kin.stop_steps(speed) > abs(delta):
        stop_segments = plan_decel(speed, moving_dir, kin)
        segments += stop_segments
        position += _net(stop_segments)
        return segments + plan_move(position, target_steps, kin, last_dir=direction)

    if kin.backlash_steps and last_dir != 0 and direction != last_dir:
        segments += _split(
            Segment(direction, kin.backlash_steps, kin.period_us(kin.min_speed_sps), counts=False),
            kin,
        )

    segments += _trapezoid(abs(delta), direction, speed, kin)
    return segments


def plan_decel(speed_sps: float, direction: int, kin: AxisKinematics) -> list[Segment]:
    """Shortest safe ramp from ``speed_sps`` down to rest, in ``direction``."""
    if direction == 0:
        return []
    steps = kin.stop_steps(speed_sps)
    if steps <= 0:
        return []
    return _ramp(steps, speed_sps, kin.min_speed_sps, direction, kin)


def plan_constant(steps: int, speed_sps: float, direction: int, kin: AxisKinematics) -> list[Segment]:
    """A plain constant-rate run — used by homing, which crawls at one speed."""
    if steps <= 0 or direction == 0:
        return []
    return _split(Segment(direction, steps, kin.period_us(speed_sps)), kin)


# -- internals --------------------------------------------------------------


def _sign(value: int) -> int:
    return 1 if value > 0 else (-1 if value < 0 else 0)


def _net(segments: list[Segment]) -> int:
    return sum(s.delta for s in segments)


def _trapezoid(n: int, direction: int, v_start: float, kin: AxisKinematics) -> list[Segment]:
    """Accelerate from ``v_start``, cruise, decelerate to rest, over ``n`` steps."""
    a = kin.accel_sps2
    v_end = kin.min_speed_sps
    # Peak reachable if we accelerate then immediately decelerate over n steps.
    v_peak = math.sqrt(max((v_start**2 + v_end**2) / 2.0 + a * n, v_end**2))
    v_peak = min(v_peak, kin.max_speed_sps)

    n_accel = max(0, int(math.ceil((v_peak**2 - v_start**2) / (2.0 * a))))
    n_decel = max(0, int(math.ceil((v_peak**2 - v_end**2) / (2.0 * a))))
    if n_accel + n_decel > n:
        # Triangular: split the distance so both ramps meet at the same speed.
        n_decel = min(n, max(0, int(round((2.0 * a * n + v_start**2 - v_end**2) / (4.0 * a)))))
        n_accel = n - n_decel
        v_peak = min(math.sqrt(max(v_start**2 + 2.0 * a * n_accel, v_end**2)), kin.max_speed_sps)
    n_cruise = n - n_accel - n_decel

    segments: list[Segment] = []
    if n_accel:
        segments += _ramp(n_accel, v_start, v_peak, direction, kin)
    if n_cruise:
        segments += _split(Segment(direction, n_cruise, kin.period_us(v_peak)), kin)
    if n_decel:
        segments += _ramp(n_decel, v_peak, v_end, direction, kin)
    if not segments:  # n == 0 guarded by callers, but stay total
        segments = _split(Segment(direction, max(n, 1), kin.period_us(v_end)), kin)
    return segments


def _ramp(
    steps: int, v_from: float, v_to: float, direction: int, kin: AxisKinematics
) -> list[Segment]:
    """A velocity ramp as a staircase of constant-rate segments.

    Every step is given the lower of the two velocities the ideal ramp holds at
    its start and its end, so the staircase always sits *under* the ideal
    curve: the axis is never asked to accelerate harder than the limit allows.
    That also puts the first step of an acceleration and the last step of a
    deceleration at the pull-in rate, which is what makes starting and stopping
    clean.

    Consecutive steps are then merged while they stay within ``max_jump_sps``
    of each other, which is what bounds the velocity discontinuity a motor has
    to swallow at a segment boundary. Merging is why a 27,000-step slew is a
    few hundred segments instead of 27,000.
    """
    if steps <= 0:
        return []
    a = kin.accel_sps2
    rising = v_to >= v_from

    def ideal(k: int) -> float:
        """Velocity the continuous ramp holds after ``k`` steps."""
        if rising:
            return min(math.sqrt(v_from**2 + 2.0 * a * k), v_to)
        return max(math.sqrt(max(v_from**2 - 2.0 * a * k, 0.0)), v_to)

    speeds = [max(min(ideal(i), ideal(i + 1)), kin.min_speed_sps) for i in range(steps)]

    segments: list[Segment] = []
    i = 0
    while i < steps:
        first = speeds[i]
        max_steps = max(1, int(first * kin.segment_max_s))
        j = i + 1
        while j < steps and abs(speeds[j] - first) <= kin.max_jump_sps and (j - i) < max_steps:
            j += 1
        # Split on the *run* speed, not the group's first: a decelerating group
        # runs slower than it starts, so it lasts longer than a naive estimate.
        segments += _split(Segment(direction, j - i, kin.period_us(min(speeds[i:j]))), kin)
        i = j
    return segments


def _split(segment: Segment, kin: AxisKinematics) -> list[Segment]:
    """Cap segment duration so the axis can react promptly to a retarget."""
    max_steps = max(1, int(kin.segment_max_s * 1e6 / segment.period_us))
    if segment.steps <= max_steps:
        return [segment]
    pieces: list[Segment] = []
    remaining = segment.steps
    while remaining > 0:
        chunk = min(max_steps, remaining)
        pieces.append(
            Segment(segment.direction, chunk, segment.period_us, counts=segment.counts)
        )
        remaining -= chunk
    return pieces
