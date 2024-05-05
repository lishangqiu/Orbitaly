"""Segment-based motion control.

Orbitaly does not toggle a GPIO pin once per step from Python. The planner
turns a move into a short list of constant-rate *segments* — ``(direction,
steps, period)`` — and a backend hands each segment to hardware that emits
exactly that many pulses at that rate. Python then runs at tens of hertz
feeding segments instead of thousands of hertz toggling a pin.

The property that makes this safe: **step counts are exact**. Timing jitter
changes how smoothly an axis moves, never where it ends up.
"""
from .segment import AxisKinematics, Segment
from .planner import plan_move, plan_decel

__all__ = ["AxisKinematics", "Segment", "plan_move", "plan_decel"]
