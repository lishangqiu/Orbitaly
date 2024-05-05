"""The electrical interface to one stepper driver, at segment granularity.

A backend accepts whole segments and is responsible for emitting exactly the
requested number of pulses. Direction changes are the backend's problem too:
it must let queued pulses drain, flip ``dir``, honour the driver chip's
direction setup time, and only then pulse again.
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from collections import deque
from typing import Callable, NamedTuple

from .segment import Segment


class AbortResult(NamedTuple):
    """Outcome of an emergency stop.

    ``exact`` is False when pulses were cut off mid-segment and the backend
    cannot say how many of them the motor actually received. The axis treats
    an inexact abort as a loss of position reference and demands re-homing —
    guessing would silently corrupt the soft limits.
    """

    consumed_steps: int
    exact: bool


class AxisDriver(ABC):
    """One physical axis: step, dir, enable, and (optionally) an endstop."""

    @abstractmethod
    def enqueue(self, segment: Segment) -> None:
        """Queue a segment for execution. Returns immediately."""

    @abstractmethod
    def consumed_steps(self) -> int:
        """Signed logical steps actually executed so far (backlash excluded)."""

    @abstractmethod
    def committed_steps(self) -> int:
        """Signed logical steps executed *or queued* — what the planner plans from."""

    @abstractmethod
    def pending_s(self) -> float:
        """Seconds of motion still queued. The supervisor tops this up."""

    @abstractmethod
    def busy(self) -> bool:
        """True while pulses are queued or in flight."""

    @abstractmethod
    def abort(self) -> AbortResult:
        """Stop pulsing immediately and discard the queue."""

    def committed_speed_sps(self) -> float:
        """Rate at the end of the queue — the speed a replan must start from."""
        return 0.0

    def committed_direction(self) -> int:
        """Direction at the end of the queue (0 when it ends at rest)."""
        return 0

    def last_direction(self) -> int:
        """Last direction physically moved, for backlash take-up."""
        return 0

    def set_enabled(self, enabled: bool) -> None:
        pass

    def has_endstop(self) -> bool:
        return False

    def endstop_triggered(self) -> bool:
        return False

    def arm_endstop(self, callback: Callable[[], None] | None) -> None:
        """Register a callback fired as soon as the endstop closes."""

    def close(self) -> None:
        pass


class BufferedAxisDriver(AxisDriver):
    """Shared bookkeeping for backends that stream segments to hardware.

    Keeps a software queue in front of a shallow hardware queue, tracks what
    has been executed versus merely committed, and serialises the drain →
    flip dir → wait → resume dance around every direction change.

    Subclasses implement three primitives: :meth:`_hand_off`, :meth:`_set_dir`
    and :meth:`_stop_pulses`.
    """

    #: How much motion to keep queued in hardware. Long enough to hide Python
    #: scheduling latency, short enough that a retarget lands promptly.
    BUFFER_S = 0.06
    MAX_IN_FLIGHT = 4

    def __init__(self, *, dir_setup_s: float = 20e-6, clock=None):
        from .clock import RealClock

        self._clock = clock or RealClock()
        self._dir_setup_s = dir_setup_s
        self._lock = threading.RLock()
        self._queue: deque[Segment] = deque()
        self._in_flight: deque[tuple[Segment, float]] = deque()  # (segment, end_time)
        self._consumed = 0
        self._committed = 0
        self._hw_direction = 0
        self._last_direction = 0
        self._aborted = False
        self._busy_until = 0.0

    # -- AxisDriver ---------------------------------------------------------

    def enqueue(self, segment: Segment) -> None:
        with self._lock:
            self._aborted = False
            self._queue.append(segment)
            self._committed += segment.delta

    def consumed_steps(self) -> int:
        with self._lock:
            self._reap()
            return self._consumed

    def committed_steps(self) -> int:
        with self._lock:
            return self._committed

    def pending_s(self) -> float:
        with self._lock:
            self._reap()
            queued = sum(s.duration_s for s in self._queue)
            in_flight = max(0.0, self._busy_until - self._clock.monotonic())
            return queued + in_flight

    def busy(self) -> bool:
        with self._lock:
            self._reap()
            return bool(self._queue or self._in_flight)

    def abort(self) -> AbortResult:
        with self._lock:
            self._reap()
            exact = not self._in_flight
            self._stop_pulses()
            self._queue.clear()
            self._in_flight.clear()
            self._busy_until = 0.0
            self._committed = self._consumed
            self._aborted = True
            return AbortResult(self._consumed, exact)

    def committed_speed_sps(self) -> float:
        with self._lock:
            last = self._last_committed_segment()
            return last.speed_sps if last else 0.0

    def committed_direction(self) -> int:
        with self._lock:
            last = self._last_committed_segment()
            return last.direction if last else 0

    def last_direction(self) -> int:
        with self._lock:
            last = self._last_committed_segment()
            return last.direction if last else self._last_direction

    # -- pump ---------------------------------------------------------------

    def pump(self) -> None:
        """Move segments from the software queue into hardware. Idempotent."""
        with self._lock:
            self._reap()
            if self._aborted:
                return
            while self._queue and len(self._in_flight) < self.MAX_IN_FLIGHT:
                now = self._clock.monotonic()
                if max(0.0, self._busy_until - now) >= self.BUFFER_S:
                    return
                segment = self._queue[0]
                if segment.direction != self._hw_direction:
                    if self._hardware_busy():
                        return  # let the current direction finish first
                    self._set_dir(segment.direction > 0)
                    self._clock.sleep(self._dir_setup_s)
                    self._hw_direction = segment.direction
                    self._last_direction = segment.direction
                    now = self._clock.monotonic()
                self._queue.popleft()
                start = max(now, self._busy_until)
                self._busy_until = start + segment.duration_s
                self._in_flight.append((segment, self._busy_until))
                self._hand_off(segment)

    def _reap(self) -> None:
        now = self._clock.monotonic()
        while self._in_flight and self._in_flight[0][1] <= now:
            segment, _ = self._in_flight.popleft()
            self._consumed += segment.delta
            self._last_direction = segment.direction
            self._on_complete(segment)

    def _on_complete(self, segment: Segment) -> None:
        """Called once a segment's pulses have finished going out."""

    def _hardware_busy(self) -> bool:
        """Ground truth for 'are pulses still going out'. Overridable."""
        return bool(self._in_flight)

    def _last_committed_segment(self) -> Segment | None:
        if self._queue:
            return self._queue[-1]
        if self._in_flight:
            return self._in_flight[-1][0]
        return None

    # -- backend primitives -------------------------------------------------

    @abstractmethod
    def _hand_off(self, segment: Segment) -> None:
        """Give one segment to the hardware. Must not block."""

    @abstractmethod
    def _set_dir(self, forward: bool) -> None:
        """Drive the direction pin. Called only when pulses are stopped."""

    @abstractmethod
    def _stop_pulses(self) -> None:
        """Kill any pulse train in progress, right now."""
