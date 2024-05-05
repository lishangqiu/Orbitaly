"""Clock seam.

Everything in the motion stack reads time and sleeps through a clock object
so the test harness can run a twelve-minute satellite pass in milliseconds and
get the same ordering every run.
"""
from __future__ import annotations

import threading
import time
from typing import Protocol


class Clock(Protocol):
    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class RealClock:
    """Wall clock. What runs on the Pi."""

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class VirtualClock:
    """Test clock: time only moves when a test says so.

    ``sleep`` does not block — it records the request and returns. Tests drive
    the system with :meth:`advance`, which is the only thing that moves time.
    Safe to share across threads, though the harness normally pumps the motion
    stack synchronously to keep runs reproducible.
    """

    def __init__(self, start: float = 0.0):
        self._now = start
        self._lock = threading.Lock()
        self.slept = 0.0

    def monotonic(self) -> float:
        with self._lock:
            return self._now

    def sleep(self, seconds: float) -> None:
        if seconds <= 0:
            return
        with self._lock:
            # Sub-microsecond waits (dir setup) are modelled as instantaneous
            # elapsed time; anything longer is the caller's own pacing loop and
            # must not move the clock on its own.
            self.slept += seconds
            if seconds <= 1e-3:
                self._now += seconds

    def advance(self, seconds: float) -> float:
        with self._lock:
            self._now += seconds
            return self._now
