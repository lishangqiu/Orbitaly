"""Axis supervisor: plans, feeds hardware, homes, and enforces the interlocks.

Runs one cheap thread per axis at a couple of hundred hertz doing bookkeeping
only — the pulses themselves are hardware's problem. The supervisor's job is
to keep roughly 60 ms of motion queued, replan whenever the target moves, and
stop everything the instant an endstop or E-stop says so.

Position is *derived from steps hardware reports as executed*, never from
elapsed time, so a busy Pi cannot make the axis lie about where it is.
"""
from __future__ import annotations

import logging
import threading
from enum import Enum

from ..config import AxisConfig
from .driver import AxisDriver
from .errors import MotionBlocked
from .planner import plan_constant, plan_decel, plan_move
from .segment import AxisKinematics

log = logging.getLogger(__name__)


class HomePhase(str, Enum):
    IDLE = "idle"
    SEEK_FAST = "seek_fast"
    BACKOFF = "backoff"
    SEEK_SLOW = "seek_slow"


class StepperAxis:
    TICK_S = 0.002
    #: keep this much motion queued in hardware
    BUFFER_S = 0.06

    def __init__(
        self,
        config: AxisConfig,
        driver: AxisDriver,
        name: str = "axis",
        *,
        clock=None,
        require_homing: bool = False,
        idle_disable_s: float = 0.0,
        realtime_priority: int = 0,
    ):
        from .clock import RealClock

        self.config = config
        self.driver = driver
        self.name = name
        self.clock = clock or RealClock()
        self.require_homing = require_homing
        self.idle_disable_s = idle_disable_s
        self.realtime_priority = realtime_priority

        self._spd = config.steps_per_deg
        self.kin = AxisKinematics(
            max_speed_sps=max(config.max_speed_dps * self._spd, config.start_speed_sps),
            accel_sps2=max(config.accel_dps2 * self._spd, 1.0),
            min_speed_sps=config.start_speed_sps,
            backlash_steps=int(round(config.backlash_deg * self._spd)),
            segment_max_s=config.segment_ms / 1000.0,
        )

        self._lock = threading.RLock()
        self._origin_steps = int(round(config.home_position_deg * self._spd))
        self._target_steps = self._origin_steps
        self._plan: list = []
        self._replan = False
        self._homed = False
        self._home_phase = HomePhase.IDLE
        self._home_travel = 0
        self._backoff_count = 0
        self._endstop_hit = False
        self._fault = ""
        self._limit_dir = 0  # which way is blocked by a tripped limit
        self._enabled = False
        self._last_motion_t = self.clock.monotonic()
        self._estopped = False

        self._stop_flag = threading.Event()
        self._thread: threading.Thread | None = None
        driver.arm_endstop(self._on_endstop)

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self._set_enabled(True)
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"axis-{self.name}")
        self._thread.start()

    def _apply_realtime_priority(self) -> None:
        """Ask for SCHED_FIFO so a busy Pi cannot starve the segment feeder.

        Missing a feed window does not cost steps — the hardware queue holds
        60 ms and every plan ends at rest — but it does mean the queue can run
        dry and the axis stutters. Needs CAP_SYS_NICE; degrades quietly.
        """
        if not self.realtime_priority:
            return
        import os

        try:
            os.sched_setscheduler(
                0, os.SCHED_FIFO, os.sched_param(self.realtime_priority)
            )
            log.info("%s: running at SCHED_FIFO %d", self.name, self.realtime_priority)
        except (PermissionError, OSError, AttributeError) as exc:
            log.info(
                "%s: staying on the normal scheduler (%s). Grant CAP_SYS_NICE to change that.",
                self.name,
                exc,
            )

    def close(self) -> None:
        self._stop_flag.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.driver.abort()
        self._set_enabled(False)
        self.driver.close()

    def _run(self) -> None:
        self._apply_realtime_priority()
        while not self._stop_flag.is_set():
            try:
                self.tick()
            except Exception:  # noqa: BLE001 - a supervisor that dies leaves motors running
                log.exception("%s: supervisor tick failed", self.name)
                self.emergency_stop("supervisor error")
            self.clock.sleep(self.TICK_S)

    # -- introspection ------------------------------------------------------

    @property
    def position_steps(self) -> int:
        return self._origin_steps + self.driver.consumed_steps()

    @property
    def position_deg(self) -> float:
        return self.position_steps / self._spd

    @property
    def target_deg(self) -> float:
        return self._target_steps / self._spd

    @property
    def moving(self) -> bool:
        return bool(self._plan) or self.driver.busy()

    @property
    def homed(self) -> bool:
        return self._homed

    @property
    def homing(self) -> bool:
        return self._home_phase is not HomePhase.IDLE

    @property
    def fault(self) -> str:
        return self._fault

    # -- commands -----------------------------------------------------------

    def goto(self, degrees: float) -> None:
        clamped = min(max(degrees, self.config.min_deg), self.config.max_deg)
        target = int(round(clamped * self._spd))
        with self._lock:
            direction = 1 if target > self.position_steps else -1
            self._check_movable(direction)
            self._home_phase = HomePhase.IDLE
            self._target_steps = target
            self._replan = True

    def jog(self, delta_deg: float) -> None:
        self.goto(self.target_deg + delta_deg)

    def stop(self) -> None:
        """Controlled stop: drop the plan and ramp down from the committed rate."""
        with self._lock:
            self._home_phase = HomePhase.IDLE
            self._plan = plan_decel(
                self.driver.committed_speed_sps(), self.driver.committed_direction(), self.kin
            )
            self._replan = False
            self._target_steps = self.driver.committed_steps() + self._origin_steps + sum(
                s.delta for s in self._plan
            )

    def emergency_stop(self, reason: str) -> None:
        """Cut pulses now. Position reference may not survive this — by design."""
        with self._lock:
            self._plan = []
            self._replan = False
            self._home_phase = HomePhase.IDLE
            result = self.driver.abort()
            self._target_steps = self.position_steps
            self._estopped = True
            if not result.exact:
                self._homed = False
                self._set_fault(
                    f"{reason}; position reference lost mid-segment — re-home before moving"
                )
            else:
                self._set_fault(reason)

    def home(self) -> None:
        # Homing is the recovery path, so it is never gated on the fault state.
        with self._lock:
            self._plan = []
            self._replan = False
            self._endstop_hit = False
            self._home_travel = 0
            self._backoff_count = 0
            self._homed = False
            self.driver.abort()
            self._set_enabled(True)
            if not self.driver.has_endstop():
                self._finish_home(self.config.home_position_deg)
                return
            self._home_phase = (
                HomePhase.BACKOFF if self.driver.endstop_triggered() else HomePhase.SEEK_FAST
            )

    def clear_fault(self) -> bool:
        """Clear a latched fault if the condition is gone. Returns True if cleared."""
        with self._lock:
            if self.driver.endstop_triggered():
                return False
            self._fault = ""
            self._limit_dir = 0
            self._estopped = False
            return True

    # -- tick ---------------------------------------------------------------

    def tick(self) -> None:
        with self._lock:
            if self._endstop_hit:
                self._endstop_hit = False
                self._handle_endstop()
            if self.homing:
                self._home_tick()
            else:
                self._motion_tick()
            self.driver.pump()
            self._idle_tick()

    def _motion_tick(self) -> None:
        if self._replan:
            self._plan = plan_move(
                self.driver.committed_steps() + self._origin_steps,
                self._target_steps,
                self.kin,
                speed_sps=self.driver.committed_speed_sps(),
                moving_dir=self.driver.committed_direction(),
                last_dir=self.driver.last_direction(),
            )
            self._replan = False
        self._feed()

    def _feed(self) -> None:
        while self._plan and self.driver.pending_s() < self.BUFFER_S:
            segment = self._plan[0]
            if self._limit_dir and segment.direction == self._limit_dir:
                self._plan = []
                return
            self._set_enabled(True)
            self.driver.enqueue(self._plan.pop(0))
            self._last_motion_t = self.clock.monotonic()

    def _idle_tick(self) -> None:
        if self.moving:
            self._last_motion_t = self.clock.monotonic()
            return
        if (
            self.idle_disable_s > 0
            and self._enabled
            and self.clock.monotonic() - self._last_motion_t > self.idle_disable_s
        ):
            self._set_enabled(False)

    # -- homing -------------------------------------------------------------

    def _home_tick(self) -> None:
        seek_speed = max(self.kin.max_speed_sps * 0.25, self.kin.min_speed_sps)
        slow_speed = max(self.kin.max_speed_sps * 0.05, self.kin.min_speed_sps)
        backoff_steps = max(1, int(round(self.config.home_backoff_deg * self._spd)))
        travel_limit = int((self.config.max_deg - self.config.min_deg) * self._spd) + backoff_steps

        if self._home_phase is HomePhase.SEEK_FAST:
            if self.driver.endstop_triggered():
                self.driver.abort()
                self._home_phase = HomePhase.BACKOFF
                return
            if self._home_travel > travel_limit:
                self.driver.abort()
                self._home_phase = HomePhase.IDLE
                self._set_fault("homing failed: endstop never closed")
                return
            self._crawl(-1, seek_speed)

        elif self._home_phase is HomePhase.BACKOFF:
            if not self.driver.busy() and not self._plan:
                if not self.driver.endstop_triggered():
                    self._home_travel = 0
                    self._home_phase = HomePhase.SEEK_SLOW
                elif self._backoff_count >= 4:
                    self._home_phase = HomePhase.IDLE
                    self._set_fault("homing failed: endstop stayed closed while backing off")
                else:
                    self._backoff_count += 1
                    self._plan = plan_constant(backoff_steps, slow_speed, 1, self.kin)
            self._feed()

        elif self._home_phase is HomePhase.SEEK_SLOW:
            # Second, slow approach: this is the one that sets the reference,
            # and approaching at crawl speed is what makes it repeatable.
            if self.driver.endstop_triggered():
                self.driver.abort()
                self._finish_home(self.config.min_deg)
                return
            if self._home_travel > 4 * backoff_steps:
                self.driver.abort()
                self._home_phase = HomePhase.IDLE
                self._set_fault("homing failed: endstop not found on slow approach")
                return
            self._crawl(-1, slow_speed)

    def _crawl(self, direction: int, speed_sps: float) -> None:
        """Keep a short constant-rate run queued while seeking."""
        if self.driver.pending_s() >= self.BUFFER_S:
            return
        self._set_enabled(True)
        chunk = max(1, int(speed_sps * self.kin.segment_max_s))
        for segment in plan_constant(chunk, speed_sps, direction, self.kin):
            self.driver.enqueue(segment)
            self._home_travel += segment.steps
        self._last_motion_t = self.clock.monotonic()

    def _finish_home(self, position_deg: float) -> None:
        self._origin_steps = int(round(position_deg * self._spd)) - self.driver.consumed_steps()
        self._target_steps = int(round(position_deg * self._spd))
        self._home_phase = HomePhase.IDLE
        self._home_travel = 0
        self._homed = True
        self._estopped = False
        self._fault = ""
        self._limit_dir = 0
        log.info("%s: homed at %.3f deg", self.name, position_deg)

    # -- safety -------------------------------------------------------------

    def _on_endstop(self) -> None:
        """Called from the backend's edge callback — keep it to a flag."""
        self._endstop_hit = True

    def _handle_endstop(self) -> None:
        if self.homing:
            return  # the homing state machine owns the switch while it runs
        if not self.driver.endstop_triggered():
            return
        log.warning("%s: endstop tripped during motion — stopping", self.name)
        self._limit_dir = -1
        self.emergency_stop("endstop tripped during motion")

    def _check_movable(self, direction: int) -> None:
        if self._limit_dir:
            # Backing away from a tripped limit is how you recover, so it is
            # always allowed — even while the fault is latched.
            if direction == self._limit_dir:
                raise MotionBlocked(f"{self.name}: at limit; move the other way to clear")
            return
        if self._fault:
            raise MotionBlocked(f"{self.name}: {self._fault}")
        if self.require_homing and not self._homed:
            raise MotionBlocked(f"{self.name}: not homed — run homing first")

    def _set_fault(self, message: str) -> None:
        self._fault = message
        log.error("%s: %s", self.name, message)

    def _set_enabled(self, enabled: bool) -> None:
        if enabled != self._enabled:
            self.driver.set_enabled(enabled)
            self._enabled = enabled
