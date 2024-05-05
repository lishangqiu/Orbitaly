"""Unattended operation: pick the best pass and work it, then the next one.

Given a watch list of satellites, the scheduler predicts everyone's passes,
resolves the overlaps, engages the tracker before AOS and lets go after LOS.

Two rules keep it out of trouble:

- **Overlaps are decided once, when planning**, by priority and then by peak
  elevation. Switching targets mid-pass would mean a long slew during the only
  minutes either satellite is visible, so a pass that has started is kept.
- **The operator always wins.** If anyone takes manual control during a
  scheduled pass, the scheduler abandons that pass rather than fighting for
  the rotator, and picks up again at the next one.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from ..config import SchedulerConfig, WatchEntry
from .predictor import Predictor
from .tracker import Tracker, TrackerState

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class ScheduledPass:
    norad_id: int
    name: str
    aos: float
    tca: float
    los: float
    max_elevation: float
    priority: int

    @property
    def key(self) -> tuple[int, float]:
        return (self.norad_id, round(self.aos))

    def score(self) -> tuple[int, float]:
        return (self.priority, self.max_elevation)

    def overlaps(self, other: "ScheduledPass", lead_s: float = 0.0) -> bool:
        return self.aos - lead_s < other.los and other.aos - lead_s < self.los

    def to_dict(self) -> dict:
        return {
            "norad_id": self.norad_id,
            "name": self.name,
            "aos": self.aos,
            "tca": self.tca,
            "los": self.los,
            "max_elevation": round(self.max_elevation, 1),
            "duration_s": round(self.los - self.aos),
            "priority": self.priority,
        }


class Scheduler:
    TICK_S = 2.0

    def __init__(
        self,
        config: SchedulerConfig,
        predictor: Predictor,
        tle_manager,
        tracker: Tracker,
        rotator,
        *,
        default_min_elevation: float = 5.0,
        time_fn=time.time,
    ):
        self.config = config
        self.predictor = predictor
        self.tle_manager = tle_manager
        self.tracker = tracker
        self.rotator = rotator
        self.default_min_elevation = default_min_elevation
        self._now = time_fn

        self.enabled = config.enabled
        self._lock = threading.RLock()
        self._plan: list[ScheduledPass] = []
        self._planned_at = -1e9
        self._engaged: ScheduledPass | None = None
        self._abandoned: set[tuple[int, float]] = set()
        self._idle_since: float | None = None
        self._stop_flag = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="scheduler")
        self._thread.start()

    def close(self) -> None:
        self._stop_flag.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                self.tick()
            except Exception:  # noqa: BLE001 - a scheduler crash must not stop tracking
                log.exception("Scheduler tick failed")
            self._stop_flag.wait(self.TICK_S)

    # -- control ------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.enabled = enabled
            if not enabled and self._engaged is not None:
                self.tracker.stop_tracking()
                self._engaged = None

    def watch_list(self) -> list[WatchEntry]:
        entries = []
        for item in self.config.satellites:
            if isinstance(item, WatchEntry):
                entries.append(item)
            elif isinstance(item, dict):
                entries.append(WatchEntry(**item))
            else:  # a bare NORAD id
                entries.append(WatchEntry(norad_id=int(item)))
        return entries

    def status(self) -> dict:
        with self._lock:
            now = self._now()
            upcoming = [p for p in self._plan if p.los > now]
            return {
                "enabled": self.enabled,
                "watching": len(self.config.satellites),
                "engaged": self._engaged.to_dict() if self._engaged else None,
                "upcoming": [p.to_dict() for p in upcoming[:10]],
            }

    # -- the loop -----------------------------------------------------------

    def tick(self) -> None:
        with self._lock:
            if not self.enabled:
                return
            now = self._now()
            if now - self._planned_at > self.config.replan_interval_s or not self._plan:
                self._replan(now)

            current = self._current_pass(now)
            if current is None:
                self._release(now)
                return
            if current.key in self._abandoned:
                return

            if self.tracker.state is TrackerState.MANUAL:
                # Somebody grabbed the controls. Give up this pass entirely
                # rather than yanking the rotator back every two seconds.
                if self._engaged is not None:
                    log.info("Manual control taken — abandoning the scheduled pass")
                    self._abandoned.add(current.key)
                    self._engaged = None
                return

            if self._engaged is None or self._engaged.key != current.key:
                log.info(
                    "Scheduling %s (NORAD %d), peak %.0f deg",
                    current.name,
                    current.norad_id,
                    current.max_elevation,
                )
                self.tracker.track(current.norad_id)
                self._engaged = current
                self._idle_since = None

    def _current_pass(self, now: float) -> ScheduledPass | None:
        lead = self.config.preposition_lead_s
        for entry in self._plan:
            if entry.aos - lead <= now <= entry.los:
                return entry
        return None

    def _release(self, now: float) -> None:
        if self._engaged is not None:
            log.info("Scheduled pass for NORAD %d complete", self._engaged.norad_id)
            self.tracker.stop_tracking()
            self._engaged = None
            self._idle_since = now
        if (
            self.config.park_after_idle_s > 0
            and self._idle_since is not None
            and now - self._idle_since > self.config.park_after_idle_s
        ):
            self._idle_since = None
            self.tracker.park()

    # -- planning -----------------------------------------------------------

    def _replan(self, now: float) -> None:
        candidates: list[ScheduledPass] = []
        for entry in self.watch_list():
            sat = self.tle_manager.get(entry.norad_id)
            if sat is None:
                log.warning("Scheduler: no TLE for NORAD %d", entry.norad_id)
                continue
            min_el = (
                entry.min_elevation_deg
                if entry.min_elevation_deg is not None
                else self.default_min_elevation
            )
            for p in self.predictor.next_passes(
                sat, now, hours=self.config.lookahead_hours, min_elevation=0.0
            ):
                if p.max_elevation < min_el:
                    continue
                candidates.append(
                    ScheduledPass(
                        norad_id=entry.norad_id,
                        name=sat.name,
                        aos=p.aos,
                        tca=p.tca,
                        los=p.los,
                        max_elevation=p.max_elevation,
                        priority=entry.priority,
                    )
                )
        self._plan = resolve_conflicts(candidates, self.config.preposition_lead_s)
        self._planned_at = now
        self._abandoned = {key for key in self._abandoned if any(p.key == key for p in self._plan)}
        log.info("Scheduler planned %d passes from %d candidates", len(self._plan), len(candidates))


def resolve_conflicts(passes: list[ScheduledPass], lead_s: float = 0.0) -> list[ScheduledPass]:
    """Drop overlapping passes, keeping the best of each clash.

    Best means higher priority, and then higher peak elevation — a 70 degree
    pass is worth far more than a 6 degree one at the same priority.
    """
    chosen: list[ScheduledPass] = []
    for candidate in sorted(passes, key=lambda p: (-p.priority, -p.max_elevation, p.aos)):
        if any(candidate.overlaps(kept, lead_s) for kept in chosen):
            continue
        chosen.append(candidate)
    return sorted(chosen, key=lambda p: p.aos)
