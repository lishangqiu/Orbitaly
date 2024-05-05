"""Scheduler: conflict resolution and handover, on a controlled clock."""
import pytest

from orbitaly.config import SchedulerConfig, WatchEntry
from orbitaly.core.scheduler import ScheduledPass, Scheduler, resolve_conflicts
from orbitaly.core.tracker import TrackerState


def make_pass(norad_id, aos, duration=600, max_el=30.0, priority=0, name="SAT"):
    return ScheduledPass(
        norad_id=norad_id,
        name=name,
        aos=aos,
        tca=aos + duration / 2,
        los=aos + duration,
        max_elevation=max_el,
        priority=priority,
    )


# -- conflict resolution ----------------------------------------------------


def test_non_overlapping_passes_are_all_kept():
    plan = resolve_conflicts([make_pass(1, 0), make_pass(2, 1000), make_pass(3, 2000)])
    assert [p.norad_id for p in plan] == [1, 2, 3]


def test_the_higher_pass_wins_a_clash():
    plan = resolve_conflicts([make_pass(1, 0, max_el=12.0), make_pass(2, 100, max_el=70.0)])
    assert [p.norad_id for p in plan] == [2]


def test_priority_beats_elevation():
    """A 10 degree pass of the satellite you actually care about outranks a
    beautiful pass of one you do not."""
    plan = resolve_conflicts(
        [make_pass(1, 0, max_el=10.0, priority=5), make_pass(2, 100, max_el=80.0)]
    )
    assert [p.norad_id for p in plan] == [1]


def test_the_lead_time_counts_as_part_of_the_pass():
    """Two passes 60 s apart still clash if we need 120 s to pre-position."""
    back_to_back = [make_pass(1, 0, duration=600), make_pass(2, 660, duration=600)]
    assert len(resolve_conflicts(back_to_back, lead_s=0.0)) == 2
    assert len(resolve_conflicts(back_to_back, lead_s=120.0)) == 1


def test_plan_comes_back_in_time_order():
    plan = resolve_conflicts([make_pass(1, 5000), make_pass(2, 1000), make_pass(3, 3000)])
    assert [p.aos for p in plan] == [1000, 3000, 5000]


# -- handover ---------------------------------------------------------------


class FakeTracker:
    def __init__(self):
        self.state = TrackerState.IDLE
        self.tracked_norad_id = None
        self.calls = []

    def track(self, norad_id):
        self.calls.append(("track", norad_id))
        self.tracked_norad_id = norad_id
        self.state = TrackerState.ACQUIRING

    def stop_tracking(self):
        self.calls.append(("stop",))
        self.tracked_norad_id = None
        self.state = TrackerState.IDLE

    def park(self):
        self.calls.append(("park",))


class StubScheduler(Scheduler):
    """Scheduler with a fixed plan, so tests control the timeline exactly."""

    def __init__(self, plan, config=None, **kwargs):
        self.now = 0.0
        config = config or SchedulerConfig(enabled=True, preposition_lead_s=120.0)
        super().__init__(
            config, predictor=None, tle_manager=None,
            tracker=FakeTracker(), rotator=None, time_fn=lambda: self.now, **kwargs
        )
        self._fixed_plan = plan

    def _replan(self, now):
        self._plan = resolve_conflicts(self._fixed_plan, self.config.preposition_lead_s)
        self._planned_at = now


def test_tracking_engages_before_aos_and_releases_after_los():
    sched = StubScheduler([make_pass(25544, 1000, duration=600)])

    sched.now = 500
    sched.tick()
    assert sched.tracker.calls == [], "too early"

    sched.now = 900  # inside the 120 s pre-position window
    sched.tick()
    assert sched.tracker.calls == [("track", 25544)]

    sched.now = 1300  # mid-pass: must not re-issue
    sched.tick()
    assert sched.tracker.calls == [("track", 25544)]

    sched.now = 1700  # after LOS
    sched.tick()
    assert sched.tracker.calls[-1] == ("stop",)


def test_it_moves_on_to_the_next_satellite():
    sched = StubScheduler([make_pass(1, 1000, duration=600), make_pass(2, 3000, duration=600)])
    for t in (900, 1300, 1700, 2900, 3300):
        sched.now = t
        sched.tick()
    assert [c for c in sched.tracker.calls if c[0] == "track"] == [("track", 1), ("track", 2)]


def test_manual_control_ends_the_scheduled_pass_rather_than_fighting():
    sched = StubScheduler([make_pass(1, 1000, duration=600)])
    sched.now = 900
    sched.tick()
    assert sched.tracker.calls == [("track", 1)]

    sched.tracker.state = TrackerState.MANUAL  # operator grabs the jog buttons
    for t in (1000, 1100, 1200, 1300):
        sched.now = t
        sched.tick()
    assert sched.tracker.calls == [("track", 1)], "must not keep grabbing the rotator back"


def test_the_next_pass_is_still_scheduled_after_a_manual_takeover():
    sched = StubScheduler([make_pass(1, 1000, duration=600), make_pass(2, 3000, duration=600)])
    sched.now = 900
    sched.tick()
    sched.tracker.state = TrackerState.MANUAL
    sched.now = 1200
    sched.tick()

    sched.tracker.state = TrackerState.IDLE
    sched.now = 2900
    sched.tick()
    assert ("track", 2) in sched.tracker.calls


def test_disabling_the_scheduler_hands_the_rotator_back():
    sched = StubScheduler([make_pass(1, 1000, duration=600)])
    sched.now = 1100
    sched.tick()
    sched.set_enabled(False)
    assert sched.tracker.calls[-1] == ("stop",)
    sched.now = 1200
    sched.tick()
    assert sched.tracker.calls[-1] == ("stop",), "stays out of the way while disabled"


def test_parking_after_a_quiet_spell():
    config = SchedulerConfig(enabled=True, preposition_lead_s=120.0, park_after_idle_s=300.0)
    sched = StubScheduler([make_pass(1, 1000, duration=600)], config=config)
    sched.now = 1100
    sched.tick()
    sched.now = 1700  # LOS: releases and starts the idle timer
    sched.tick()
    sched.now = 1900
    sched.tick()
    assert ("park",) not in sched.tracker.calls
    sched.now = 2100  # more than 300 s idle
    sched.tick()
    assert ("park",) in sched.tracker.calls


# -- the watch list ---------------------------------------------------------


def test_watch_list_accepts_entries_dicts_and_bare_ids():
    config = SchedulerConfig(
        satellites=[WatchEntry(norad_id=1, priority=3), {"norad_id": 2}, 25544]
    )
    sched = Scheduler(config, None, None, FakeTracker(), None)
    entries = sched.watch_list()
    assert [e.norad_id for e in entries] == [1, 2, 25544]
    assert entries[0].priority == 3
    assert entries[2].min_elevation_deg is None


def test_status_lists_what_is_coming(monkeypatch):
    sched = StubScheduler([make_pass(1, 1000, name="AO-91"), make_pass(2, 5000, name="SO-50")])
    sched.now = 100
    sched.tick()
    status = sched.status()
    assert status["enabled"]
    assert [p["name"] for p in status["upcoming"]] == ["AO-91", "SO-50"]
    assert status["engaged"] is None


# -- integration with real orbital predictions ------------------------------


def test_planning_against_real_passes_produces_a_conflict_free_timeline():
    from orbitaly.config import StationConfig
    from orbitaly.core.predictor import Predictor
    from test_pi_simulation import ISS, MODERATE_PASS_AOS

    class OneSat:
        def get(self, norad_id):
            return ISS if norad_id == 25544 else None

    config = SchedulerConfig(
        enabled=True, satellites=[{"norad_id": 25544}], lookahead_hours=24.0
    )
    sched = Scheduler(
        config,
        Predictor(StationConfig(latitude=40.44, longitude=-79.99, altitude_m=300)),
        OneSat(),
        FakeTracker(),
        None,
        time_fn=lambda: MODERATE_PASS_AOS - 3600,
    )
    sched.tick()
    plan = sched._plan
    assert len(plan) >= 4
    for earlier, later in zip(plan, plan[1:]):
        assert not earlier.overlaps(later, config.preposition_lead_s)
        assert later.max_elevation >= 5.0
