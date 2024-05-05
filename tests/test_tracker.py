"""Azimuth unwrap logic and tracker state transitions."""
from orbitaly.config import TrackerConfig
from orbitaly.core.tracker import Tracker, TrackerState, choose_az_offset, unwrap_azimuths


class FakeRotator:
    """Records commands; reports a 450-degree azimuth rotator."""

    def __init__(self, az_travel=(-90.0, 450.0)):
        self.commands = []
        self._az_travel = az_travel

    def goto(self, az, el):
        self.commands.append(("goto", az, el))

    def jog(self, d_az, d_el):
        self.commands.append(("jog", d_az, d_el))

    def stop(self):
        self.commands.append(("stop",))

    def home(self):
        self.commands.append(("home",))

    def state(self):
        raise NotImplementedError

    @property
    def azimuth_travel(self):
        return self._az_travel

    @property
    def elevation_travel(self):
        return (0.0, 180.0)


class FakeTleManager:
    def __init__(self, ids=(99999,)):
        self._ids = set(ids)

    def get(self, norad_id):
        return object() if norad_id in self._ids else None


def make_tracker(rotator=None):
    return Tracker(
        TrackerConfig(),
        predictor=None,
        tle_manager=FakeTleManager(),
        rotator=rotator or FakeRotator(),
    )


# -- unwrap helpers ---------------------------------------------------------

def test_unwrap_no_wrap():
    assert unwrap_azimuths([10, 20, 30]) == [10, 20, 30]


def test_unwrap_north_crossing_ascending():
    result = unwrap_azimuths([350, 355, 2, 8])
    assert result == [350, 355, 362, 368]


def test_unwrap_north_crossing_descending():
    result = unwrap_azimuths([10, 5, 358, 350])
    assert result == [10, 5, -2, -10]


def test_unwrap_empty():
    assert unwrap_azimuths([]) == []


def test_offset_fits_directly():
    assert choose_az_offset(10, 170, 0, 360) == 0.0


def test_offset_shifts_down():
    # Pass spans 350..368 (unwrapped); 450-degree rotator holds it as-is
    assert choose_az_offset(350, 368, -90, 450) == 0.0
    # A descending crossing -10..10 needs no shift either on a -90..450 rotator
    assert choose_az_offset(-10, 10, -90, 450) == 0.0
    # A fully negative span shifts up by 360 to fit a strict 0..360 rotator
    assert choose_az_offset(-30, -5, 0, 360) == 360.0
    # But a north-crossing span cannot fit a strict 0..360 rotator at all
    assert choose_az_offset(-10, 10, 0, 360) is None


def test_offset_impossible():
    # A pass spanning more than the rotator's travel cannot fit
    assert choose_az_offset(0, 500, 0, 360) is None


# -- tracker state machine --------------------------------------------------

def test_track_unknown_satellite():
    tracker = make_tracker()
    try:
        tracker.track(12345)
        raised = False
    except KeyError:
        raised = True
    assert raised
    assert tracker.state == TrackerState.IDLE


def test_track_engages_acquiring():
    tracker = make_tracker()
    tracker.track(99999)
    assert tracker.state == TrackerState.ACQUIRING
    assert tracker.tracked_norad_id == 99999
    tracker.stop_tracking()
    assert tracker.state == TrackerState.IDLE
    assert tracker.tracked_norad_id is None


def test_manual_goto_switches_state_and_commands():
    rotator = FakeRotator()
    tracker = make_tracker(rotator)
    tracker.track(99999)
    tracker.manual_goto(123.0, 45.0)
    assert tracker.state == TrackerState.MANUAL
    assert tracker.tracked_norad_id is None
    assert rotator.commands[-1] == ("goto", 123.0, 45.0)


def test_park_commands_park_position():
    rotator = FakeRotator()
    tracker = make_tracker(rotator)
    tracker.park()
    assert rotator.commands[-1] == ("goto", 0.0, 90.0)
    assert tracker.state == TrackerState.IDLE


def test_command_applies_offset():
    rotator = FakeRotator(az_travel=(0.0, 360.0))
    tracker = make_tracker(rotator)
    tracker._az_offset = 360.0
    tracker._prev_continuous_az = -5.0
    tracker._command(357.0, 10.0)  # continuous: -3, commanded: 357
    cmd = rotator.commands[-1]
    assert cmd[0] == "goto"
    assert abs(cmd[1] - 357.0) < 1e-6
    assert cmd[2] == 10.0


def test_command_continuous_tracking_through_north():
    rotator = FakeRotator(az_travel=(-90.0, 450.0))
    tracker = make_tracker(rotator)
    tracker._az_offset = 0.0
    # Sweep through north: 350 -> 355 -> 2 -> 8; commands must not jump
    for az in [350.0, 355.0, 2.0, 8.0]:
        tracker._command(az, 20.0)
    commanded = [c[1] for c in rotator.commands]
    assert commanded == [350.0, 355.0, 362.0, 368.0]
