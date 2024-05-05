"""Ground tracks, the band gate, and the elevation mask.

The ground-track path is vectorized and the catalog path is hoisted, so the
tests that matter most are the ones asserting they still produce exactly what
the plain per-sample `observe()` produces. An optimization that quietly
changes the numbers is worse than no optimization.
"""
from datetime import datetime, timezone

import pytest

from orbitaly.config import BandConfig, Config, StationConfig
from orbitaly.core.bands import (
    OUT_OF_BAND,
    RX_ONLY,
    TWO_WAY,
    UNKNOWN,
    band_status,
    effective_mask_deg,
    is_workable,
)
from orbitaly.core.predictor import (
    MAX_GROUND_TRACK_SAMPLES,
    Predictor,
    default_ground_track_window,
    trim_to_below_mask,
)
from orbitaly.core.tle import Satellite

ISS_L1 = "1 25544U 98067A   20045.18587073  .00000950  00000-0  25302-4 0  9990"
ISS_L2 = "2 25544  51.6443 242.0161 0004885 264.6060 207.3845 15.49165514212791"

# A synthetic element set standing in for IO-117 (GreenCube): mean motion
# 6.4640 rev/day puts it at ~5,800 km and a ~3.7 h period. Not the real
# current TLE — it is here for its *shape*, as the plan's worked example of a
# satellite that a hardcoded "one orbit is ~100 minutes" window would clip.
IO117_L1 = "1 53106U 22077E   24045.51782407  .00000079  00000-0  00000-0 0  9993"
IO117_L2 = "2 53106  70.1408 106.2555 0011983 249.4324 110.5410  6.46400068 88135"
IO117_PERIOD_H = 24.0 / 6.4640

EPOCH = datetime(2020, 2, 14, 4, 27, 39, tzinfo=timezone.utc).timestamp()


@pytest.fixture
def iss():
    return Satellite(25544, "ISS (ZARYA)", ISS_L1, ISS_L2)


@pytest.fixture
def io117():
    return Satellite(53106, "IO-117 (GREENCUBE)", IO117_L1, IO117_L2)


@pytest.fixture
def predictor():
    return Predictor(StationConfig(latitude=45.515, longitude=-122.678, altitude_m=15))


# -- ground_track -----------------------------------------------------------


def test_ground_track_matches_observe_at_the_same_instants(predictor, iss):
    """The vectorized path is only worth having if it is the same math."""
    points = predictor.ground_track(iss, EPOCH, EPOCH + 1800.0, step_s=60.0)
    assert len(points) == 31
    for p in points:
        reference = predictor.observe(iss, p.time)
        assert p.latitude == pytest.approx(reference.latitude, abs=1e-9)
        assert p.longitude == pytest.approx(reference.longitude, abs=1e-9)
        assert p.altitude_km == pytest.approx(reference.altitude_km, abs=1e-9)
        # The elevation the map highlights "in view" with must be the same
        # elevation the tracker and the pass list are using.
        assert p.elevation == pytest.approx(reference.elevation, abs=1e-9)


def test_ground_track_samples_the_requested_window(predictor, iss):
    points = predictor.ground_track(iss, EPOCH, EPOCH + 600.0, step_s=60.0)
    assert points[0].time == pytest.approx(EPOCH)
    assert points[-1].time == pytest.approx(EPOCH + 600.0)
    steps = [b.time - a.time for a, b in zip(points, points[1:])]
    assert all(s == pytest.approx(60.0) for s in steps)


def test_ground_track_stays_on_the_globe(predictor, iss):
    points = predictor.ground_track(iss, EPOCH, EPOCH + 5400.0, step_s=60.0)
    assert all(-90.0 <= p.latitude <= 90.0 for p in points)
    assert all(-180.0 <= p.longitude <= 180.0 for p in points)
    assert all(300.0 < p.altitude_km < 500.0 for p in points)
    # An ISS orbit crosses the antimeridian, so the raw longitudes must
    # contain the wrap the renderer has to split on. If they never did, the
    # split helper would be untested by construction.
    jumps = [
        abs(b.longitude - a.longitude) for a, b in zip(points, points[1:])
    ]
    assert max(jumps) > 180.0


def test_ground_track_widens_the_step_rather_than_truncating(predictor, iss):
    """A huge window must still cover the window, just more coarsely."""
    span = 24 * 3600.0
    points = predictor.ground_track(iss, EPOCH, EPOCH + span, step_s=1.0)
    assert len(points) <= MAX_GROUND_TRACK_SAMPLES
    assert points[0].time == pytest.approx(EPOCH)
    assert points[-1].time == pytest.approx(EPOCH + span, abs=span / MAX_GROUND_TRACK_SAMPLES)


def test_ground_track_empty_window(predictor, iss):
    assert predictor.ground_track(iss, EPOCH, EPOCH) == []
    assert predictor.ground_track(iss, EPOCH, EPOCH - 60.0) == []


# -- window scaling ---------------------------------------------------------


def test_orbital_period_is_read_from_the_tle(predictor, iss, io117):
    assert predictor.orbital_period_s(iss) == pytest.approx(92.9 * 60.0, rel=0.02)
    assert predictor.orbital_period_s(io117) == pytest.approx(IO117_PERIOD_H * 3600.0, rel=0.02)
    # The altitude that sets the coverage-ring radius follows from it.
    assert predictor.observe(io117, EPOCH).altitude_km == pytest.approx(5800.0, rel=0.05)


def test_window_follows_the_orbit_not_a_fixed_number_of_minutes(predictor, iss, io117):
    """A LEO window would show less than half of IO-117's orbit."""
    leo = default_ground_track_window(predictor.orbital_period_s(iss))
    heo = default_ground_track_window(predictor.orbital_period_s(io117))
    leo_ahead, leo_step = leo.ahead_s, leo.step_s
    heo_ahead, heo_step = heo.ahead_s, heo.step_s
    assert heo_ahead > 2 * leo_ahead
    # Step scales with the period, so covering a whole high orbit does not
    # cost multiples of the samples a LEO bird's orbit costs.
    assert heo_step > leo_step
    leo_samples, heo_samples = leo_ahead / leo_step, heo_ahead / heo_step
    assert heo_samples <= 1.5 * leo_samples
    # At a fixed 60 s step — the LEO-shaped assumption — it would.
    assert heo_ahead / 60.0 > 2 * leo_samples


def test_window_covers_a_whole_orbit_of_io117(predictor, io117):
    window = default_ground_track_window(predictor.orbital_period_s(io117))
    points = predictor.ground_track(
        io117, EPOCH - window.behind_s, EPOCH + window.ahead_s, window.step_s
    )
    assert len(points) <= MAX_GROUND_TRACK_SAMPLES
    # A full orbit of a 70-degree-inclination bird sweeps most of its
    # latitude range; a clipped window would not.
    lats = [p.latitude for p in points]
    assert max(lats) > 60.0 and min(lats) < -60.0


def test_window_is_capped_for_absurd_periods():
    window = default_ground_track_window(30 * 24 * 3600.0)
    assert window.ahead_s == 6 * 3600.0
    # The tail margin is inside the cap, not added on top of it.
    assert window.trim_after_s == 6 * 3600.0


def test_window_falls_back_when_the_period_is_unusable():
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        window = default_ground_track_window(bad)
        assert window.behind_s > 0 and window.ahead_s > 0 and window.step_s >= 60.0
        assert window.trim_after_s > 0


# -- where the projection stops ---------------------------------------------

MASK = 5.0


def test_the_window_is_sampled_past_one_orbit_so_the_tail_can_be_trimmed(predictor, iss):
    window = default_ground_track_window(predictor.orbital_period_s(iss))
    assert window.ahead_s > window.trim_after_s
    assert window.trim_after_s == pytest.approx(predictor.orbital_period_s(iss))


def test_a_track_whose_window_ends_mid_pass_is_trimmed_below_the_mask(predictor, iss):
    """The reported defect, reproduced: the full-weight in-view line stopped
    dead in open air because the *window* stopped in the middle of a pass.

    One period ahead the subpoint has moved ~24 degrees west, so for an
    operator watching a pass come up the terminus lands right beside the
    station — which is exactly when someone is looking at the map.
    """
    period = predictor.orbital_period_s(iss)
    best = max(
        predictor.next_passes(iss, EPOCH, hours=24.0, min_elevation=0.0),
        key=lambda p: p.max_elevation,
    )
    now = best.tca - period + 240.0  # so now + one period sits 4 min past TCA
    window = default_ground_track_window(period)
    points = predictor.ground_track(
        iss, now - window.behind_s, now + window.ahead_s, window.step_s
    )

    at_one_period = [p for p in points if p.time <= now + window.trim_after_s]
    assert at_one_period[-1].elevation > MASK, "scenario is not reproducing the defect"

    trimmed = trim_to_below_mask(points, now + window.trim_after_s, MASK)
    assert trimmed[-1].elevation < MASK
    # A whole orbit is still drawn: the margin is what gets given up, never the
    # orbit itself.
    assert trimmed[-1].time >= now + window.trim_after_s
    assert len(trimmed) < len(points)
    assert trimmed == points[: len(trimmed)]


def test_trimming_stops_at_the_first_below_mask_sample_not_the_lowest(predictor, iss):
    points = predictor.ground_track(iss, EPOCH, EPOCH + 3600.0, step_s=60.0)
    trimmed = trim_to_below_mask(points, EPOCH, MASK)
    assert trimmed[-1].elevation < MASK
    assert all(p.elevation >= MASK for p in trimmed[:-1])


def test_a_satellite_that_never_sets_keeps_its_whole_window(predictor, iss):
    """No below-mask sample to trim back to: return the window untouched and
    let the client's fading tail cover an arbitrary end, as it was built to."""
    points = predictor.ground_track(iss, EPOCH, EPOCH + 3600.0, step_s=60.0)
    assert trim_to_below_mask(points, EPOCH, -90.0) is points
    # Nor may it cut into the orbit the caller asked for: with nothing past the
    # boundary to trim to, the whole window survives.
    assert trim_to_below_mask(points, points[-1].time, MASK) == points


# -- observe_many -----------------------------------------------------------


def test_observe_many_is_identical_to_observing_one_at_a_time(predictor, iss, io117):
    """The catalog poll's speedup must be free of numerical cost."""
    sats = [iss, io117]
    many = predictor.observe_many(sats, EPOCH)
    assert set(many) == {iss.norad_id, io117.norad_id}
    for sat in sats:
        one, batch = predictor.observe(sat, EPOCH), many[sat.norad_id]
        for f in (
            "azimuth",
            "elevation",
            "range_km",
            "range_rate_km_s",
            "latitude",
            "longitude",
            "altitude_km",
            "footprint_km",
        ):
            assert getattr(batch, f) == pytest.approx(getattr(one, f), abs=1e-9), f
        assert batch.sunlit == one.sunlit


def test_updated_elements_are_not_served_from_the_cache(predictor, iss):
    """Line 2 carries the orbital elements, so it has to be part of the key."""
    before = predictor.observe(iss, EPOCH)
    nudged = Satellite(
        iss.norad_id,
        iss.name,
        iss.line1,
        # same epoch, different inclination and RAAN
        "2 25544  51.6443 100.0000 0004885 264.6060 207.3845 15.49165514212791",
    )
    after = predictor.observe(nudged, EPOCH)
    assert after.longitude != before.longitude


def test_observe_many_skips_a_satellite_that_will_not_propagate(predictor, iss):
    """One bad element set must not blank the whole catalog."""
    broken = Satellite(99999, "BROKEN", ISS_L1[:20] + "?" * 49, ISS_L2)
    result = predictor.observe_many([iss, broken], EPOCH)
    assert iss.norad_id in result
    assert broken.norad_id not in result


# -- band gate --------------------------------------------------------------

TWO_M = [BandConfig(min_hz=144e6, max_hz=148e6, tx=True, rx=True)]
TWO_M_RX_ONLY = [BandConfig(min_hz=144e6, max_hz=148e6, tx=False, rx=True)]

# The plan's worked examples, from the real transponder database.
ISS_APRS = {"name": "APRS", "uplink_hz": 145_825_000, "downlink_hz": 145_825_000}
ISS_VOICE = {"name": "FM Voice", "uplink_hz": 145_990_000, "downlink_hz": 437_800_000}
SO50 = {"name": "SO-50", "uplink_hz": 145_850_000, "downlink_hz": 436_795_000}
AO91 = {"name": "AO-91", "uplink_hz": 435_250_000, "downlink_hz": 145_960_000}
SSTV = {"name": "SSTV", "uplink_hz": None, "downlink_hz": 145_800_000}


def test_so50_is_not_workable_on_a_2m_station():
    """The defect the band gate exists to prevent: SO-50 downlinks on 70 cm."""
    assert band_status([SO50], TWO_M) == OUT_OF_BAND
    assert not is_workable(band_status([SO50], TWO_M))


def test_ao91_downlink_is_receivable_but_its_uplink_is_not():
    assert band_status([AO91], TWO_M) == RX_ONLY
    assert is_workable(band_status([AO91], TWO_M))


def test_iss_is_two_way_through_the_aprs_digipeater():
    assert band_status([ISS_VOICE, ISS_APRS, SSTV], TWO_M) == TWO_WAY


def test_two_way_is_judged_per_transponder_not_per_satellite():
    """A reachable downlink on one transponder and a reachable uplink on
    another does not make either one workable both ways."""
    downlink_only = {"name": "A", "uplink_hz": 435_000_000, "downlink_hz": 145_900_000}
    uplink_only = {"name": "B", "uplink_hz": 145_900_000, "downlink_hz": 435_000_000}
    assert band_status([downlink_only, uplink_only], TWO_M) == RX_ONLY


def test_a_receive_only_station_is_never_told_it_can_transmit():
    assert band_status([ISS_APRS], TWO_M_RX_ONLY) == RX_ONLY


def test_a_downlink_only_beacon_is_receivable():
    assert band_status([SSTV], TWO_M) == RX_ONLY


def test_no_transponder_data_makes_no_claim():
    assert band_status([], TWO_M) == UNKNOWN
    assert not is_workable(UNKNOWN)


def test_band_edges_are_inclusive():
    edge = {"name": "edge", "uplink_hz": 144_000_000, "downlink_hz": 148_000_000}
    assert band_status([edge], TWO_M) == TWO_WAY
    outside = {"name": "out", "uplink_hz": 144_000_000, "downlink_hz": 148_000_001}
    assert band_status([outside], TWO_M) == OUT_OF_BAND


# -- config surface ---------------------------------------------------------


def test_bands_default_to_the_2m_panels_range():
    bands = StationConfig().band_list()
    assert len(bands) == 1
    assert bands[0].contains(145_800_000)
    assert not bands[0].contains(437_800_000)


def test_bands_accept_yaml_mappings():
    station = StationConfig(bands=[{"min_hz": 430e6, "max_hz": 440e6, "tx": False}])
    bands = station.band_list()
    assert bands[0].contains(435_000_000) and bands[0].rx and not bands[0].tx


def test_bands_reject_junk():
    with pytest.raises(ValueError):
        StationConfig(bands=[145_800_000]).band_list()


def test_mask_never_promises_a_pass_the_tracker_would_refuse():
    config = Config()
    config.station.min_workable_elevation_deg = 2.0
    config.tracker.min_elevation_deg = 10.0
    assert effective_mask_deg(config) == 10.0
    config.station.min_workable_elevation_deg = 15.0
    assert effective_mask_deg(config) == 15.0
