"""Sanity checks for orbital math against a fixed historical ISS TLE."""
from datetime import datetime, timezone

import pytest

from orbitaly.config import StationConfig
from orbitaly.core.predictor import Predictor
from orbitaly.core.tle import Satellite, parse_tle_text

ISS_NAME = "ISS (ZARYA)"
ISS_L1 = "1 25544U 98067A   20045.18587073  .00000950  00000-0  25302-4 0  9990"
ISS_L2 = "2 25544  51.6443 242.0161 0004885 264.6060 207.3845 15.49165514212791"

EPOCH = datetime(2020, 2, 14, 4, 27, 39, tzinfo=timezone.utc).timestamp()


@pytest.fixture
def sat():
    return Satellite(25544, ISS_NAME, ISS_L1, ISS_L2)


@pytest.fixture
def predictor():
    return Predictor(StationConfig(latitude=40.44, longitude=-79.99, altitude_m=300))


def test_observation_ranges(predictor, sat):
    obs = predictor.observe(sat, EPOCH)
    assert 0.0 <= obs.azimuth < 360.0
    assert -90.0 <= obs.elevation <= 90.0
    assert 300.0 < obs.range_km < 20000.0  # ISS is in LEO
    assert 300.0 < obs.altitude_km < 500.0
    assert abs(obs.range_rate_km_s) < 8.0
    assert -90.0 <= obs.latitude <= 90.0
    assert 1000.0 < obs.footprint_km < 3000.0


def test_doppler_magnitude(predictor, sat):
    obs = predictor.observe(sat, EPOCH)
    shift = obs.doppler_shift_hz(145_800_000)
    # LEO doppler at 2 m band stays within a few kHz
    assert abs(shift) < 4000.0
    assert abs(obs.doppler_factor - 1.0) < 1e-4


def test_pass_prediction(predictor, sat):
    passes = predictor.next_passes(sat, EPOCH, hours=24.0, min_elevation=0.0)
    assert len(passes) >= 3  # ISS passes a mid-latitude station several times a day
    for p in passes:
        assert p.aos <= p.tca <= p.los
        assert p.duration_s < 20 * 60  # LEO passes are short
        assert p.max_elevation > 0.0
        assert len(p.profile) >= 2
        assert p.profile[0].time == pytest.approx(p.aos)
        # Elevation at AOS/LOS is near the horizon
        assert abs(p.profile[0].elevation) < 5.0
        assert abs(p.profile[-1].elevation) < 5.0
    # Chronological, non-overlapping
    for a, b in zip(passes, passes[1:]):
        assert a.los <= b.aos


def test_pass_elevation_filter(predictor, sat):
    all_passes = predictor.next_passes(sat, EPOCH, hours=24.0, min_elevation=0.0)
    high_passes = predictor.next_passes(sat, EPOCH, hours=24.0, min_elevation=30.0)
    assert len(high_passes) <= len(all_passes)


def test_parse_tle_3le():
    text = f"{ISS_NAME}\n{ISS_L1}\n{ISS_L2}\n"
    sats = parse_tle_text(text)
    assert sats[25544].name == ISS_NAME
    assert sats[25544].line1 == ISS_L1


def test_parse_tle_2le():
    sats = parse_tle_text(f"{ISS_L1}\n{ISS_L2}\n")
    assert 25544 in sats
    assert sats[25544].name == "NORAD 25544"


def test_parse_tle_garbage_ignored():
    text = f"junk line\nanother\n{ISS_NAME}\n{ISS_L1}\n{ISS_L2}\ntrailing"
    sats = parse_tle_text(text)
    assert list(sats) == [25544]
