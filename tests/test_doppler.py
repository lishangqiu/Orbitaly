"""2 m band doppler readout: correction signs, rate, and band validation."""
from datetime import datetime, timezone

import pytest

from orbitaly.app import Services
from orbitaly.config import Config
from orbitaly.core.tle import Satellite

ISS_L1 = "1 25544U 98067A   20045.18587073  .00000950  00000-0  25302-4 0  9990"
ISS_L2 = "2 25544  51.6443 242.0161 0004885 264.6060 207.3845 15.49165514212791"
EPOCH = datetime(2020, 2, 14, 4, 27, 39, tzinfo=timezone.utc).timestamp()

C_M_S = 299_792_458.0


@pytest.fixture
def services():
    config = Config()
    config.tle.sources = []
    config.tle.cache_path = "/nonexistent/ignore.json"
    s = Services(config)
    sat = Satellite(25544, "ISS", ISS_L1, ISS_L2)
    with s.tle._lock:
        s.tle._satellites = {25544: sat}
    yield s
    s.rotator.close()


def test_inactive_when_not_tracking(services):
    readout = services.doppler_2m_readout(at=EPOCH)
    assert readout["active"] is False
    assert readout["band"] == "2m"
    assert readout["uplink_hz"] == 145_990_000.0
    assert readout["downlink_hz"] == 145_800_000.0


def test_correction_signs_are_opposite(services):
    services.tracker.track(25544)
    readout = services.doppler_2m_readout(at=EPOCH)
    assert readout["active"] is True
    down = readout["downlink_corrected_hz"] - readout["downlink_hz"]
    up = readout["uplink_corrected_hz"] - readout["uplink_hz"]
    # Approaching: RX tunes high, TX tunes low (and vice versa) — always opposite
    assert down * up < 0
    assert abs(down) == pytest.approx(abs(readout["downlink_shift_hz"]))


def test_shift_matches_range_rate(services):
    services.tracker.track(25544)
    readout = services.doppler_2m_readout(at=EPOCH)
    sat = services.tle.get(25544)
    obs = services.predictor.observe(sat, EPOCH)
    expected = -readout["downlink_hz"] * (obs.range_rate_km_s * 1000.0) / C_M_S
    assert readout["downlink_shift_hz"] == pytest.approx(expected, rel=1e-9)
    # 2 m LEO doppler stays within a few kHz
    assert abs(readout["downlink_shift_hz"]) < 4000.0


def test_rate_is_negative_through_pass(services):
    # Doppler shift decreases monotonically through a pass (approach -> recede),
    # so the observed-frequency rate is negative at any point along it.
    services.tracker.track(25544)
    readout = services.doppler_2m_readout(at=EPOCH)
    assert readout["downlink_rate_hz_s"] < 0
    # TX correction moves the other way
    assert readout["uplink_rate_hz_s"] > 0
    # LEO doppler rate at 145 MHz peaks around tens of Hz/s
    assert abs(readout["downlink_rate_hz_s"]) < 100.0


def test_set_frequencies_validated(services):
    services.set_doppler_2m(144_100_000.0, 145_900_000.0)
    assert services.doppler_2m_uplink_hz == 144_100_000.0
    with pytest.raises(ValueError, match="2 m band"):
        services.set_doppler_2m(435_000_000.0, 145_800_000.0)  # 70 cm uplink
    with pytest.raises(ValueError, match="2 m band"):
        services.set_doppler_2m(145_000_000.0, 143_999_999.0)
    # Failed set leaves previous values intact
    assert services.doppler_2m_uplink_hz == 144_100_000.0


def test_snapshot_includes_doppler(services):
    snap = services.status_snapshot()
    assert snap["doppler_2m"]["band"] == "2m"
