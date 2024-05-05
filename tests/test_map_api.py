"""The HTTP surface the map view reads: catalog rows, ground tracks, station.

These go through the real app so the payload shape is checked where the
browser actually meets it, including the JSON encoding itself.
"""
import json

import pytest
from fastapi.testclient import TestClient

from orbitaly.app import create_app
from orbitaly.config import Config
from orbitaly.core.tle import Satellite

ISS_L1 = "1 25544U 98067A   20045.18587073  .00000950  00000-0  25302-4 0  9990"
ISS_L2 = "2 25544  51.6443 242.0161 0004885 264.6060 207.3845 15.49165514212791"

# SO-50: 70 cm downlink, so a 2 m station cannot hear it. The catalog must say so.
SO50_L1 = "1 27607U 02058C   20045.20800672  .00000075  00000-0  30982-4 0  9997"
SO50_L2 = "2 27607  64.5554 155.0459 0055800 268.7207  90.7607 14.75395396920671"

# A corrupt element set: the eccentricity field mangled to very nearly 1, so
# the orbit is effectively parabolic. SGP4 answers with NaN rather than an
# exception, and bare NaN is not valid JSON — one of these must not be able to
# break the whole catalog.
BROKEN_L1 = ISS_L1
BROKEN_L2 = "2 25544  51.6443 242.0161 9999999 264.6060 207.3845 15.49165514212791"


@pytest.fixture
def client():
    config = Config()
    config.tle.sources = []
    config.tle.cache_path = "/nonexistent/ignore.json"
    config.station.latitude, config.station.longitude = 45.515, -122.678
    app = create_app(config)
    services = app.state.services
    with services.tle._lock:
        services.tle._satellites = {
            25544: Satellite(25544, "ISS (ZARYA)", ISS_L1, ISS_L2),
            27607: Satellite(27607, "SO-50", SO50_L1, SO50_L2),
            99999: Satellite(99999, "BROKEN", BROKEN_L1, BROKEN_L2),
        }
        for sat in services.tle._satellites.values():
            sat.transponders = services.tle._transponders.get(sat.norad_id, [])
    with TestClient(app) as c:
        yield c


# -- catalog ----------------------------------------------------------------


def test_catalog_rows_carry_the_subpoint(client):
    rows = {s["norad_id"]: s for s in client.get("/api/satellites").json()["satellites"]}
    iss = rows[25544]
    assert -90.0 <= iss["latitude"] <= 90.0
    assert -180.0 <= iss["longitude"] <= 180.0
    assert 300.0 < iss["altitude_km"] < 500.0


def test_catalog_reports_the_band_gate(client):
    rows = {s["norad_id"]: s for s in client.get("/api/satellites").json()["satellites"]}
    # ISS is workable on 2 m through the APRS digipeater; SO-50 is not
    # workable at all here, however good the geometry looks.
    assert rows[25544]["band"] == "two_way"
    assert rows[27607]["band"] == "out_of_band"


def test_one_unpropagatable_tle_does_not_break_the_catalog(client):
    response = client.get("/api/satellites")
    assert response.status_code == 200
    # The browser's JSON.parse rejects bare NaN, so assert on the raw bytes
    # rather than on Python's more forgiving decoder.
    assert "NaN" not in response.text
    json.loads(response.text, parse_constant=_reject)
    ids = {s["norad_id"] for s in response.json()["satellites"]}
    assert {25544, 27607} <= ids
    assert 99999 not in ids


def _reject(name):
    raise AssertionError(f"payload contains {name}, which is not valid JSON")


# -- ground track -----------------------------------------------------------


def test_ground_track_defaults_to_the_satellites_own_orbit(client):
    body = client.get("/api/satellites/25544/groundtrack").json()
    assert body["norad_id"] == 25544
    assert body["period_s"] == pytest.approx(92.9 * 60.0, rel=0.02)
    # A quarter orbit behind, and at least one orbit ahead — plus however much
    # of the tail margin it took to reach a below-mask sample.
    span = body["points"][-1]["t"] - body["points"][0]["t"]
    quarter_and_one = body["period_s"] * 1.25
    assert quarter_and_one - 60.0 <= span <= quarter_and_one + 20 * 60.0
    assert body["step_s"] >= 60.0


def test_the_default_track_ends_below_the_horizon(client):
    """Where a projection stops is arbitrary; it should at least be somewhere
    the operator has no reason to be looking. The defect this prevents is a
    window that ends mid-pass, right where the map is brightest."""
    mask = client.get("/api/status").json()["station"]["mask_deg"]
    body = client.get("/api/satellites/25544/groundtrack").json()
    assert body["points"][-1]["el"] < mask


def test_an_explicit_window_is_left_exactly_where_the_caller_put_it(client):
    """...which is also how the mid-pass terminus is reproduced in a browser."""
    body = client.get(
        "/api/satellites/25544/groundtrack",
        params={"minutes_behind": 20, "minutes_ahead": 100},
    ).json()
    span = body["points"][-1]["t"] - body["points"][0]["t"]
    assert span == pytest.approx(120 * 60.0, abs=body["step_s"])


def test_ground_track_now_index_splits_behind_from_ahead(client):
    body = client.get("/api/satellites/25544/groundtrack").json()
    points, i = body["points"], body["now_index"]
    assert 0 < i < len(points) - 1
    times = [p["t"] for p in points]
    assert times == sorted(times)
    # A quarter of the samples sit behind the marker, three quarters ahead
    assert i / len(points) == pytest.approx(0.2, abs=0.05)


def test_ground_track_points_are_complete_and_finite(client):
    body = client.get("/api/satellites/25544/groundtrack").json()
    for p in body["points"]:
        assert set(p) == {"t", "lat", "lon", "alt_km", "el"}
        assert -90.0 <= p["lat"] <= 90.0
        assert -180.0 <= p["lon"] <= 180.0
        assert -90.0 <= p["el"] <= 90.0


def test_ground_track_honours_an_explicit_window(client):
    body = client.get(
        "/api/satellites/25544/groundtrack",
        params={"minutes_behind": 10, "minutes_ahead": 20, "step_s": 60},
    ).json()
    span = body["points"][-1]["t"] - body["points"][0]["t"]
    assert span == pytest.approx(1800.0, abs=60.0)
    assert body["step_s"] == pytest.approx(60.0)


def test_ground_track_caps_its_own_sample_count(client):
    """A one-second step over a day must not return 86,400 points."""
    body = client.get(
        "/api/satellites/25544/groundtrack",
        params={"minutes_behind": 0, "minutes_ahead": 1440, "step_s": 1},
    ).json()
    assert len(body["points"]) <= 500
    assert body["step_s"] > 1.0  # widened, not truncated
    span = body["points"][-1]["t"] - body["points"][0]["t"]
    assert span == pytest.approx(1440 * 60.0, rel=0.01)


def test_ground_track_refuses_a_tle_that_will_not_propagate(client):
    assert client.get("/api/satellites/99999/groundtrack").status_code == 422


def test_ground_track_unknown_satellite(client):
    assert client.get("/api/satellites/12345/groundtrack").status_code == 404


@pytest.mark.parametrize(
    "params",
    [
        {"minutes_ahead": -5},
        {"minutes_behind": 5000},
        {"step_s": 0},
        {"minutes_behind": 0, "minutes_ahead": 0},
    ],
)
def test_ground_track_rejects_nonsense_windows(client, params):
    assert client.get("/api/satellites/25544/groundtrack", params=params).status_code == 422


# -- station ----------------------------------------------------------------


def test_status_exposes_what_the_map_needs_to_draw_the_ring(client):
    station = client.get("/api/status").json()["station"]
    assert station["latitude"] == 45.515
    assert station["longitude"] == -122.678
    assert station["mask_deg"] >= station["min_workable_elevation_deg"]
    assert station["altitude_m"] == 300.0
    assert station["bands"] == [
        {"min_hz": 144_000_000.0, "max_hz": 148_000_000.0, "tx": True, "rx": True}
    ]


def test_mask_is_never_below_what_the_tracker_will_schedule(client):
    status = client.get("/api/status").json()
    assert status["station"]["mask_deg"] >= 5.0
