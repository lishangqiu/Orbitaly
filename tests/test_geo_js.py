"""Run the map's geometry helpers under node.

`static/geo.js` holds the projection, the antimeridian split and the
great-circle sampling — the first UI code in this project with real geometry
in it, and the place a map view is most likely to be quietly wrong. There is
no JS toolchain here and this feature is not the excuse to add one, so the
checks are plain assertions in `geo_checks.js` and pytest just runs them.

Skips when node is absent, the same way the hamlib client test skips when
`rotctl` is not installed: a missing tool is not a failure, but it must be
visible rather than silent.
"""
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import pytest

CHECKS = Path(__file__).parent / "geo_checks.js"
GEO_JS = Path(__file__).parent.parent / "orbitaly" / "static" / "geo.js"

needs_node = pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")


def run_geo(expression: str):
    """Evaluate an expression against geo.js under node and return its JSON."""
    script = f"const Geo = require({str(GEO_JS)!r}); console.log(JSON.stringify({expression}));"
    result = subprocess.run(
        ["node", "-e", script], capture_output=True, text=True, timeout=60, check=True
    )
    return json.loads(result.stdout)


@needs_node
def test_geo_js_checks_pass():
    result = subprocess.run(
        ["node", str(CHECKS)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        pytest.fail(
            "geo.js checks failed:\n"
            + result.stdout[-4000:]
            + "\n"
            + result.stderr[-4000:]
        )
    # Guard the guard: a checks file that silently ran nothing would otherwise
    # pass here forever.
    assert "checks passed" in result.stdout
    count = int(result.stdout.rsplit("\n", 2)[-2].split()[0])
    assert count >= 20, f"expected the full check set to run, saw {count}"


@needs_node
@pytest.mark.parametrize(
    "when",
    [
        datetime(2024, 3, 20, 6, 0, tzinfo=timezone.utc),   # near an equinox
        datetime(2024, 6, 20, 18, 0, tzinfo=timezone.utc),  # near a solstice
        datetime(2024, 12, 21, 0, 0, tzinfo=timezone.utc),
        datetime(2025, 9, 8, 11, 30, tzinfo=timezone.utc),
    ],
)
def test_the_terminator_agrees_with_the_ephemeris(when):
    """geo.js draws the terminator from low-precision almanac formulae.

    Cheap and dependency-free is the right trade for a line on a map, but
    "cheap" has to mean cheap and *correct*, so it is checked against the same
    JPL ephemeris the rest of the app uses for the eclipse flag.
    """
    from skyfield.api import load, wgs84

    unix = when.timestamp()
    lon_js, lat_js = run_geo(f"Geo.solarSubpoint({unix!r})")

    ts = load.timescale()
    eph = load("de421.bsp")
    t = ts.from_datetime(when)
    subpoint = wgs84.subpoint_of(eph["earth"].at(t).observe(eph["sun"]).apparent())

    assert lat_js == pytest.approx(subpoint.latitude.degrees, abs=0.02)
    # Compare the longitudes the short way round, so a wrap is not a failure.
    delta = (lon_js - subpoint.longitude.degrees + 180) % 360 - 180
    assert delta == pytest.approx(0.0, abs=0.02)
