#!/usr/bin/env python3
"""Re-encode Natural Earth coastlines into the compact file the map draws.

    python scripts/make_world_json.py

Source
    Natural Earth 1:110m physical coastline, via the natural-earth-vector
    GeoJSON distribution:
    https://github.com/nvkelso/natural-earth-vector/blob/master/geojson/ne_110m_coastline.geojson

    Natural Earth is in the **public domain** (no permission needed, no
    attribution required, though it is offered here anyway). See
    https://www.naturalearthdata.com/about/terms-of-use/

Why this exists
    The console runs entirely offline from `static/` — no tiles, no CDN — the
    way a headless field station actually lives. So the coastlines ship as
    data in the repo rather than being fetched at runtime. This script is the
    provenance record: it is how `orbitaly/static/world.json` was produced,
    and running it again reproduces that file.

    110m is the right resolution for a ~900 px map; 50m doubles the weight for
    detail nobody can see at that scale. Douglas-Peucker at a tolerance below
    one pixel and 2-decimal coordinates (~1 km) throw away only what the
    canvas could not have drawn.
"""
from __future__ import annotations

import json
import sys
import urllib.request
from pathlib import Path

SOURCE_URL = (
    "https://raw.githubusercontent.com/nvkelso/natural-earth-vector"
    "/master/geojson/ne_110m_coastline.geojson"
)
OUTPUT = Path(__file__).resolve().parent.parent / "orbitaly" / "static" / "world.json"

#: Douglas-Peucker tolerance in degrees. A 900 px-wide equirectangular map is
#: 0.4°/px, so this is roughly a third of a pixel.
TOLERANCE_DEG = 0.12
#: 2 decimal places is about 1 km at the equator — a fortieth of a pixel.
PRECISION = 2


def simplify(points: list[list[float]], tolerance: float) -> list[list[float]]:
    """Douglas-Peucker, iterative so a long coastline cannot blow the stack."""
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        first, last = stack.pop()
        if last <= first + 1:
            continue
        worst_index, worst = -1, tolerance
        ax, ay = points[first]
        bx, by = points[last]
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        for i in range(first + 1, last):
            px, py = points[i]
            if span == 0.0:
                distance = ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
            else:
                # Perpendicular distance to the segment, clamped to its ends
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / span))
                qx, qy = ax + t * dx, ay + t * dy
                distance = ((px - qx) ** 2 + (py - qy) ** 2) ** 0.5
            if distance > worst:
                worst_index, worst = i, distance
        if worst_index >= 0:
            keep[worst_index] = True
            stack.append((first, worst_index))
            stack.append((worst_index, last))
    return [points[i] for i in range(len(points)) if keep[i]]


def clean(points: list[list[float]]) -> list[list[float]]:
    """Round to PRECISION and drop points that round onto their neighbour."""
    out: list[list[float]] = []
    for lon, lat in points:
        rounded = [round(lon, PRECISION), round(lat, PRECISION)]
        if not out or rounded != out[-1]:
            out.append(rounded)
    return out


def coastlines(geojson: dict) -> list[list[list[float]]]:
    """Every LineString in the collection, as lon/lat polylines."""
    lines: list[list[list[float]]] = []
    for feature in geojson.get("features", []):
        geometry = feature.get("geometry") or {}
        kind = geometry.get("type")
        if kind == "LineString":
            parts = [geometry["coordinates"]]
        elif kind == "MultiLineString":
            parts = geometry["coordinates"]
        else:
            continue
        for part in parts:
            simplified = clean(simplify([list(p[:2]) for p in part], TOLERANCE_DEG))
            if len(simplified) >= 2:
                lines.append(simplified)
    return lines


def main() -> int:
    print(f"fetching {SOURCE_URL}")
    with urllib.request.urlopen(SOURCE_URL, timeout=60) as response:
        geojson = json.load(response)

    lines = coastlines(geojson)
    points = sum(len(line) for line in lines)
    payload = {
        "source": "Natural Earth 1:110m physical coastline (public domain)",
        "url": SOURCE_URL,
        "generator": "scripts/make_world_json.py",
        "tolerance_deg": TOLERANCE_DEG,
        "lines": lines,
    }
    # No pretty-printing: this file is read by a canvas, not by a person.
    text = json.dumps(payload, separators=(",", ":"))
    OUTPUT.write_text(text)
    print(
        f"wrote {OUTPUT.relative_to(Path.cwd()) if OUTPUT.is_relative_to(Path.cwd()) else OUTPUT}"
        f": {len(lines)} polylines, {points} points, {len(text) / 1024:.1f} KB"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
