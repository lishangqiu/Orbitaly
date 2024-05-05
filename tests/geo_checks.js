/* Checks for static/geo.js, run under node by tests/test_geo_js.py.
 *
 * Plain assertions, no framework — the project has no JS toolchain and this
 * is not the feature that should introduce one. Each check prints a line so a
 * failure names itself.
 */
"use strict";

const assert = require("assert");
const path = require("path");
const Geo = require(path.join(__dirname, "..", "orbitaly", "static", "geo.js"));

let checks = 0;
function check(name, fn) {
  fn();
  checks++;
  console.log("ok - " + name);
}

const close = (a, b, tol, what) =>
  assert.ok(Math.abs(a - b) <= tol, `${what || ""}: ${a} != ${b} (tol ${tol})`);

/* -- projection -------------------------------------------------------- */

check("projection maps the corners of the world to the corners of the canvas", () => {
  assert.deepStrictEqual(Geo.project(-180, 90, 900, 450), [0, 0]);
  assert.deepStrictEqual(Geo.project(180, -90, 900, 450), [900, 450]);
  assert.deepStrictEqual(Geo.project(0, 0, 900, 450), [450, 225]);
});

check("projection is linear in longitude and inverted in latitude", () => {
  const [x1, y1] = Geo.project(-90, 45, 900, 450);
  const [x2, y2] = Geo.project(90, -45, 900, 450);
  close(x1, 225, 1e-9);
  close(x2, 675, 1e-9);
  assert.ok(y1 < y2, "north must be up");
});

/* -- longitude wrapping ------------------------------------------------ */

check("wrapLon normalizes onto [-180, 180)", () => {
  close(Geo.wrapLon(0), 0, 1e-9);
  close(Geo.wrapLon(-179.5), -179.5, 1e-9);
  close(Geo.wrapLon(190), -170, 1e-9);
  close(Geo.wrapLon(-190), 170, 1e-9);
  close(Geo.wrapLon(540), -180, 1e-9);
  close(Geo.wrapLon(360), 0, 1e-9);
});

/* -- antimeridian ------------------------------------------------------ */

check("a track that does not wrap is left as one polyline", () => {
  const line = [[-10, 0], [0, 5], [10, 10]];
  const parts = Geo.splitAntimeridian(line);
  assert.strictEqual(parts.length, 1);
  assert.deepStrictEqual(parts[0], line);
});

check("an eastward crossing splits at +180 / -180", () => {
  const parts = Geo.splitAntimeridian([[170, 0], [179, 10], [-179, 20], [-170, 30]]);
  assert.strictEqual(parts.length, 2);
  const first = parts[0];
  const second = parts[1];
  close(first[first.length - 1][0], 180, 1e-9, "first part ends at +180");
  close(second[0][0], -180, 1e-9, "second part starts at -180");
  // Crossing is halfway between 179 and -179, so halfway between lat 10 and 20
  close(first[first.length - 1][1], 15, 1e-9, "crossing latitude");
  close(second[0][1], 15, 1e-9, "crossing latitude matches on both edges");
});

check("a westward crossing splits at -180 / +180", () => {
  const parts = Geo.splitAntimeridian([[-170, 0], [-179, 10], [179, 20], [170, 30]]);
  assert.strictEqual(parts.length, 2);
  close(parts[0][parts[0].length - 1][0], -180, 1e-9);
  close(parts[1][0][0], 180, 1e-9);
  close(parts[0][parts[0].length - 1][1], 15, 1e-9);
});

check("no split leaves a segment spanning more than half the world", () => {
  // The actual defect being prevented: a horizontal streak across the map.
  const track = [];
  for (let lon = 100; lon <= 460; lon += 7) track.push([Geo.wrapLon(lon), 20]);
  for (const part of Geo.splitAntimeridian(track)) {
    for (let i = 1; i < part.length; i++) {
      assert.ok(
        Math.abs(part[i][0] - part[i - 1][0]) <= 180,
        `segment spans ${Math.abs(part[i][0] - part[i - 1][0])} degrees of longitude`
      );
    }
  }
});

check("multiple crossings each split", () => {
  // Three times around the world crosses the antimeridian three times, so the
  // track comes back as four pieces.
  const track = [];
  for (let lon = 0; lon <= 1080; lon += 30) track.push([Geo.wrapLon(lon), 0]);
  assert.strictEqual(Geo.splitAntimeridian(track).length, 4);
});

check("extra fields per point survive a split", () => {
  // Ground-track samples carry their elevation, which is what decides whether
  // an arc renders as in-view. It must not be dropped at the frame edge.
  const parts = Geo.splitAntimeridian([[179, 10, 42], [-179, 20, 44]]);
  assert.strictEqual(parts.length, 2);
  assert.strictEqual(parts[0][0][2], 42);
  assert.strictEqual(parts[0][1][2], 42, "edge point inherits the sample before it");
  assert.strictEqual(parts[1][0][2], 44, "edge point inherits the sample after it");
});

check("degenerate inputs do not throw", () => {
  assert.deepStrictEqual(Geo.splitAntimeridian([]), []);
  assert.deepStrictEqual(Geo.splitAntimeridian([[0, 0]]), []);
  assert.strictEqual(Geo.splitAntimeridian([[0, 0], [1, 1]]).length, 1);
});

/* -- interpolation ----------------------------------------------------- */

check("a point interpolated along a segment carries every trailing field", () => {
  const p = Geo.interpolatePoint([0, 0, 10, 1000, 400], [10, 20, 30, 1060, 420], 0.5);
  close(p[0], 5, 1e-9, "lon");
  close(p[1], 10, 1e-9, "lat");
  close(p[2], 20, 1e-9, "elevation");
  close(p[3], 1030, 1e-9, "time");
  close(p[4], 410, 1e-9, "altitude");
});

check("interpolation crosses the antimeridian the short way", () => {
  // 179 -> -179 is two degrees of motion, not 358. Getting this wrong puts an
  // interpolated point on the far side of the world.
  close(Geo.interpolatePoint([179, 0], [-179, 0], 0.5)[0], -180, 1e-9);
  close(Geo.interpolatePoint([-179, 0], [179, 0], 0.5)[0], -180, 1e-9);
  close(Geo.interpolatePoint([170, 0], [-175, 0], 0.5)[0], 177.5, 1e-9);
});

check("a non-numeric trailing field is carried, not turned into NaN", () => {
  assert.strictEqual(Geo.interpolatePoint([0, 0, 5, 1, null], [2, 2, 7, 3, 9], 0.5)[4], null);
});

/* -- clipping runs ----------------------------------------------------- */

/* A synthetic pass: below the mask, up over it, and back down. */
const PASS = [
  [0, 0, -10, 0],
  [10, 0, -2, 60],
  [20, 0, 6, 120],
  [30, 0, 20, 180],
  [40, 0, 6, 240],
  [50, 0, -2, 300],
  [60, 0, -10, 360],
];

check("an in-view run begins and ends exactly on the mask crossing", () => {
  // Snapping to whole samples instead would start the 3 px highlight up to a
  // full sample — ~450 km of ground at LEO — from the crossing it marks.
  const runs = Geo.clipRunsAtMask(PASS, 5);
  assert.strictEqual(runs.length, 1);
  const run = runs[0];
  close(run[0][2], 5, 1e-12, "entry elevation is the mask");
  close(run[run.length - 1][2], 5, 1e-12, "exit elevation is the mask");
  // (5 - -2) / (6 - -2) of the way from lon 10 to lon 20
  close(run[0][0], 18.75, 1e-9, "entry longitude");
  close(run[run.length - 1][0], 41.25, 1e-9, "exit longitude");
  close(run[0][3], 112.5, 1e-9, "entry time is interpolated too");
});

check("a clipped run stays inside the polyline it came from", () => {
  for (const mask of [-20, -5, 0, 5, 15, 19.9]) {
    for (const run of Geo.clipRunsAtMask(PASS, mask)) {
      for (const p of run) {
        assert.ok(p[0] >= 0 && p[0] <= 60, `longitude ${p[0]} outside the track`);
        assert.ok(p[2] >= mask - 1e-9, `elevation ${p[2]} below the mask`);
        assert.ok(p[3] >= 0 && p[3] <= 360, `time ${p[3]} outside the track`);
      }
      const times = run.map((p) => p[3]);
      assert.deepStrictEqual(times, [...times].sort((a, b) => a - b), "run is ordered");
    }
  }
});

check("a mask nothing reaches gives no runs, one everything clears gives one", () => {
  assert.deepStrictEqual(Geo.clipRunsAtMask(PASS, 25), []);
  const all = Geo.clipRunsAtMask(PASS, -90);
  assert.strictEqual(all.length, 1);
  assert.deepStrictEqual(all[0], PASS, "no crossing to interpolate: the track itself");
});

check("two separate passes clip to two separate runs", () => {
  const two = PASS.concat([
    [70, 0, 8, 420],
    [80, 0, 2, 480],
  ]);
  assert.strictEqual(Geo.clipRunsAtMask(two, 5).length, 2);
});

check("a crossing over the antimeridian lands on the right side of the world", () => {
  const runs = Geo.clipRunsAtMask(
    [[160, 0, -10, 0], [170, 0, 0, 60], [-175, 0, 10, 120], [-165, 0, 20, 180]],
    5
  );
  assert.strictEqual(runs.length, 1);
  close(runs[0][0][0], 177.5, 1e-9, "crossing longitude, the short way round");
  close(runs[0][0][2], 5, 1e-12);
  // ... and the run still splits at the frame edge rather than streaking back
  for (const part of Geo.splitAntimeridian(runs[0])) {
    for (let i = 1; i < part.length; i++) {
      assert.ok(Math.abs(part[i][0] - part[i - 1][0]) <= 180);
    }
  }
});

check("a scheduled window clips at its AOS and LOS, not at the nearest sample", () => {
  const runs = Geo.clipRunsInWindow(PASS, 90, 270);
  assert.strictEqual(runs.length, 1);
  const run = runs[0];
  close(run[0][3], 90, 1e-12, "starts at AOS");
  close(run[run.length - 1][3], 270, 1e-12, "ends at LOS");
  close(run[0][0], 15, 1e-9, "halfway between the samples either side of AOS");
  close(run[run.length - 1][0], 45, 1e-9);
});

check("a window shorter than one sample step still draws", () => {
  // Half a minute inside a 60 s step: whole-sample filtering dropped this
  // entirely, un-drawing a window the station really had committed to.
  const runs = Geo.clipRunsInWindow(PASS, 70, 100);
  assert.strictEqual(runs.length, 1);
  close(runs[0][0][3], 70, 1e-12);
  close(runs[0][1][3], 100, 1e-12);
});

check("clipping degenerate inputs does not throw", () => {
  assert.deepStrictEqual(Geo.clipRunsAtMask([], 5), []);
  assert.deepStrictEqual(Geo.clipRunsAtMask([[0, 0, 90, 0]], 5), []);
  assert.deepStrictEqual(Geo.clipRunsInWindow(PASS, 1000, 2000), []);
});

/* -- visibility circle ------------------------------------------------- */

check("visibility radius matches the plan's worked numbers", () => {
  // psi = acos( (Re/(Re+h)) cos eps ) - eps, at a 5 degree mask
  close(Geo.visibilityRadiusKm(420, 5), 1760, 15, "ISS altitude");
  close(Geo.visibilityRadiusKm(550, 5), 2060, 15, "typical smallsat");
  close(Geo.visibilityRadiusKm(800, 5), 2540, 20, "many amateur birds");
  close(Geo.visibilityRadiusKm(5800, 5), 5900, 60, "IO-117");
});

check("a lower horizon reaches further", () => {
  assert.ok(Geo.visibilityRadiusKm(500, 0) > Geo.visibilityRadiusKm(500, 5));
  assert.ok(Geo.visibilityRadiusKm(500, 5) > Geo.visibilityRadiusKm(500, 20));
});

check("a zero mask gives the satellite's full footprint", () => {
  // With eps = 0 the formula reduces to the horizon circle acos(Re/(Re+h)).
  const h = 420;
  const expected = Geo.EARTH_RADIUS_KM * Math.acos(Geo.EARTH_RADIUS_KM / (Geo.EARTH_RADIUS_KM + h));
  close(Geo.visibilityRadiusKm(h, 0), expected, 1e-6);
});

check("impossible geometry collapses to nothing rather than NaN", () => {
  assert.strictEqual(Geo.visibilityRadiusKm(0, 5), 0);
  assert.strictEqual(Geo.visibilityRadiusKm(-100, 5), 0);
  assert.ok(Number.isFinite(Geo.visibilityRadiusKm(400, 90)));
  assert.strictEqual(Geo.visibilityRadiusKm(400, 90), 0);
});

/* -- great circles ----------------------------------------------------- */

check("every point of a circle is the requested distance from its centre", () => {
  const lat0 = 45.515;
  const lon0 = -122.678;
  const radius = 2000;
  const haversine = (aLat, aLon, bLat, bLon) => {
    const d2r = Math.PI / 180;
    const dLat = (bLat - aLat) * d2r;
    const dLon = (bLon - aLon) * d2r;
    const s =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(aLat * d2r) * Math.cos(bLat * d2r) * Math.sin(dLon / 2) ** 2;
    return 2 * Geo.EARTH_RADIUS_KM * Math.asin(Math.sqrt(s));
  };
  for (const p of Geo.greatCircle(lat0, lon0, radius, 72)) {
    close(haversine(lat0, lon0, p[1], p[0]), radius, 1e-6, "ring radius");
  }
});

check("a circle is not an ellipse in this projection", () => {
  // Drawing it as one would be wrong by hundreds of km at Portland's latitude,
  // which is exactly why greatCircle exists.
  const ring = Geo.greatCircle(45.515, -122.678, 2000, 180);
  const lats = ring.map((p) => p[1]);
  const north = Math.max(...lats);
  const south = Math.min(...lats);
  const northSpan = north - 45.515;
  const southSpan = 45.515 - south;
  close(northSpan, southSpan, 1e-6, "latitude extent is symmetric");
  // ... but the longitude extent is not what an equal-degree circle would give
  const lons = ring.map((p) => p[0]);
  const lonSpan = (Math.max(...lons) - Math.min(...lons)) / 2;
  assert.ok(
    lonSpan > northSpan * 1.2,
    `a circle at this latitude must be wider in longitude than in latitude: ${lonSpan} vs ${northSpan}`
  );
});

check("a circle closes on itself", () => {
  const ring = Geo.greatCircle(10, 20, 1500, 36);
  assert.strictEqual(ring.length, 37);
  close(ring[0][0], ring[36][0], 1e-9);
  close(ring[0][1], ring[36][1], 1e-9);
});

check("a ring spanning the antimeridian splits instead of streaking", () => {
  const ring = Geo.greatCircle(0, 179, 2000, 180);
  const parts = Geo.splitAntimeridian(ring);
  assert.ok(parts.length >= 2, "expected the ring to be cut by the frame edge");
  for (const part of parts) {
    for (let i = 1; i < part.length; i++) {
      assert.ok(Math.abs(part[i][0] - part[i - 1][0]) <= 180);
    }
  }
});

/* -- subsolar point ---------------------------------------------------- */

check("the sun is over the tropics at the solstices and the equator at equinox", () => {
  // 2024 June solstice, 2024 December solstice, 2024 March equinox (UTC)
  close(Geo.solarSubpoint(Date.UTC(2024, 5, 20, 20, 51) / 1000)[1], 23.44, 0.05, "June");
  close(Geo.solarSubpoint(Date.UTC(2024, 11, 21, 9, 20) / 1000)[1], -23.44, 0.05, "December");
  close(Geo.solarSubpoint(Date.UTC(2024, 2, 20, 3, 6) / 1000)[1], 0, 0.05, "March");
});

check("the subsolar point is near local noon", () => {
  // At 12:00 UTC the sun is roughly over the prime meridian.
  const [lon] = Geo.solarSubpoint(Date.UTC(2024, 5, 20, 12, 0) / 1000);
  close(lon, 0, 2.0, "subsolar longitude at 12:00 UTC");
});

check("the subsolar point travels west at fifteen degrees an hour", () => {
  const t = Date.UTC(2024, 5, 20, 12, 0) / 1000;
  const a = Geo.solarSubpoint(t)[0];
  const b = Geo.solarSubpoint(t + 3600)[0];
  close(Geo.wrapLon(a - b), 15.0, 0.05);
});

console.log(`\n${checks} checks passed`);
