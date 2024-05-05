/* Map geometry: projection, antimeridian splitting, great circles.
 *
 * Pure functions over numbers — no DOM, no canvas, no application state. Two
 * reasons it is separated out:
 *
 *   1. This is the first UI code in the project with real geometry in it, and
 *      geometry is where map views go quietly wrong. Kept pure, it is checked
 *      by `tests/test_geo_js.py`, which runs these functions under node.
 *   2. A 3D globe would replace only the projection. Everything else here —
 *      the visibility circle, the wrap handling — carries over unchanged.
 *
 * Convention: a point is an array `[lon, lat, ...]`. Anything after the first
 * two elements rides along untouched, so a ground-track sample can carry its
 * elevation through a split and still be drawn as in-view.
 */
"use strict";

(function (root, factory) {
  const api = factory();
  /* eslint-disable no-undef */
  if (typeof module === "object" && module.exports) module.exports = api; // node, for the tests
  else root.Geo = api;
})(typeof self !== "undefined" ? self : this, function () {
  const EARTH_RADIUS_KM = 6371.0;
  const D2R = Math.PI / 180;
  const R2D = 180 / Math.PI;

  /* Equirectangular. Deliberately the plainest projection there is: the view
     exists to answer "where is it, and when is it mine", and a fixed 2:1 world
     needs no zoom, no pan, and no tiles. */
  function project(lon, lat, w, h) {
    return [((lon + 180) / 360) * w, ((90 - lat) / 180) * h];
  }

  /* Normalize to [-180, 180). 180 itself maps to -180, which is the same
     meridian; the split helper places its own edge points explicitly so this
     never turns a track's endpoint into a wrap. */
  function wrapLon(lon) {
    let x = (lon + 180) % 360;
    if (x < 0) x += 360;
    return x - 180;
  }

  /* Split a polyline wherever it crosses the antimeridian, inserting the
     crossing point on both edges so the two halves meet the frame instead of
     stopping short of it.

     This is the classic source of horizontal streaks across map UIs: without
     it, a satellite leaving at +179° and arriving at -179° draws a line all
     the way back across the world. Tracks, coverage circles and coastlines all
     go through here. */
  function splitAntimeridian(points) {
    const out = [];
    let run = [];
    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      if (i === 0) {
        run.push(p);
        continue;
      }
      const q = points[i - 1];
      const dlon = p[0] - q[0];
      if (Math.abs(dlon) > 180) {
        // dlon strongly negative: moving east, off the +180 edge.
        const east = dlon < 0;
        const before = east ? 180 : -180;
        const after = east ? -180 : 180;
        const toEdge = east ? 180 - q[0] : q[0] + 180;
        const fromEdge = east ? p[0] + 180 : 180 - p[0];
        const span = toEdge + fromEdge;
        const f = span === 0 ? 0.5 : toEdge / span;
        const lat = q[1] + (p[1] - q[1]) * f;
        // Carry the trailing fields (elevation, time) from the nearer sample,
        // so an in-view arc stays in-view right up to the frame edge.
        run.push([before, lat].concat(q.slice(2)));
        out.push(run);
        run = [[after, lat].concat(p.slice(2)), p];
      } else {
        run.push(p);
      }
    }
    out.push(run);
    return out.filter((r) => r.length >= 2);
  }

  /* A point `f` of the way from `a` to `b`.

     Longitude goes the short way round — a track running 179 -> -179 is one
     degree of motion, not 358 — and every trailing field (elevation, time,
     altitude) is interpolated alongside, so an interpolated point is a valid
     ground-track sample and not just a position. Non-numeric trailing fields
     are carried from `a` rather than turned into NaN. */
  function interpolatePoint(a, b, f) {
    let dlon = b[0] - a[0];
    if (dlon > 180) dlon -= 360;
    else if (dlon < -180) dlon += 360;
    const out = [wrapLon(a[0] + dlon * f), a[1] + (b[1] - a[1]) * f];
    for (let i = 2; i < Math.max(a.length, b.length); i++) {
      const av = a[i];
      const bv = b[i];
      out.push(
        typeof av === "number" && typeof bv === "number" ? av + (bv - av) * f : av
      );
    }
    return out;
  }

  /* Runs of a polyline whose field `index` lies within [lo, hi], entering and
     leaving at the *interpolated* boundary rather than at whole samples.

     Snapping a run to samples is what makes an emphasized stretch of track
     look like a separate floating stroke: at a 60 s step a LEO subpoint moves
     ~450 km between samples, so a 3 px highlight would begin and end up to
     that far from the crossing it is supposed to mark — a fat line starting
     in open ocean. */
  function clipRuns(points, index, lo, hi) {
    const runs = [];
    let run = [];
    const value = (p) => p[index];
    const inside = (p) => value(p) >= lo && value(p) <= hi;
    const crossing = (a, b, bound) => {
      const span = value(b) - value(a);
      const p = interpolatePoint(a, b, span === 0 ? 0 : (bound - value(a)) / span);
      p[index] = bound; // exactly on the boundary, not one rounding off it
      return p;
    };
    const flush = () => {
      if (run.length >= 2) runs.push(run);
      run = [];
    };

    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      const q = i > 0 ? points[i - 1] : null;
      if (inside(p)) {
        if (!run.length && q) run.push(crossing(q, p, value(q) < lo ? lo : hi));
        run.push(p);
      } else if (run.length) {
        run.push(crossing(q, p, value(p) < lo ? lo : hi));
        flush();
      } else if (q && !inside(q) && (value(q) < lo) !== (value(p) < lo)) {
        // In and out again between two samples — a window shorter than the
        // step. Rare, but dropping it would silently un-draw a real one.
        const rising = value(q) < lo;
        runs.push([
          crossing(q, p, rising ? lo : hi),
          crossing(q, p, rising ? hi : lo),
        ]);
      }
    }
    flush();
    return runs;
  }

  /* Where the satellite is above the station's elevation mask. */
  function clipRunsAtMask(points, mask) {
    return clipRuns(points, 2, mask, Infinity);
  }

  /* Where a track sample falls inside a time window — the same clipping, so
     a scheduled pass starts at its AOS instead of at the first sample after
     it. Points are `[lon, lat, el, t, ...]`. */
  function clipRunsInWindow(points, from, to) {
    return clipRuns(points, 3, from, to);
  }

  /* Angular radius of the region that can see a satellite at `altitudeKm`
     above elevation `maskDeg`:

         psi = acos( (Re / (Re + h)) * cos(eps) ) - eps

     For a 6-element yagi on LEO this *is* the range estimate: such a station
     is geometry-limited, not gain-limited — operators work FM birds to the
     horizon with 7 dBi handhelds — so what sets usable range is the lowest
     elevation you can see over trees and terrain, not a link budget.

     Sanity values at eps = 5 degrees: 420 km -> ~1760 km, 550 km -> ~2060 km,
     800 km -> ~2540 km, 5800 km -> ~5900 km. */
  function angularRadiusRad(altitudeKm, maskDeg) {
    if (!(altitudeKm > 0)) return 0;
    const eps = (maskDeg || 0) * D2R;
    const ratio = (EARTH_RADIUS_KM / (EARTH_RADIUS_KM + altitudeKm)) * Math.cos(eps);
    if (ratio >= 1) return 0; // mask so high nothing at this altitude clears it
    return Math.max(0, Math.acos(Math.max(-1, ratio)) - eps);
  }

  function visibilityRadiusKm(altitudeKm, maskDeg) {
    return EARTH_RADIUS_KM * angularRadiusRad(altitudeKm, maskDeg);
  }

  /* Point at `angularRad` from (lat, lon) along `bearingRad`. */
  function destination(lat, lon, angularRad, bearingRad) {
    const phi1 = lat * D2R;
    const lambda1 = lon * D2R;
    const sinPhi1 = Math.sin(phi1);
    const cosPhi1 = Math.cos(phi1);
    const sinD = Math.sin(angularRad);
    const cosD = Math.cos(angularRad);
    const phi2 = Math.asin(sinPhi1 * cosD + cosPhi1 * sinD * Math.cos(bearingRad));
    const lambda2 =
      lambda1 +
      Math.atan2(Math.sin(bearingRad) * sinD * cosPhi1, cosD - sinPhi1 * Math.sin(phi2));
    return [wrapLon(lambda2 * R2D), phi2 * R2D];
  }

  /* A circle of constant great-circle radius, sampled as points on the sphere
     and left for the caller to project.

     It must not be drawn as an ellipse: at Portland's latitude a 2000 km
     circle is visibly not a circle in equirectangular, and IO-117's ring is
     hemisphere-scale. Sampling the sphere is the only way that stays true. */
  function greatCircle(lat, lon, radiusKm, samples) {
    const n = samples || 180;
    const psi = radiusKm / EARTH_RADIUS_KM;
    const points = [];
    for (let i = 0; i <= n; i++) {
      points.push(destination(lat, lon, psi, ((2 * Math.PI) / n) * i));
    }
    return points;
  }

  /* Subsolar point, for the day/night terminator. Low-precision almanac
     formulae — good to a hundredth of a degree, which is far past what a
     terminator drawn at map scale can show. */
  function solarSubpoint(unixSeconds) {
    const n = unixSeconds / 86400.0 + 2440587.5 - 2451545.0; // days from J2000
    const meanLon = 280.46 + 0.9856474 * n;
    const meanAnomaly = (357.528 + 0.9856003 * n) * D2R;
    const eclipticLon =
      (meanLon + 1.915 * Math.sin(meanAnomaly) + 0.02 * Math.sin(2 * meanAnomaly)) * D2R;
    const obliquity = (23.439 - 0.0000004 * n) * D2R;
    const declination = Math.asin(Math.sin(obliquity) * Math.sin(eclipticLon));
    const rightAscension = Math.atan2(
      Math.cos(obliquity) * Math.sin(eclipticLon),
      Math.cos(eclipticLon)
    );
    const gmst = 280.46061837 + 360.98564736629 * n;
    return [wrapLon(rightAscension * R2D - gmst), declination * R2D];
  }

  return {
    EARTH_RADIUS_KM,
    project,
    wrapLon,
    splitAntimeridian,
    interpolatePoint,
    clipRunsAtMask,
    clipRunsInWindow,
    angularRadiusRad,
    visibilityRadiusKm,
    destination,
    greatCircle,
    solarSubpoint,
  };
});
