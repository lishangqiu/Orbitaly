"""Orbital math: real-time observations, pass prediction, doppler.

Thin, well-typed wrapper around Skyfield. All angles in degrees, distances
in kilometers, frequencies in hertz, times as unix timestamps (UTC).
"""
from __future__ import annotations

import math
import threading
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np
from skyfield.api import EarthSatellite, load, wgs84

from ..config import StationConfig
from .tle import Satellite

try:
    # Skyfield's own eclipse test, so the hoisted version below stays exactly
    # what `Position.is_sunlit()` computes rather than a lookalike of it.
    from skyfield.constants import ERAD
    from skyfield.geometry import intersect_line_and_sphere
except ImportError:  # pragma: no cover - falls back to the per-position method
    intersect_line_and_sphere = None
    ERAD = 0.0

SPEED_OF_LIGHT_M_S = 299_792_458.0
EARTH_RADIUS_KM = 6371.0

#: Backstop on one ground-track request. A decayed or mis-parsed TLE can ask
#: for an arbitrarily long window; past this the step is widened rather than
#: the window truncated, because half a track drawn as a whole one would lie.
MAX_GROUND_TRACK_SAMPLES = 500

#: Never project further ahead than this, however long the orbit. A decayed or
#: geosynchronous element set would otherwise ask for a day of samples.
MAX_GROUND_TRACK_AHEAD_S = 6 * 3600.0

#: Fallback period when a TLE's mean motion is unusable: a typical LEO orbit.
FALLBACK_PERIOD_S = 95 * 60.0

#: How far past one period the window is sampled so that its end can be moved
#: to somewhere that means something. One period ahead, the subpoint lands
#: ~24° west of the current one — so whenever an operator is watching a pass
#: come up, and the current subpoint is therefore near the station, the
#: terminus keeps landing near or inside the *next* station pass. A track that
#: stops mid-pass is indistinguishable from a severed one, and was read as
#: one. `trim_to_below_mask` spends this margin ending the track below the
#: horizon instead.
GROUND_TRACK_TAIL_MARGIN_S = 20 * 60.0


class TrackWindow(NamedTuple):
    """A ground-track request, in seconds relative to now.

    `trim_after_s` is the earliest terminus a tail trim may cut back to — one
    whole period — so the margin past it can be given up but the orbit itself
    never is.
    """

    behind_s: float
    ahead_s: float
    step_s: float
    trim_after_s: float


def default_ground_track_window(period_s: float) -> TrackWindow:
    """The window to draw for a satellite of this orbital period.

    Scaled by the orbit rather than fixed in minutes. "~100 min is one orbit"
    is a LEO assumption, and IO-117 — a ~3.8 h orbit and one of the most-worked
    digipeaters flying — would show less than half its track under it.
    """
    if not period_s or period_s <= 0 or not math.isfinite(period_s):
        period_s = FALLBACK_PERIOD_S
    behind = period_s / 4.0
    one_orbit = min(period_s, MAX_GROUND_TRACK_AHEAD_S)
    ahead = min(period_s + GROUND_TRACK_TAIL_MARGIN_S, MAX_GROUND_TRACK_AHEAD_S)
    # ~120 samples an orbit: at map scale a LEO subpoint moves about 4 px in a
    # minute, so a finer step buys nothing but samples.
    step = max(60.0, period_s / 120.0)
    return TrackWindow(behind, ahead, step, one_orbit)


def trim_to_below_mask(
    points: list[GroundTrackPoint], not_before: float, mask_deg: float
) -> list[GroundTrackPoint]:
    """Cut a track's tail back to the first below-mask sample past `not_before`.

    A projection has to stop somewhere, and where it stops carries no meaning —
    so it should at least stop somewhere the operator has no reason to look. A
    below-mask terminus is that place: the satellite is under the horizon
    there, which is the same statement the in-view highlight makes with the
    same number (`services.mask_deg`).

    If nothing in the margin is below the mask — a satellite that never sets,
    or one already past the sample cap — the window is returned untouched and
    the client's fading tail covers a genuinely arbitrary end, which is what it
    was built for.
    """
    for i, p in enumerate(points):
        if p.time >= not_before and p.elevation < mask_deg:
            return points[: i + 1]
    return points


@dataclass
class Observation:
    time: float
    azimuth: float
    elevation: float
    range_km: float
    range_rate_km_s: float
    latitude: float
    longitude: float
    altitude_km: float
    footprint_km: float
    sunlit: bool

    @property
    def is_finite(self) -> bool:
        """False when the TLE would not propagate.

        SGP4 reports a hopeless element set by returning NaN rather than by
        raising, so a `try/except` around propagation does not catch it. That
        matters at the JSON boundary: a bare `NaN` is not valid JSON, and one
        decayed satellite would otherwise make the whole catalog unparseable
        in the browser.
        """
        return all(
            math.isfinite(v)
            for v in (
                self.azimuth,
                self.elevation,
                self.range_km,
                self.range_rate_km_s,
                self.latitude,
                self.longitude,
                self.altitude_km,
            )
        )

    @property
    def doppler_factor(self) -> float:
        """Multiply an emitted downlink frequency by this to get the observed one."""
        return 1.0 - (self.range_rate_km_s * 1000.0) / SPEED_OF_LIGHT_M_S

    def doppler_shift_hz(self, frequency_hz: float) -> float:
        return frequency_hz * (self.doppler_factor - 1.0)


@dataclass
class GroundTrackPoint:
    """One sample of a satellite's path over the ground.

    `elevation` is the satellite's elevation *at this station*, carried along
    so the map can highlight in-view arcs with a comparison instead of
    redoing spherical trigonometry in the browser.
    """

    time: float
    latitude: float
    longitude: float
    altitude_km: float
    elevation: float


@dataclass
class PassPoint:
    time: float
    azimuth: float
    elevation: float


@dataclass
class Pass:
    norad_id: int
    aos: float
    tca: float
    los: float
    max_elevation: float
    aos_azimuth: float
    los_azimuth: float
    profile: list[PassPoint] = field(default_factory=list)

    @property
    def duration_s(self) -> float:
        return self.los - self.aos


class Predictor:
    """Computes observations and passes for one ground station."""

    def __init__(self, station: StationConfig):
        self.station = station
        self._ts = load.timescale()
        self._topos = wgs84.latlon(station.latitude, station.longitude, station.altitude_m)
        self._eph = None  # lazy: sun ephemeris for sunlit flag
        self._sat_cache: dict[int, tuple[str, EarthSatellite]] = {}
        self._lock = threading.Lock()

    def _earth_satellite(self, sat: Satellite) -> EarthSatellite:
        # Keyed on both lines: line 2 carries the orbital elements, so
        # validating only line 1 would hand back a stale satellite for any
        # update that left the epoch alone.
        with self._lock:
            cached = self._sat_cache.get(sat.norad_id)
            if cached and cached[0] == (sat.line1, sat.line2):
                return cached[1]
            es = EarthSatellite(sat.line1, sat.line2, sat.name, self._ts)
            self._sat_cache[sat.norad_id] = ((sat.line1, sat.line2), es)
            return es

    def observe(self, sat: Satellite, unix_time: float) -> Observation:
        es = self._earth_satellite(sat)
        t = self._ts.from_datetime(_utc(unix_time))
        return self._observation(
            es.at(t), self._topos.at(t), unix_time, self._sun_vector_m(t)
        )

    def observe_many(
        self, sats: Iterable[Satellite], unix_time: float
    ) -> dict[int, Observation]:
        """Observe a whole catalog at one instant, keyed by NORAD id.

        Same numbers as calling `observe()` in a loop, but everything that
        depends only on the *time* — the Time object and its nutation/GAST
        terms, the station's position, the Sun's — is computed once instead of
        once per satellite. That is most of the work: a plain loop spends about
        half its time re-evaluating the JPL ephemeris for the eclipse flag.

        Satellites whose TLE will not propagate are skipped, as the catalog
        endpoint has always done: one decayed element set must not blank the
        whole list.
        """
        t = self._ts.from_datetime(_utc(unix_time))
        topos_at = self._topos.at(t)
        sun_m = self._sun_vector_m(t)
        out: dict[int, Observation] = {}
        for sat in sats:
            try:
                geocentric = self._earth_satellite(sat).at(t)
                observation = self._observation(geocentric, topos_at, unix_time, sun_m)
            except Exception:
                continue  # malformed TLE
            if observation.is_finite:
                out[sat.norad_id] = observation
        return out

    def ground_track(
        self,
        sat: Satellite,
        start_time: float,
        end_time: float,
        step_s: float = 60.0,
    ) -> list[GroundTrackPoint]:
        """Subpoints and station elevation across a time window.

        One vectorized Skyfield propagation over the whole time array, not a
        sample-by-sample loop: the server is a Pi that is also feeding motion
        segments in soft real time, and prediction CPU there is not free.
        """
        if end_time <= start_time:
            return []
        step_s = max(float(step_s), 1.0)
        span = end_time - start_time
        if span / step_s + 1 > MAX_GROUND_TRACK_SAMPLES:
            step_s = span / (MAX_GROUND_TRACK_SAMPLES - 1)
        times = np.arange(start_time, end_time + step_s * 0.5, step_s)

        es = self._earth_satellite(sat)
        t = self._ts.from_datetimes([_utc(x) for x in times])
        geocentric = es.at(t)
        subpoint = wgs84.subpoint_of(geocentric)
        heights_km = wgs84.height_of(geocentric).km
        alt, _az, _distance = (geocentric - self._topos.at(t)).frame_latlon(self._topos)

        lats = subpoint.latitude.degrees
        lons = subpoint.longitude.degrees
        elevations = alt.degrees
        if not np.isfinite(lats).all() or not np.isfinite(elevations).all():
            # SGP4 signals a hopeless element set with NaN rather than an
            # exception. Say so, rather than serving a track full of nulls.
            raise ValueError(f"{sat.name} does not propagate over this window")
        return [
            GroundTrackPoint(
                time=float(times[i]),
                latitude=float(lats[i]),
                longitude=float(lons[i]),
                altitude_km=float(heights_km[i]),
                elevation=float(elevations[i]),
            )
            for i in range(len(times))
        ]

    def orbital_period_s(self, sat: Satellite) -> float:
        """Orbital period from the TLE's mean motion, 0.0 if it is unusable."""
        try:
            no_kozai = float(self._earth_satellite(sat).model.no_kozai)  # rad/min
        except Exception:
            return 0.0
        if no_kozai <= 0.0 or not math.isfinite(no_kozai):
            return 0.0
        return 2.0 * math.pi / no_kozai * 60.0

    # -- shared observation math -------------------------------------------

    def _observation(
        self, geocentric, topos_at, unix_time: float, sun_m
    ) -> Observation:
        """Build an Observation from an already-propagated geocentric position.

        Subtracting a precomputed station position is bit-for-bit what
        `(satellite - topos).at(t)` produces, and it propagates the satellite
        once rather than twice.
        """
        difference = geocentric - topos_at
        alt, az, distance, _, _, range_rate = difference.frame_latlon_and_rates(self._topos)
        subpoint = wgs84.subpoint_of(geocentric)
        altitude_km = float(wgs84.height_of(geocentric).km)
        # Footprint: great-circle radius of the visibility circle
        ratio = EARTH_RADIUS_KM / (EARTH_RADIUS_KM + altitude_km)
        footprint = EARTH_RADIUS_KM * math.acos(min(1.0, max(-1.0, ratio)))
        # float() everywhere: strip numpy scalars at the boundary so plain
        # Python types flow through the tracker, HAL, and JSON API.
        return Observation(
            time=unix_time,
            azimuth=float(az.degrees) % 360.0,
            elevation=float(alt.degrees),
            range_km=float(distance.km),
            range_rate_km_s=float(range_rate.km_per_s),
            latitude=float(subpoint.latitude.degrees),
            longitude=float(subpoint.longitude.degrees),
            altitude_km=altitude_km,
            footprint_km=float(footprint),
            sunlit=self._is_sunlit(geocentric, sun_m),
        )

    def _sun_vector_m(self, t):
        """Earth->Sun vector in metres, or None if the eclipse test is unavailable."""
        eph = self._ephemeris()
        if eph is None or intersect_line_and_sphere is None:
            return None
        return (eph["sun"] - eph["earth"]).at(t).xyz.m

    def _is_sunlit(self, geocentric, sun_m) -> bool:
        if sun_m is None:
            eph = self._ephemeris()
            # offline, or an unfamiliar Skyfield: degrade gracefully
            return bool(geocentric.is_sunlit(eph)) if eph else True
        earth_m = -geocentric.xyz.m
        _near, far = intersect_line_and_sphere(sun_m + earth_m, earth_m, ERAD)
        return bool(np.nan_to_num(far) <= 0)

    def next_passes(
        self,
        sat: Satellite,
        start_time: float,
        hours: float = 24.0,
        min_elevation: float = 0.0,
        profile_step_s: float = 15.0,
    ) -> list[Pass]:
        es = self._earth_satellite(sat)
        t0 = self._ts.from_datetime(_utc(start_time))
        t1 = self._ts.from_datetime(_utc(start_time + hours * 3600.0))
        times, events = es.find_events(self._topos, t0, t1, altitude_degrees=min_elevation)

        passes: list[Pass] = []
        current: dict[str, float] = {}
        for t, event in zip(times, events):
            unix = t.utc_datetime().timestamp()
            if event == 0:  # rise
                current = {"aos": unix}
            elif event == 1:  # culmination
                current.setdefault("aos", start_time)
                current["tca"] = unix
            elif event == 2:  # set
                aos = current.get("aos", start_time)
                tca = current.get("tca", (aos + unix) / 2.0)
                passes.append(self._build_pass(sat, aos, tca, unix, profile_step_s))
                current = {}
        # A pass in progress at the end of the window (rise without set)
        if "aos" in current:
            aos = current["aos"]
            tca = current.get("tca", aos)
            passes.append(self._build_pass(sat, aos, max(tca, aos), start_time + hours * 3600.0, profile_step_s))
        return passes

    def _build_pass(
        self, sat: Satellite, aos: float, tca: float, los: float, step_s: float
    ) -> Pass:
        profile: list[PassPoint] = []
        t = aos
        while t < los:
            obs = self.observe(sat, t)
            profile.append(PassPoint(t, obs.azimuth, obs.elevation))
            t += step_s
        los_obs = self.observe(sat, los)
        profile.append(PassPoint(los, los_obs.azimuth, los_obs.elevation))
        tca_obs = self.observe(sat, tca)
        return Pass(
            norad_id=sat.norad_id,
            aos=aos,
            tca=tca,
            los=los,
            max_elevation=max([p.elevation for p in profile] + [tca_obs.elevation]),
            aos_azimuth=profile[0].azimuth,
            los_azimuth=profile[-1].azimuth,
            profile=profile,
        )

    def _ephemeris(self):
        if self._eph is None:
            try:
                self._eph = load("de421.bsp")
            except Exception:
                self._eph = False  # offline: sunlit flag degrades gracefully
        return self._eph if self._eph else None


def _utc(unix_time: float):
    from datetime import datetime, timezone

    return datetime.fromtimestamp(unix_time, tz=timezone.utc)
