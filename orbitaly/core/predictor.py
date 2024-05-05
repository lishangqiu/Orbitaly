"""Orbital math: real-time observations, pass prediction, doppler.

Thin, well-typed wrapper around Skyfield. All angles in degrees, distances
in kilometers, frequencies in hertz, times as unix timestamps (UTC).
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

from skyfield.api import EarthSatellite, load, wgs84

from ..config import StationConfig
from .tle import Satellite

SPEED_OF_LIGHT_M_S = 299_792_458.0
EARTH_RADIUS_KM = 6371.0


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
    def doppler_factor(self) -> float:
        """Multiply an emitted downlink frequency by this to get the observed one."""
        return 1.0 - (self.range_rate_km_s * 1000.0) / SPEED_OF_LIGHT_M_S

    def doppler_shift_hz(self, frequency_hz: float) -> float:
        return frequency_hz * (self.doppler_factor - 1.0)


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
        with self._lock:
            cached = self._sat_cache.get(sat.norad_id)
            if cached and cached[0] == sat.line1:
                return cached[1]
            es = EarthSatellite(sat.line1, sat.line2, sat.name, self._ts)
            self._sat_cache[sat.norad_id] = (sat.line1, es)
            return es

    def observe(self, sat: Satellite, unix_time: float) -> Observation:
        es = self._earth_satellite(sat)
        t = self._ts.from_datetime(_utc(unix_time))
        difference = (es - self._topos).at(t)
        alt, az, distance, _, _, range_rate = difference.frame_latlon_and_rates(self._topos)
        geocentric = es.at(t)
        subpoint = wgs84.subpoint_of(geocentric)
        altitude_km = wgs84.height_of(geocentric).km
        # Footprint: great-circle radius of the visibility circle
        import math

        ratio = EARTH_RADIUS_KM / (EARTH_RADIUS_KM + altitude_km)
        footprint = EARTH_RADIUS_KM * math.acos(min(1.0, max(-1.0, ratio)))
        eph = self._ephemeris()
        sunlit = bool(geocentric.is_sunlit(eph)) if eph else True
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
            altitude_km=float(altitude_km),
            footprint_km=float(footprint),
            sunlit=sunlit,
        )

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
