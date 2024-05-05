"""FastAPI application wiring: services container, lifespan, static files."""
from __future__ import annotations

import logging
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .config import Config
from .core.predictor import Observation, Predictor
from .core.tle import TleManager
from .core.tracker import Tracker
from .hardware.gpio import make_rotator

log = logging.getLogger(__name__)

BAND_2M_HZ = (144_000_000.0, 148_000_000.0)


class Services:
    """Everything the API layer needs, in one place."""

    def __init__(self, config: Config):
        self.config = config
        self.tle = TleManager(config.tle, config.transponder_overrides)
        self.predictor = Predictor(config.station)
        self.rotator = make_rotator(config.rotator)
        self.tracker = Tracker(config.tracker, self.predictor, self.tle, self.rotator)
        self.doppler_2m_uplink_hz = config.doppler_2m.uplink_hz
        self.doppler_2m_downlink_hz = config.doppler_2m.downlink_hz

    def start(self) -> None:
        self.tle.start()
        self.tracker.start()

    def close(self) -> None:
        self.tracker.close()
        self.rotator.close()
        self.tle.stop()

    # -- shared snapshot (REST /api/status and the WebSocket feed) ----------

    def observation_dict(self, obs: Observation, transponders: list[dict]) -> dict:
        entries = []
        for t in transponders:
            downlink = t.get("downlink_hz")
            uplink = t.get("uplink_hz")
            entries.append(
                {
                    **t,
                    "downlink_corrected_hz": downlink + obs.doppler_shift_hz(downlink) if downlink else None,
                    "uplink_corrected_hz": uplink - obs.doppler_shift_hz(uplink) if uplink else None,
                }
            )
        return {
            "time": obs.time,
            "azimuth": round(obs.azimuth, 2),
            "elevation": round(obs.elevation, 2),
            "range_km": round(obs.range_km, 1),
            "range_rate_km_s": round(obs.range_rate_km_s, 4),
            "latitude": round(obs.latitude, 3),
            "longitude": round(obs.longitude, 3),
            "altitude_km": round(obs.altitude_km, 1),
            "footprint_km": round(obs.footprint_km, 0),
            "sunlit": obs.sunlit,
            "transponders": entries,
        }

    # -- 2 m band doppler ---------------------------------------------------

    def set_doppler_2m(self, uplink_hz: float, downlink_hz: float) -> None:
        lo, hi = BAND_2M_HZ
        for label, freq in (("uplink", uplink_hz), ("downlink", downlink_hz)):
            if not lo <= freq <= hi:
                raise ValueError(
                    f"{label} {freq/1e6:.4f} MHz is outside the 2 m band "
                    f"({lo/1e6:.0f}-{hi/1e6:.0f} MHz)"
                )
        self.doppler_2m_uplink_hz = uplink_hz
        self.doppler_2m_downlink_hz = downlink_hz

    def doppler_2m_readout(self, at: float | None = None) -> dict:
        """Live doppler correction for the configured 2 m working frequencies.

        Applied to the currently tracked satellite. Rate (Hz/s) is derived
        numerically over a 1 s baseline — it is what an operator (or later,
        CAT control) needs to keep a signal centered near TCA.
        """
        readout = {
            "band": "2m",
            "uplink_hz": self.doppler_2m_uplink_hz,
            "downlink_hz": self.doppler_2m_downlink_hz,
            "active": False,
        }
        norad_id = self.tracker.tracked_norad_id
        sat = self.tle.get(norad_id) if norad_id is not None else None
        if sat is None:
            return readout
        now = at if at is not None else time.time()
        obs = self.predictor.observe(sat, now)
        obs_next = self.predictor.observe(sat, now + 1.0)

        down_shift = obs.doppler_shift_hz(self.doppler_2m_downlink_hz)
        up_shift = obs.doppler_shift_hz(self.doppler_2m_uplink_hz)
        readout.update(
            {
                "active": True,
                "norad_id": sat.norad_id,
                # RX: tune here to hear a downlink transmitted on downlink_hz
                "downlink_corrected_hz": self.doppler_2m_downlink_hz + down_shift,
                "downlink_shift_hz": down_shift,
                "downlink_rate_hz_s": obs_next.doppler_shift_hz(self.doppler_2m_downlink_hz) - down_shift,
                # TX: transmit here to arrive on uplink_hz at the satellite
                "uplink_corrected_hz": self.doppler_2m_uplink_hz - up_shift,
                "uplink_shift_hz": up_shift,
                "uplink_rate_hz_s": -(obs_next.doppler_shift_hz(self.doppler_2m_uplink_hz) - up_shift),
            }
        )
        return readout

    def status_snapshot(self) -> dict:
        rotator_state = self.rotator.state()
        tracked = None
        norad_id = self.tracker.tracked_norad_id
        if norad_id is not None:
            sat = self.tle.get(norad_id)
            if sat is not None:
                obs = self.predictor.observe(sat, time.time())
                tracked = {
                    "norad_id": norad_id,
                    "name": sat.name,
                    "observation": self.observation_dict(obs, sat.transponders),
                }
                current_pass = self.tracker.current_pass
                if current_pass is not None:
                    tracked["pass"] = pass_dict(current_pass, include_profile=True)
        return {
            "time": time.time(),
            "station": {
                "name": self.config.station.name,
                "latitude": self.config.station.latitude,
                "longitude": self.config.station.longitude,
            },
            "tracker": {
                "state": self.tracker.state.value,
                "wrap_warning": self.tracker.wrap_warning,
            },
            "tracked": tracked,
            "doppler_2m": self.doppler_2m_readout(),
            "rotator": {
                "azimuth": round(rotator_state.azimuth, 2),
                "elevation": round(rotator_state.elevation, 2),
                "target_azimuth": round(rotator_state.target_azimuth, 2),
                "target_elevation": round(rotator_state.target_elevation, 2),
                "moving": rotator_state.moving,
                "homed": rotator_state.homed,
                "fault": rotator_state.fault,
                "azimuth_travel": self.rotator.azimuth_travel,
                "elevation_travel": self.rotator.elevation_travel,
            },
            "tle": {
                "satellite_count": len(self.tle.satellites),
                "fetched_at": self.tle.fetched_at,
            },
        }


def pass_dict(p, include_profile: bool = False) -> dict:
    d = {
        "norad_id": p.norad_id,
        "aos": p.aos,
        "tca": p.tca,
        "los": p.los,
        "duration_s": round(p.duration_s, 0),
        "max_elevation": round(p.max_elevation, 1),
        "aos_azimuth": round(p.aos_azimuth, 1),
        "los_azimuth": round(p.los_azimuth, 1),
    }
    if include_profile:
        d["profile"] = [
            {"time": pt.time, "az": round(pt.azimuth, 1), "el": round(pt.elevation, 1)}
            for pt in p.profile
        ]
    return d


def create_app(config: Config) -> FastAPI:
    services = Services(config)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        services.start()
        try:
            yield
        finally:
            services.close()

    app = FastAPI(title="Orbitaly", version="0.1.0", lifespan=lifespan)
    app.state.services = services

    from .api.rest import router as rest_router
    from .api.ws import router as ws_router

    app.include_router(rest_router, prefix="/api")
    app.include_router(ws_router)

    static_dir = Path(__file__).parent / "static"
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
    return app
