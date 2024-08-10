"""FastAPI application wiring: services container, lifespan, static files."""
from __future__ import annotations

import logging
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from .config import Config
from .core.bands import band_status, effective_mask_deg
from .core.predictor import Observation, Predictor
from .core.tle import TleManager
from .core.scheduler import Scheduler
from .core.tracker import Tracker
from .motion.detect import make_rotator
from .radio import DopplerTuner, make_rig

log = logging.getLogger(__name__)

BAND_2M_HZ = (144_000_000.0, 148_000_000.0)

#: What index.html ships with, and what the configured theme replaces. Keeping
#: a real attribute in the file rather than a placeholder means the page is
#: still valid — and still dark — if it is ever opened straight off disk.
THEME_MARKER = 'data-theme="dark"'


class Services:
    """Everything the API layer needs, in one place."""

    def __init__(self, config: Config):
        self.config = config
        # Normalized once at startup: bad YAML should fail here, loudly, not
        # per request on a station that is already flying a pass.
        self.bands = config.station.band_list()
        self.mask_deg = effective_mask_deg(config)
        self.tle = TleManager(config.tle, config.transponder_overrides)
        self.predictor = Predictor(config.station)
        self.rotator = make_rotator(config.rotator)
        self.tracker = Tracker(config.tracker, self.predictor, self.tle, self.rotator)
        self.doppler_2m_uplink_hz = config.doppler_2m.uplink_hz
        self.doppler_2m_downlink_hz = config.doppler_2m.downlink_hz
        self.rig = make_rig(config.rig)
        self.tuner = DopplerTuner(config.rig, self.rig, self.rig_readout)
        self.scheduler = Scheduler(
            config.scheduler,
            self.predictor,
            self.tle,
            self.tracker,
            self.rotator,
            default_min_elevation=config.tracker.min_elevation_deg,
        )

    def start(self) -> None:
        self.tle.start()
        self.tracker.start()
        self.tuner.start()
        self.scheduler.start()

    def close(self) -> None:
        self.scheduler.close()
        self.tracker.close()
        self.tuner.close()
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

    def station_dict(self) -> dict:
        """Where we are and what we can work.

        The map draws its range ring from this rather than hardcoding a QTH,
        and `mask_deg` is what it may call "in view" — never lower than the
        elevation the tracker itself will schedule.
        """
        station = self.config.station
        return {
            "name": station.name,
            "latitude": station.latitude,
            "longitude": station.longitude,
            "altitude_m": station.altitude_m,
            "min_workable_elevation_deg": station.min_workable_elevation_deg,
            "mask_deg": self.mask_deg,
            "bands": [
                {"min_hz": b.min_hz, "max_hz": b.max_hz, "tx": b.tx, "rx": b.rx}
                for b in self.bands
            ],
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

    def doppler_readout(
        self, uplink_hz: float, downlink_hz: float, band: str = "", at: float | None = None
    ) -> dict:
        """Live doppler correction for one uplink/downlink pair.

        Applied to the currently tracked satellite. Rate (Hz/s) is derived
        numerically over a 1 s baseline — it is what an operator, or the CAT
        tuner, needs to keep a signal centered near TCA.
        """
        readout = {
            "band": band,
            "uplink_hz": uplink_hz,
            "downlink_hz": downlink_hz,
            "active": False,
        }
        norad_id = self.tracker.tracked_norad_id
        sat = self.tle.get(norad_id) if norad_id is not None else None
        if sat is None:
            return readout
        now = at if at is not None else time.time()
        obs = self.predictor.observe(sat, now)
        obs_next = self.predictor.observe(sat, now + 1.0)

        down_shift = obs.doppler_shift_hz(downlink_hz)
        up_shift = obs.doppler_shift_hz(uplink_hz)
        readout.update(
            {
                "active": True,
                "norad_id": sat.norad_id,
                "elevation": obs.elevation,
                # RX: tune here to hear a downlink transmitted on downlink_hz
                "downlink_corrected_hz": downlink_hz + down_shift,
                "downlink_shift_hz": down_shift,
                "downlink_rate_hz_s": obs_next.doppler_shift_hz(downlink_hz) - down_shift,
                # TX: transmit here to arrive on uplink_hz at the satellite
                "uplink_corrected_hz": uplink_hz - up_shift,
                "uplink_shift_hz": up_shift,
                "uplink_rate_hz_s": -(obs_next.doppler_shift_hz(uplink_hz) - up_shift),
            }
        )
        return readout

    def doppler_2m_readout(self, at: float | None = None) -> dict:
        return self.doppler_readout(
            self.doppler_2m_uplink_hz, self.doppler_2m_downlink_hz, band="2m", at=at
        )

    def rig_readout(self, at: float | None = None) -> dict:
        """What the radio should be tuned to right now.

        Defaults to the 2 m panel's working frequencies, so a station that
        only works 2 m needs no extra configuration, but the rig can be
        pointed at any band (a 70 cm downlink, say) independently.
        """
        uplink = self.config.rig.uplink_hz or self.doppler_2m_uplink_hz
        downlink = self.config.rig.downlink_hz or self.doppler_2m_downlink_hz
        return self.doppler_readout(uplink, downlink, band="rig", at=at)

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
                    "band": band_status(sat.transponders, self.bands),
                    "observation": self.observation_dict(obs, sat.transponders),
                }
                current_pass = self.tracker.current_pass
                if current_pass is not None:
                    tracked["pass"] = pass_dict(current_pass, include_profile=True)
        return {
            "time": time.time(),
            "station": self.station_dict(),
            "tracker": {
                "state": self.tracker.state.value,
                "wrap_warning": self.tracker.wrap_warning,
                "blocked": self.tracker.blocked,
            },
            "tracked": tracked,
            "doppler_2m": self.doppler_2m_readout(),
            "rig": self.tuner.status(),
            "schedule": self.scheduler.status(),
            "rotator": {
                "azimuth": round(rotator_state.azimuth, 2),
                "elevation": round(rotator_state.elevation, 2),
                "target_azimuth": round(rotator_state.target_azimuth, 2),
                "target_elevation": round(rotator_state.target_elevation, 2),
                "moving": rotator_state.moving,
                "homed": rotator_state.homed,
                "homing": rotator_state.homing,
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
        rotctld = None
        if config.server.rotctld.enabled:
            from .net.rotctld_server import RotctldServer

            rotctld = RotctldServer(services, config.server.rotctld)
            await rotctld.start()
        try:
            yield
        finally:
            if rotctld is not None:
                await rotctld.stop()
            services.close()

    app = FastAPI(title="Orbitaly", version="0.1.0", lifespan=lifespan)
    app.state.services = services

    from .api.rest import router as rest_router
    from .api.ws import router as ws_router

    app.include_router(rest_router, prefix="/api")
    app.include_router(ws_router)

    static_dir = Path(__file__).parent / "static"
    index_path = static_dir / "index.html"

    # The one page StaticFiles does not get to serve: the theme is stamped onto
    # <html> here rather than applied by app.js after load, so a light-theme
    # station does not flash the dark palette on every refresh. Registered
    # before the mount below, which would otherwise answer "/" itself.
    @app.get("/", include_in_schema=False)
    @app.get("/index.html", include_in_schema=False)
    def index() -> HTMLResponse:
        html = index_path.read_text(encoding="utf-8")
        return HTMLResponse(
            html.replace(THEME_MARKER, f'data-theme="{config.ui.theme}"', 1)
        )

    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
    return app
