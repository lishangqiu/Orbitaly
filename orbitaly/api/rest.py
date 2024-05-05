"""REST API routes."""
from __future__ import annotations

import time

import anyio
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..app import pass_dict
from ..motion.errors import MotionBlocked
from .deps import get_services

router = APIRouter()


def _blocked(exc: MotionBlocked) -> HTTPException:
    """An interlock refusal is a conflict, not a bad request."""
    return HTTPException(409, str(exc))


class GotoRequest(BaseModel):
    az: float
    el: float


class JogRequest(BaseModel):
    d_az: float = 0.0
    d_el: float = 0.0


class Doppler2mRequest(BaseModel):
    uplink_hz: float
    downlink_hz: float


class RigRequest(BaseModel):
    enabled: bool | None = None
    uplink_hz: float | None = None
    downlink_hz: float | None = None


@router.get("/satellites")
def list_satellites(request: Request):
    services = get_services(request)
    now = time.time()
    result = []
    for sat in services.tle.satellites.values():
        try:
            obs = services.predictor.observe(sat, now)
        except Exception:
            continue  # bad/decayed TLE
        result.append(
            {
                "norad_id": sat.norad_id,
                "name": sat.name,
                "azimuth": round(obs.azimuth, 1),
                "elevation": round(obs.elevation, 1),
                "range_km": round(obs.range_km, 0),
                "has_transponders": bool(sat.transponders),
            }
        )
    result.sort(key=lambda s: -s["elevation"])
    return {"satellites": result, "count": len(result)}


@router.get("/satellites/{norad_id}")
def satellite_detail(norad_id: int, request: Request):
    services = get_services(request)
    sat = services.tle.get(norad_id)
    if sat is None:
        raise HTTPException(404, f"Unknown satellite {norad_id}")
    obs = services.predictor.observe(sat, time.time())
    return {
        "norad_id": sat.norad_id,
        "name": sat.name,
        "observation": services.observation_dict(obs, sat.transponders),
    }


@router.get("/satellites/{norad_id}/passes")
def satellite_passes(norad_id: int, request: Request, hours: float = 24.0):
    services = get_services(request)
    sat = services.tle.get(norad_id)
    if sat is None:
        raise HTTPException(404, f"Unknown satellite {norad_id}")
    min_el = services.config.tracker.min_elevation_deg
    passes = services.predictor.next_passes(sat, time.time(), hours=hours, min_elevation=0.0)
    visible = [p for p in passes if p.max_elevation >= min_el]
    return {"passes": [pass_dict(p, include_profile=True) for p in visible]}


@router.get("/status")
def status(request: Request):
    return get_services(request).status_snapshot()


@router.post("/track/stop")
def track_stop(request: Request):
    get_services(request).tracker.stop_tracking()
    return {"ok": True}


@router.post("/track/{norad_id}")
def track(norad_id: int, request: Request):
    try:
        get_services(request).tracker.track(norad_id)
    except KeyError as exc:
        raise HTTPException(404, str(exc)) from exc
    except MotionBlocked as exc:
        raise _blocked(exc) from exc
    return {"ok": True}


@router.post("/rotator/goto")
def rotator_goto(body: GotoRequest, request: Request):
    try:
        get_services(request).tracker.manual_goto(body.az, body.el)
    except MotionBlocked as exc:
        raise _blocked(exc) from exc
    return {"ok": True}


@router.post("/rotator/jog")
def rotator_jog(body: JogRequest, request: Request):
    try:
        get_services(request).tracker.manual_jog(body.d_az, body.d_el)
    except MotionBlocked as exc:
        raise _blocked(exc) from exc
    return {"ok": True}


@router.post("/rotator/stop")
def rotator_stop(request: Request):
    get_services(request).tracker.manual_stop()
    return {"ok": True}


@router.post("/rotator/estop")
def rotator_estop(request: Request):
    """Cut motion now. Position reference may be lost; re-home afterwards."""
    services = get_services(request)
    services.tracker.stop_tracking()
    services.rotator.emergency_stop("emergency stop requested from the dashboard")
    return {"ok": True, "rotator": services.status_snapshot()["rotator"]}


@router.post("/rotator/fault/clear")
def rotator_clear_fault(request: Request):
    services = get_services(request)
    cleared = services.rotator.clear_fault()
    state = services.status_snapshot()["rotator"]
    if not cleared:
        raise HTTPException(409, f"Fault condition still present: {state['fault']}")
    return {"ok": True, "rotator": state}


@router.post("/rotator/park")
def rotator_park(request: Request):
    try:
        get_services(request).tracker.park()
    except MotionBlocked as exc:
        raise _blocked(exc) from exc
    return {"ok": True}


@router.post("/rotator/home")
def rotator_home(request: Request):
    get_services(request).rotator.home()
    return {"ok": True}


@router.get("/schedule")
def schedule(request: Request):
    return get_services(request).scheduler.status()


@router.post("/schedule/{action}")
def schedule_set(action: str, request: Request):
    if action not in ("enable", "disable"):
        raise HTTPException(404, "Use /api/schedule/enable or /api/schedule/disable")
    services = get_services(request)
    services.scheduler.set_enabled(action == "enable")
    return services.scheduler.status()


@router.get("/rig")
def rig_status(request: Request):
    services = get_services(request)
    return {**services.tuner.status(), "readout": services.rig_readout()}


@router.post("/rig")
def rig_configure(body: RigRequest, request: Request):
    """Enable/disable tuning and set the rig's working frequencies."""
    services = get_services(request)
    rig_config = services.config.rig
    if body.uplink_hz is not None:
        rig_config.uplink_hz = _check_frequency(body.uplink_hz, "uplink")
    if body.downlink_hz is not None:
        rig_config.downlink_hz = _check_frequency(body.downlink_hz, "downlink")
    if body.enabled is not None:
        services.tuner.set_enabled(body.enabled)
    return {**services.tuner.status(), "readout": services.rig_readout()}


def _check_frequency(hz: float, label: str) -> float:
    # Wide open on purpose: satellites are worked from 29 MHz to 24 GHz.
    if not 1e6 <= hz <= 3e10:
        raise HTTPException(422, f"{label} {hz/1e6:.4f} MHz is not a plausible rig frequency")
    return hz


@router.get("/doppler")
def doppler_2m(request: Request):
    return get_services(request).doppler_2m_readout()


@router.post("/doppler")
def set_doppler_2m(body: Doppler2mRequest, request: Request):
    services = get_services(request)
    try:
        services.set_doppler_2m(body.uplink_hz, body.downlink_hz)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    return services.doppler_2m_readout()


@router.post("/tle/refresh")
async def tle_refresh(request: Request):
    services = get_services(request)
    count = await anyio.to_thread.run_sync(services.tle.refresh)
    return {"ok": True, "satellite_count": count}
