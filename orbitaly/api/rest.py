"""REST API routes."""
from __future__ import annotations

import time

import anyio
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..app import pass_dict
from .deps import get_services

router = APIRouter()


class GotoRequest(BaseModel):
    az: float
    el: float


class JogRequest(BaseModel):
    d_az: float = 0.0
    d_el: float = 0.0


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
    return {"ok": True}


@router.post("/rotator/goto")
def rotator_goto(body: GotoRequest, request: Request):
    get_services(request).tracker.manual_goto(body.az, body.el)
    return {"ok": True}


@router.post("/rotator/jog")
def rotator_jog(body: JogRequest, request: Request):
    get_services(request).tracker.manual_jog(body.d_az, body.d_el)
    return {"ok": True}


@router.post("/rotator/stop")
def rotator_stop(request: Request):
    get_services(request).tracker.manual_stop()
    return {"ok": True}


@router.post("/rotator/park")
def rotator_park(request: Request):
    get_services(request).tracker.park()
    return {"ok": True}


@router.post("/rotator/home")
def rotator_home(request: Request):
    get_services(request).rotator.home()
    return {"ok": True}


@router.post("/tle/refresh")
async def tle_refresh(request: Request):
    services = get_services(request)
    count = await anyio.to_thread.run_sync(services.tle.refresh)
    return {"ok": True, "satellite_count": count}
