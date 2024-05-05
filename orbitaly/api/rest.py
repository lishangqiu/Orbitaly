"""REST API routes."""
from __future__ import annotations

import time

import anyio
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..app import pass_dict
from ..core.bands import band_status
from ..core.predictor import default_ground_track_window, trim_to_below_mask
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
    sats = services.tle.satellites
    # One pass over the whole catalog rather than an observe() per satellite:
    # this runs on a timer, on a host that is also feeding motion segments.
    observations = services.predictor.observe_many(sats.values(), now)
    bands = services.bands
    result = []
    for norad_id, obs in observations.items():
        sat = sats[norad_id]
        result.append(
            {
                "norad_id": norad_id,
                "name": sat.name,
                "azimuth": round(obs.azimuth, 1),
                "elevation": round(obs.elevation, 1),
                "range_km": round(obs.range_km, 0),
                # Subpoint, so the map can place every satellite it already
                # polls without a ground-track fetch each.
                "latitude": round(obs.latitude, 3),
                "longitude": round(obs.longitude, 3),
                "altitude_km": round(obs.altitude_km, 1),
                "has_transponders": bool(sat.transponders),
                "band": band_status(sat.transponders, bands),
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


@router.get("/satellites/{norad_id}/groundtrack")
def satellite_ground_track(
    norad_id: int,
    request: Request,
    minutes_behind: float | None = None,
    minutes_ahead: float | None = None,
    step_s: float | None = None,
):
    """Subpoints and station elevation across a window around now.

    With no parameters the window is derived from the TLE's own mean motion —
    a quarter orbit behind, one orbit ahead — so a high orbit gets its whole
    track and a LEO bird does not get a needlessly dense one. A little past one
    orbit is sampled and then trimmed back to a below-mask sample, so the
    projection ends under the horizon rather than at an arbitrary instant that
    can fall in the middle of a station pass.
    """
    services = get_services(request)
    sat = services.tle.get(norad_id)
    if sat is None:
        raise HTTPException(404, f"Unknown satellite {norad_id}")

    period_s = services.predictor.orbital_period_s(sat)
    window = default_ground_track_window(period_s)
    behind_s, ahead_s, step = window.behind_s, window.ahead_s, window.step_s
    # An explicit window is the caller saying exactly where the track should
    # end, so it is honoured exactly — which is also how the mid-pass terminus
    # this trim exists to prevent gets reproduced in a browser.
    trim_after_s = window.trim_after_s
    if minutes_behind is not None:
        behind_s = _clamp(minutes_behind, 0.0, 24 * 60.0, "minutes_behind") * 60.0
    if minutes_ahead is not None:
        ahead_s = _clamp(minutes_ahead, 0.0, 24 * 60.0, "minutes_ahead") * 60.0
        trim_after_s = None
    if step_s is not None:
        step = _clamp(step_s, 1.0, 3600.0, "step_s")
    if behind_s + ahead_s <= 0.0:
        raise HTTPException(422, "Ground-track window is empty")

    now = time.time()
    try:
        points = services.predictor.ground_track(sat, now - behind_s, now + ahead_s, step)
    except Exception as exc:  # a decayed element set that will not propagate
        raise HTTPException(422, f"Cannot propagate {sat.name}: {exc}") from exc
    if not points:
        raise HTTPException(422, "Ground-track window is empty")
    if trim_after_s is not None:
        points = trim_to_below_mask(points, now + trim_after_s, services.mask_deg)

    now_index = min(range(len(points)), key=lambda i: abs(points[i].time - now))
    return {
        "norad_id": sat.norad_id,
        "name": sat.name,
        "period_s": round(period_s, 1),
        # The step actually used: `ground_track` widens it rather than truncate
        # a window that would otherwise exceed the sample cap.
        "step_s": round((points[1].time - points[0].time) if len(points) > 1 else step, 3),
        "now_index": now_index,
        "points": [
            {
                "t": round(p.time, 1),
                "lat": round(p.latitude, 3),
                "lon": round(p.longitude, 3),
                "alt_km": round(p.altitude_km, 1),
                "el": round(p.elevation, 2),
            }
            for p in points
        ],
    }


def _clamp(value: float, low: float, high: float, label: str) -> float:
    if not low <= value <= high:
        raise HTTPException(422, f"{label} must be between {low:g} and {high:g}")
    return value


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
