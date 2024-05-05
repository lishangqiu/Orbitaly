"""WebSocket state feed: pushes the status snapshot to every client at 1 Hz."""
from __future__ import annotations

import asyncio
import logging

import anyio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .deps import get_services

log = logging.getLogger(__name__)
router = APIRouter()

BROADCAST_INTERVAL_S = 1.0


@router.websocket("/ws")
async def state_feed(websocket: WebSocket):
    services = get_services(websocket)
    await websocket.accept()
    try:
        while True:
            snapshot = await anyio.to_thread.run_sync(services.status_snapshot)
            await websocket.send_json(snapshot)
            await asyncio.sleep(BROADCAST_INTERVAL_S)
    except (WebSocketDisconnect, RuntimeError):
        pass
    except Exception:
        log.exception("WebSocket feed error")
