"""Shared API dependencies."""
from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import Request, WebSocket

if TYPE_CHECKING:
    from ..app import Services


def get_services(request: Request | WebSocket) -> "Services":
    return request.app.state.services
