"""TLE fetching, caching, and the transponder database."""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path

import httpx

from ..config import TleConfig

log = logging.getLogger(__name__)


@dataclass
class Satellite:
    norad_id: int
    name: str
    line1: str
    line2: str
    transponders: list[dict] = field(default_factory=list)


def parse_tle_text(text: str) -> dict[int, Satellite]:
    """Parse a 3-line-element file into satellites keyed by NORAD id."""
    sats: dict[int, Satellite] = {}
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    i = 0
    while i < len(lines) - 1:
        # A record is NAME / 1 ... / 2 ...  (name line optional in 2LE files)
        if lines[i].startswith("1 ") and i + 1 < len(lines) and lines[i + 1].startswith("2 "):
            name, l1, l2 = "", lines[i], lines[i + 1]
            i += 2
        elif (
            i + 2 < len(lines)
            and lines[i + 1].startswith("1 ")
            and lines[i + 2].startswith("2 ")
        ):
            name, l1, l2 = lines[i].strip(), lines[i + 1], lines[i + 2]
            i += 3
        else:
            i += 1
            continue
        try:
            norad_id = int(l1[2:7])
        except ValueError:
            continue
        sats[norad_id] = Satellite(norad_id, name or f"NORAD {norad_id}", l1, l2)
    return sats


def load_transponder_db() -> dict[int, list[dict]]:
    """Built-in transponder frequency database, keyed by NORAD id."""
    raw = resources.files("orbitaly.data").joinpath("transponders.json").read_text()
    data = json.loads(raw)
    return {int(k): v for k, v in data.items()}


class TleManager:
    """Fetches TLE sets, caches them on disk, and refreshes periodically."""

    def __init__(self, config: TleConfig, transponder_overrides: dict | None = None):
        self.config = config
        self._lock = threading.Lock()
        self._satellites: dict[int, Satellite] = {}
        self._fetched_at: float = 0.0
        self._transponders = load_transponder_db()
        for key, value in (transponder_overrides or {}).items():
            self._transponders[int(key)] = value
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- public API ---------------------------------------------------------

    @property
    def satellites(self) -> dict[int, Satellite]:
        with self._lock:
            return dict(self._satellites)

    def get(self, norad_id: int) -> Satellite | None:
        with self._lock:
            return self._satellites.get(norad_id)

    @property
    def fetched_at(self) -> float:
        return self._fetched_at

    def start(self) -> None:
        """Load cache immediately, then refresh in the background."""
        self._load_cache()
        self._thread = threading.Thread(target=self._refresh_loop, daemon=True, name="tle-refresh")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def refresh(self) -> int:
        """Fetch all sources now. Returns satellite count; raises nothing."""
        merged: dict[int, Satellite] = {}
        for url in self.config.sources:
            try:
                response = httpx.get(url, timeout=30.0, follow_redirects=True)
                response.raise_for_status()
                merged.update(parse_tle_text(response.text))
            except Exception as exc:  # network failures must never kill tracking
                log.warning("TLE fetch failed for %s: %s", url, exc)
        if merged:
            self._install(merged, time.time())
            self._save_cache()
        return len(merged)

    # -- internals ----------------------------------------------------------

    def _install(self, sats: dict[int, Satellite], fetched_at: float) -> None:
        for sat in sats.values():
            sat.transponders = self._transponders.get(sat.norad_id, [])
        with self._lock:
            self._satellites = sats
            self._fetched_at = fetched_at

    def _refresh_loop(self) -> None:
        interval = max(self.config.refresh_hours, 0.1) * 3600.0
        while not self._stop.is_set():
            if time.time() - self._fetched_at >= interval:
                self.refresh()
            self._stop.wait(60.0)

    def _cache_file(self) -> Path:
        return self.config.resolved_cache_path()

    def _save_cache(self) -> None:
        path = self._cache_file()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with self._lock:
                payload = {
                    "fetched_at": self._fetched_at,
                    "satellites": [
                        [s.norad_id, s.name, s.line1, s.line2]
                        for s in self._satellites.values()
                    ],
                }
            path.write_text(json.dumps(payload))
        except OSError as exc:
            log.warning("Could not write TLE cache %s: %s", path, exc)

    def _load_cache(self) -> None:
        path = self._cache_file()
        try:
            payload = json.loads(path.read_text())
        except (OSError, ValueError):
            return
        sats = {
            int(nid): Satellite(int(nid), name, l1, l2)
            for nid, name, l1, l2 in payload.get("satellites", [])
        }
        if sats:
            self._install(sats, float(payload.get("fetched_at", 0.0)))
            log.info("Loaded %d satellites from cache", len(sats))
