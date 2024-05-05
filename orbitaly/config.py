"""Configuration loading for Orbitaly.

Everything has a default so `python -m orbitaly` runs with no config file at
all (simulated rotator, placeholder station). A YAML file overrides any
subset of the defaults.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_TLE_SOURCES = [
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle",
]


@dataclass
class StationConfig:
    name: str = "Ground Station"
    latitude: float = 40.4406
    longitude: float = -79.9959
    altitude_m: float = 300.0


@dataclass
class TleConfig:
    sources: list[str] = field(default_factory=lambda: list(DEFAULT_TLE_SOURCES))
    refresh_hours: float = 6.0
    cache_path: str = ""

    def resolved_cache_path(self) -> Path:
        if self.cache_path:
            return Path(self.cache_path).expanduser()
        base = Path(os.environ.get("XDG_CACHE_HOME", "~/.cache")).expanduser()
        return base / "orbitaly" / "tle.json"


@dataclass
class ParkConfig:
    az: float = 0.0
    el: float = 90.0


@dataclass
class TrackerConfig:
    update_rate_s: float = 1.0
    min_elevation_deg: float = 5.0
    pass_lookahead_hours: float = 24.0
    park: ParkConfig = field(default_factory=ParkConfig)


@dataclass
class PinConfig:
    step: int = 0
    dir: int = 0
    enable: int = -1   # -1 = not wired
    endstop: int = -1


@dataclass
class AxisConfig:
    min_deg: float = 0.0
    max_deg: float = 360.0
    max_speed_dps: float = 6.0
    accel_dps2: float = 4.0
    steps_per_rev: int = 200
    microsteps: int = 8
    gear_ratio: float = 60.0
    backlash_deg: float = 0.0
    invert_dir: bool = False
    home_position_deg: float = 0.0
    pins: PinConfig = field(default_factory=PinConfig)

    @property
    def steps_per_deg(self) -> float:
        return self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0


@dataclass
class RotatorConfig:
    backend: str = "simulated"  # simulated | gpio
    azimuth: AxisConfig = field(
        default_factory=lambda: AxisConfig(min_deg=-90.0, max_deg=450.0)
    )
    elevation: AxisConfig = field(
        default_factory=lambda: AxisConfig(
            min_deg=0.0, max_deg=180.0, max_speed_dps=4.0, gear_ratio=40.0
        )
    )


@dataclass
class Doppler2mConfig:
    """Working frequencies for the 2 m band doppler readout (Hz)."""

    uplink_hz: float = 145_990_000.0    # ARISS voice uplink as a sensible default
    downlink_hz: float = 145_800_000.0  # ISS voice downlink


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000


@dataclass
class Config:
    station: StationConfig = field(default_factory=StationConfig)
    tle: TleConfig = field(default_factory=TleConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    rotator: RotatorConfig = field(default_factory=RotatorConfig)
    doppler_2m: Doppler2mConfig = field(default_factory=Doppler2mConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    transponder_overrides: dict[str, Any] = field(default_factory=dict)


def _merge_dataclass(instance: Any, data: dict[str, Any]) -> Any:
    """Recursively apply a dict of overrides onto a dataclass instance."""
    for key, value in data.items():
        if not hasattr(instance, key):
            raise ValueError(f"Unknown config key: {key!r} on {type(instance).__name__}")
        current = getattr(instance, key)
        if isinstance(value, dict) and hasattr(current, "__dataclass_fields__"):
            _merge_dataclass(current, value)
        else:
            setattr(instance, key, value)
    return instance


def load_config(path: str | Path | None = None) -> Config:
    config = Config()
    if path is None:
        return config
    raw = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Config file {path} must be a YAML mapping")
    return _merge_dataclass(config, raw)
