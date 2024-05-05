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
    """BCM pin numbers. -1 means not wired.

    step and dir default to unwired rather than to GPIO 0: zero is a real pin
    (the HAT ID EEPROM line), and defaulting both signals to it would have any
    unconfigured axis quietly claim a reserved pin twice.
    """

    step: int = -1
    dir: int = -1
    enable: int = -1
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

    # -- pulse shaping (driver datasheet territory) -------------------------
    start_speed_sps: float = 8.0   # crawl speed a stepper can start/stop at
    pulse_width_us: float = 5.0    # step pulse high time; A4988/DRV8825 need >= 1-2 us
    dir_setup_us: float = 20.0     # settle time after a direction change
    enable_active_low: bool = True  # true for A4988 / DRV8825 / TMC2209
    segment_ms: float = 20.0       # motion handed to hardware per segment

    # -- endstop / homing ---------------------------------------------------
    endstop_normally_closed: bool = True  # NC fails safe: a cut wire reads as a trip
    endstop_debounce_ms: float = 5.0
    home_backoff_deg: float = 1.0  # back off and re-approach slowly for repeatability

    @property
    def steps_per_deg(self) -> float:
        return self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0


@dataclass
class RotatorConfig:
    #: auto | lgpio | pio | simulated | kinematic  ("gpio" is a deprecated alias for lgpio)
    backend: str = "auto"
    gpiochip: int = -1              # -1 = find the header chip by label
    estop_pin: int = -1             # BCM pin, wired normally-closed to ground
    require_homing: bool | None = None  # None = on for hardware, off for simulation
    idle_disable_s: float = 0.0     # drop the enable line after this long at rest (0 = never)
    watchdog_s: float = 5.0         # stop if whatever is steering stops heartbeating
    realtime_priority: int = 0      # SCHED_FIFO priority for the axis threads (0 = normal)
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


#: How far the corrected frequency may drift before the rig is retuned, by
#: mode. CW ears notice ten hertz; FM does not care until hundreds. Retuning
#: below these thresholds just fills the CAT link with traffic and, on many
#: rigs, clicks the audio.
DEFAULT_DEADBANDS_HZ = {
    "CW": 10.0,
    "CWR": 10.0,
    "USB": 20.0,
    "LSB": 20.0,
    "SSB": 20.0,
    "FM": 200.0,
    "FMN": 200.0,
    "default": 20.0,
}


@dataclass
class RigConfig:
    """Radio control for automatic doppler tuning."""

    backend: str = "none"          # none | rigctld | simulated
    host: str = "127.0.0.1"
    port: int = 4532               # hamlib rigctld; 4533 is rotctld
    timeout_s: float = 2.0
    tx_vfo: str = "VFOB"           # satellite full duplex: RX on the main VFO, TX on this one
    uplink_hz: float = 0.0         # 0 = follow the 2 m doppler panel
    downlink_hz: float = 0.0
    enabled: bool = True           # start tuning as soon as a satellite is tracked
    tune_rx: bool = True
    tune_tx: bool = True
    tune_while_tx: bool = True     # some rigs click when retuned mid-transmission
    max_tune_rate_hz: float = 2.0  # cap on CAT writes per second
    resolution_hz: int = 1         # round to the rig's tuning step
    min_elevation_deg: float = -1.0  # do not tune a satellite that is not up
    mode_poll_s: float = 10.0
    deadbands_hz: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_DEADBANDS_HZ))


@dataclass
class WatchEntry:
    """One satellite on the scheduler's watch list."""

    norad_id: int = 0
    priority: int = 0                     # higher wins when two passes overlap
    min_elevation_deg: float | None = None  # None = use the tracker default


@dataclass
class SchedulerConfig:
    enabled: bool = False
    satellites: list = field(default_factory=list)  # WatchEntry, or dicts from YAML
    lookahead_hours: float = 12.0
    replan_interval_s: float = 600.0
    preposition_lead_s: float = 120.0   # engage this long before AOS
    park_after_idle_s: float = 0.0      # 0 = stay where the last pass left it


@dataclass
class RotctldConfig:
    """Hamlib rotator protocol server, so gpredict et al can drive Orbitaly."""

    enabled: bool = False
    host: str = "0.0.0.0"
    port: int = 4533


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000
    rotctld: RotctldConfig = field(default_factory=RotctldConfig)


@dataclass
class Config:
    station: StationConfig = field(default_factory=StationConfig)
    tle: TleConfig = field(default_factory=TleConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    rotator: RotatorConfig = field(default_factory=RotatorConfig)
    doppler_2m: Doppler2mConfig = field(default_factory=Doppler2mConfig)
    rig: RigConfig = field(default_factory=RigConfig)
    scheduler: SchedulerConfig = field(default_factory=SchedulerConfig)
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
