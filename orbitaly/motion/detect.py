"""Which motion backend can this machine actually run?

Detection is explicit and reportable rather than clever: ``orbitaly doctor``
prints exactly what was found and what was chosen, because "it silently fell
back to the simulator" is a bad thing to discover with an antenna attached.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from importlib.util import find_spec
from pathlib import Path

from ..config import RotatorConfig
from ..hardware.base import Rotator
from .backend import Backend, LgpioBackend, PioBackend, SimulatedBackend

log = logging.getLogger(__name__)

BACKENDS = ("auto", "lgpio", "pio", "simulated", "kinematic")
#: accepted for configs written against v0.1, when the only real backend was RPi.GPIO
DEPRECATED_ALIASES = {"gpio": "lgpio"}


@dataclass
class HardwareReport:
    model: str = ""
    is_raspberry_pi: bool = False
    gpiochips: list[str] = field(default_factory=list)
    lgpio_installed: bool = False
    pio_device: bool = False
    pio_binding: bool = False
    notes: list[str] = field(default_factory=list)

    @property
    def can_lgpio(self) -> bool:
        return self.lgpio_installed and bool(self.gpiochips)


def probe() -> HardwareReport:
    report = HardwareReport()
    try:
        report.model = Path("/proc/device-tree/model").read_text().strip("\x00 \n")
    except OSError:
        report.model = ""
    report.is_raspberry_pi = "raspberry pi" in report.model.lower()
    report.gpiochips = sorted(p.name for p in Path("/dev").glob("gpiochip*"))
    report.lgpio_installed = find_spec("lgpio") is not None
    report.pio_device = Path("/dev/pio0").exists()
    report.pio_binding = find_spec("adafruit_rp1pio") is not None

    if report.is_raspberry_pi and not report.lgpio_installed:
        report.notes.append(
            "lgpio is not installed — pip install 'orbitaly[pi5]' (or apt install python3-lgpio)"
        )
    if report.is_raspberry_pi and not report.gpiochips:
        report.notes.append("no /dev/gpiochip* visible — check that the user is in the gpio group")
    if "raspberry pi 5" in report.model.lower():
        report.notes.append(
            "Pi 5 detected: RPi.GPIO and pigpio cannot work here (RP1 southbridge); lgpio is the supported path"
        )
    return report


def select_backend(requested: str, report: HardwareReport | None = None) -> str:
    """Resolve a configured backend name to a concrete one."""
    requested = DEPRECATED_ALIASES.get(requested, requested)
    if requested not in BACKENDS:
        raise ValueError(f"Unknown rotator backend {requested!r}; expected one of {BACKENDS}")
    if requested != "auto":
        return requested
    report = report or probe()
    if report.can_lgpio and report.is_raspberry_pi:
        return "lgpio"
    return "simulated"


def make_rotator(config: RotatorConfig, *, clock=None, mechanics=None) -> Rotator:
    """Build the rotator described by ``config`` for the machine we're on."""
    report = probe()
    chosen = select_backend(config.backend, report)

    if chosen == "kinematic":
        # v0.1's continuous-motion model. Kept because it is a useful pure-UI
        # demo, but it does not exercise the planner, so it is not the default.
        from ..hardware.simulated import SimulatedRotator

        return SimulatedRotator(config)

    backend: Backend
    if chosen == "lgpio":
        backend = LgpioBackend(config, clock=clock)
    elif chosen == "pio":
        backend = PioBackend(config, clock=clock)
    else:
        backend = SimulatedBackend(mechanics=mechanics, clock=clock)

    from .rotator import StepperRotator

    require_homing = config.require_homing
    if require_homing is None:
        # Hardware must prove it knows where it is; a simulator has nothing to
        # crash into, and demanding a homing click there just trains people to
        # click past the interlock.
        require_homing = chosen in ("lgpio", "pio")

    rotator = StepperRotator(config, backend, clock=clock, require_homing=require_homing)
    log.info("Rotator backend: %s", backend.describe())
    for note in report.notes:
        log.info("Hardware note: %s", note)
    rotator.start()
    return rotator
