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

BACKENDS = ("auto", "serial", "lgpio", "pio", "simulated", "kinematic")
#: accepted for configs written against v0.1, when the only real backend was RPi.GPIO
DEPRECATED_ALIASES = {"gpio": "lgpio"}

#: Substrings that mark a USB serial device as plausibly *our* Arduino, matched
#: against the /dev/serial/by-id name (which carries the USB product strings).
#:
#: This filter is the whole reason ``auto`` is safe to leave on. A ground station
#: has other USB serial devices — a rig CAT cable is the obvious one — and
#: probing means opening the port (which resets an Arduino) and writing a HELLO
#: at it. Doing that to a radio to find out what it is would be rude at best.
#: Anything not matching here has to be named explicitly in ``rotator.serial.port``.
FIRMWARE_PORT_MARKERS = ("arduino", "genuino")


@dataclass
class HardwareReport:
    model: str = ""
    is_raspberry_pi: bool = False
    gpiochips: list[str] = field(default_factory=list)
    lgpio_installed: bool = False
    serial_ports: list[str] = field(default_factory=list)
    pyserial_installed: bool = False
    #: filled in only by an explicit handshake — probing is a side effect, so it
    #: never happens as part of a plain probe()
    serial_ident: object | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def can_lgpio(self) -> bool:
        return self.lgpio_installed and bool(self.gpiochips)

    @property
    def likely_firmware_ports(self) -> list[str]:
        return [
            port
            for port in self.serial_ports
            if any(marker in port.lower() for marker in FIRMWARE_PORT_MARKERS)
        ]


def probe() -> HardwareReport:
    report = HardwareReport()
    try:
        report.model = Path("/proc/device-tree/model").read_text().strip("\x00 \n")
    except OSError:
        report.model = ""
    report.is_raspberry_pi = "raspberry pi" in report.model.lower()
    report.gpiochips = sorted(p.name for p in Path("/dev").glob("gpiochip*"))
    report.lgpio_installed = find_spec("lgpio") is not None
    report.pyserial_installed = find_spec("serial") is not None
    from .serial_driver import probe_serial_ports

    report.serial_ports = probe_serial_ports()

    if report.serial_ports and not report.pyserial_installed:
        report.notes.append(
            "a USB serial device is present but pyserial is not installed — "
            "pip install 'orbitaly[serial]' to use backend: serial"
        )
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
    """Resolve a configured backend name to a concrete one.

    ``auto`` prefers a handshaking Arduino over the Pi's own GPIO, because that
    is the reference deployment — but only among ports that *look* like an
    Arduino (see :data:`FIRMWARE_PORT_MARKERS`), and only when pyserial is
    installed. Any other device has to be named in config, so auto-detection
    can never go poking at a radio.
    """
    requested = DEPRECATED_ALIASES.get(requested, requested)
    if requested not in BACKENDS:
        raise ValueError(f"Unknown rotator backend {requested!r}; expected one of {BACKENDS}")
    if requested != "auto":
        return requested
    report = report or probe()
    if report.pyserial_installed and report.likely_firmware_ports:
        return "serial"
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
    if chosen == "serial":
        from .serial_driver import SerialBackend

        backend = SerialBackend(config, clock=clock)
    elif chosen == "lgpio":
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
        require_homing = chosen in ("serial", "lgpio", "pio")

    rotator = StepperRotator(config, backend, clock=clock, require_homing=require_homing)
    log.info("Rotator backend: %s", backend.describe())
    for note in report.notes:
        log.info("Hardware note: %s", note)
    rotator.start()
    return rotator
