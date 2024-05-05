"""``orbitaly doctor`` — say exactly what this machine can and cannot do.

Written because the worst failure mode of an auto-detecting backend is a
silent downgrade: you wire up a rotator, press Track, watch nothing move, and
have no idea the software quietly chose the simulator. Everything it decided,
and why, is printed here.
"""
from __future__ import annotations

import grp
import os
import pwd
from pathlib import Path

from ..config import Config, load_config
from ..motion.detect import probe, select_backend

OK, WARN, BAD = "ok  ", "warn", "FAIL"

#: Pins the standard overlays take. Claiming one of these fails with
#: "GPIO busy" on the character-device interface, where RPi.GPIO would
#: silently have stomped on it.
OVERLAY_PINS = {
    "spi0": (7, 8, 9, 10, 11),
    "i2c1": (2, 3),
    "uart0": (14, 15),
}


def run(config_path: str | None = None) -> int:
    config: Config = load_config(config_path)
    report = probe()
    lines: list[tuple[str, str]] = []

    lines.append((OK if report.model else WARN, f"Model: {report.model or 'unknown (not a Pi?)'}"))
    for note in report.notes:
        lines.append((WARN, note))

    lines.append(
        (OK if report.gpiochips else WARN, f"gpiochips: {', '.join(report.gpiochips) or 'none'}")
    )
    lgpio_note = "installed" if report.lgpio_installed else "missing — pip install 'orbitaly[pi]'"
    lines.append((OK if report.lgpio_installed else WARN, f"lgpio module: {lgpio_note}"))
    lines.append(
        (
            OK if report.pio_device else WARN,
            f"/dev/pio0 (RP1 PIO): {'present' if report.pio_device else 'absent'}"
            + ("" if report.pio_binding else " · no Python binding installed"),
        )
    )

    lines += _permission_lines(report.gpiochips)

    requested = config.rotator.backend
    chosen = select_backend(requested, report)
    severity = OK
    if chosen in ("simulated", "kinematic") and report.is_raspberry_pi:
        severity = WARN
    lines.append((severity, f"Rotator backend: {requested} -> {chosen}"))
    if chosen == "simulated" and report.is_raspberry_pi:
        lines.append((WARN, "  This is a Pi but the simulator was selected — nothing will move."))

    lines += _pin_lines(config)
    lines += _travel_lines(config)
    lines.append(
        (
            OK,
            f"Homing required before motion: "
            f"{'yes' if _require_homing(config, chosen) else 'no'}",
        )
    )
    lines.append(
        (
            OK if config.rig.backend != "none" else WARN,
            f"Rig control: {config.rig.backend}"
            + (f" at {config.rig.host}:{config.rig.port}" if config.rig.backend == "rigctld" else ""),
        )
    )
    lines.append(
        (
            OK,
            f"rotctld server: {'enabled on port ' + str(config.server.rotctld.port) if config.server.rotctld.enabled else 'disabled'}",
        )
    )

    worst = OK
    for level, text in lines:
        print(f"[{level}] {text}")
        if level == BAD or (level == WARN and worst == OK):
            worst = level
    print()
    print(
        {
            OK: "Looks good.",
            WARN: "Usable, but check the warnings above before trusting it with an antenna.",
            BAD: "Not ready — fix the failures above.",
        }[worst]
    )
    return 0 if worst != BAD else 1


def _require_homing(config: Config, chosen: str) -> bool:
    if config.rotator.require_homing is None:
        return chosen in ("lgpio", "pio")
    return bool(config.rotator.require_homing)


def _permission_lines(gpiochips: list[str]) -> list[tuple[str, str]]:
    if not gpiochips:
        return []
    device = Path("/dev") / gpiochips[0]
    if os.access(device, os.R_OK | os.W_OK):
        return [(OK, f"Access to {device}: read/write")]
    try:
        groups = {grp.getgrgid(gid).gr_name for gid in os.getgroups()}
        user = pwd.getpwuid(os.getuid()).pw_name
    except (KeyError, OSError):
        groups, user = set(), "this user"
    hint = "" if "gpio" in groups else f" — try: sudo usermod -aG gpio {user}, then log out and in"
    return [(BAD, f"No access to {device}{hint}")]


def _pin_lines(config: Config) -> list[tuple[str, str]]:
    lines: list[tuple[str, str]] = []
    used: dict[int, str] = {}
    axes = {"azimuth": config.rotator.azimuth, "elevation": config.rotator.elevation}
    for axis_name, axis in axes.items():
        for role in ("step", "dir", "enable", "endstop"):
            pin = getattr(axis.pins, role)
            if pin < 0:
                continue
            label = f"{axis_name}.{role}"
            if pin in used:
                lines.append((BAD, f"GPIO {pin} is assigned to both {used[pin]} and {label}"))
            used[pin] = label
            for overlay, pins in OVERLAY_PINS.items():
                if pin in pins:
                    lines.append(
                        (WARN, f"GPIO {pin} ({label}) belongs to {overlay} when that overlay is on")
                    )
    if config.rotator.estop_pin >= 0:
        used[config.rotator.estop_pin] = "estop"
    if used:
        lines.append((OK, f"Pins in use: {', '.join(f'{p}={r}' for p, r in sorted(used.items()))}"))
    else:
        lines.append((WARN, "No GPIO pins configured — a hardware backend has nothing to drive"))
    if config.rotator.estop_pin < 0:
        lines.append((WARN, "No E-stop pin configured (rotator.estop_pin)"))
    for axis_name, axis in axes.items():
        if axis.pins.endstop < 0:
            lines.append(
                (WARN, f"{axis_name} has no endstop — homing will just accept the current position")
            )
    return lines


def _travel_lines(config: Config) -> list[tuple[str, str]]:
    lines = []
    for name, axis in (("azimuth", config.rotator.azimuth), ("elevation", config.rotator.elevation)):
        top_speed = axis.max_speed_dps * axis.steps_per_deg
        lines.append(
            (
                OK,
                f"{name}: {axis.min_deg:g}..{axis.max_deg:g} deg, {axis.steps_per_deg:.1f} steps/deg, "
                f"{axis.max_speed_dps:g} deg/s = {top_speed:.0f} steps/s "
                f"({1e6 / top_speed:.0f} us per step)",
            )
        )
        if top_speed > 20_000:
            lines.append(
                (WARN, f"  {name} top step rate is high for software timing; reduce microsteps")
            )
    span = config.rotator.azimuth.max_deg - config.rotator.azimuth.min_deg
    if span < 400:
        lines.append(
            (WARN, "Azimuth travel under 400 deg: north-crossing passes will need a wrap slew")
        )
    return lines
