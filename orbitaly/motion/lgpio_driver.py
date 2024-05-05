"""lgpio axis backend — the primary hardware path on a Raspberry Pi 5.

Why lgpio: the Pi 5's RP1 southbridge broke every register-poking library.
RPi.GPIO does not work, and pigpio — the DMA waveform backend the original
design counted on — cannot be ported, because it drives BCM peripherals that
no longer own the pins. lgpio talks to the kernel's gpiochip character device,
which is the supported interface on RP1.

``lgpio.tx_pulse`` runs the pulse train in a C thread and takes an exact cycle
count, so the two things that matter are covered: Python is not in the timing
loop, and the number of steps emitted is exactly the number requested. Jitter
lands on the *period* (tens of microseconds against a 625 us azimuth step),
which costs a little velocity ripple and nothing at all in position.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Callable

from ..config import AxisConfig
from .driver import BufferedAxisDriver
from .segment import Segment

log = logging.getLogger(__name__)

#: chip labels for the header pins, newest first
_HEADER_CHIP_LABELS = ("pinctrl-rp1", "pinctrl-bcm2712", "pinctrl-bcm2711", "pinctrl-bcm2835")


def resolve_gpiochip(preferred: int | None = None) -> int:
    """Find the gpiochip that owns the 40-pin header.

    The number moved between kernel releases (the Pi 5 header was gpiochip4 on
    early Bookworm and gpiochip0 from 6.6.45 on), so match on the chip label
    rather than trusting a number. ``orbitaly doctor`` reports what this picked.
    """
    if preferred is not None and preferred >= 0:
        return preferred
    for device in sorted(Path("/sys/bus/gpio/devices").glob("gpiochip*")):
        label_file = device / "label"
        try:
            label = label_file.read_text().strip()
        except OSError:
            continue
        if label in _HEADER_CHIP_LABELS:
            try:
                return int(device.name.removeprefix("gpiochip"))
            except ValueError:
                continue
    log.warning("Could not identify the header gpiochip by label; falling back to 0")
    return 0


class LgpioChip:
    """One open handle to a gpiochip, shared by both axes."""

    def __init__(self, chip: int | None = None):
        import lgpio  # imported lazily: absent on dev machines

        self.lgpio = lgpio
        self.number = resolve_gpiochip(chip)
        self.handle = lgpio.gpiochip_open(self.number)
        self._claimed: set[int] = set()

    def claim_output(self, gpio: int, initial: int = 0) -> None:
        self._guard(gpio)
        self.lgpio.gpio_claim_output(self.handle, gpio, initial)
        self._claimed.add(gpio)

    def claim_alert(self, gpio: int, pull_up: bool, debounce_us: int) -> None:
        self._guard(gpio)
        flags = self.lgpio.SET_PULL_UP if pull_up else self.lgpio.SET_PULL_DOWN
        self.lgpio.gpio_claim_alert(self.handle, gpio, self.lgpio.BOTH_EDGES, lFlags=flags)
        if debounce_us > 0:
            self.lgpio.gpio_set_debounce_micros(self.handle, gpio, debounce_us)
        self._claimed.add(gpio)

    def _guard(self, gpio: int) -> None:
        if gpio in self._claimed:
            raise ValueError(f"GPIO {gpio} is claimed twice — check the pin map in your config")

    def close(self) -> None:
        for gpio in self._claimed:
            try:
                self.lgpio.gpio_free(self.handle, gpio)
            except Exception:  # noqa: BLE001 - teardown must not mask the real error
                pass
        self._claimed.clear()
        try:
            self.lgpio.gpiochip_close(self.handle)
        except Exception:  # noqa: BLE001
            pass


class LgpioAxisDriver(BufferedAxisDriver):
    def __init__(self, config: AxisConfig, chip: LgpioChip, *, clock=None):
        super().__init__(dir_setup_s=config.dir_setup_us / 1e6, clock=clock)
        self.config = config
        self.chip = chip
        self.lgpio = chip.lgpio
        self._pins = config.pins
        self._endstop_callback: Callable[[], None] | None = None
        self._cb_handle = None

        if self._pins.step < 0 or self._pins.dir < 0:
            raise ValueError(
                "This axis has no step/dir pins configured. Set rotator.<axis>.pins "
                "in your config (see config.example.yaml) before using a hardware backend."
            )
        chip.claim_output(self._pins.step, 0)
        chip.claim_output(self._pins.dir, 0)
        if self._pins.enable >= 0:
            # Driver enable lines are active-low on A4988/DRV8825/TMC2209.
            chip.claim_output(self._pins.enable, 0 if not config.enable_active_low else 1)
        if self._pins.endstop >= 0:
            chip.claim_alert(
                self._pins.endstop,
                pull_up=True,
                debounce_us=int(config.endstop_debounce_ms * 1000),
            )

    # -- backend primitives -------------------------------------------------

    def _hand_off(self, segment: Segment) -> None:
        period = segment.period_us
        on = max(1, min(int(self.config.pulse_width_us), period - 1))
        off = max(1, period - on)
        self.lgpio.tx_pulse(self.chip.handle, self._pins.step, on, off, 0, segment.steps)

    def _set_dir(self, forward: bool) -> None:
        level = 1 if (forward != self.config.invert_dir) else 0
        self.lgpio.gpio_write(self.chip.handle, self._pins.dir, level)

    def _stop_pulses(self) -> None:
        try:
            self.lgpio.tx_pwm(self.chip.handle, self._pins.step, 0, 0)
        except Exception:  # noqa: BLE001 - stopping must never raise
            log.exception("Failed to halt pulse train on GPIO %d", self._pins.step)
        try:
            self.lgpio.gpio_write(self.chip.handle, self._pins.step, 0)
        except Exception:  # noqa: BLE001
            pass

    def _hardware_busy(self) -> bool:
        """Ask lgpio, not the clock — flipping dir under live pulses is ruinous."""
        try:
            if self.lgpio.tx_busy(self.chip.handle, self._pins.step, self.lgpio.TX_PWM):
                return True
        except Exception:  # noqa: BLE001
            return True  # unknown state: assume busy, never flip dir on a guess
        return bool(self._in_flight)

    # -- pins ---------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> None:
        if self._pins.enable < 0:
            return
        level = (0 if enabled else 1) if self.config.enable_active_low else (1 if enabled else 0)
        self.lgpio.gpio_write(self.chip.handle, self._pins.enable, level)

    def has_endstop(self) -> bool:
        return self._pins.endstop >= 0

    def endstop_triggered(self) -> bool:
        if self._pins.endstop < 0:
            return False
        level = self.lgpio.gpio_read(self.chip.handle, self._pins.endstop)
        # Switch to ground against the internal pull-up. A normally-closed
        # switch idles at 0 and goes high when it opens — which is also what a
        # cut wire does, so NC wiring fails safe. That is the recommended way
        # to wire this; see docs/HARDWARE.md.
        if self.config.endstop_normally_closed:
            return level == 1
        return level == 0

    def arm_endstop(self, callback: Callable[[], None] | None) -> None:
        self._endstop_callback = callback
        if self._pins.endstop < 0 or self._cb_handle is not None:
            return
        self._cb_handle = self.lgpio.callback(
            self.chip.handle, self._pins.endstop, self.lgpio.BOTH_EDGES, self._on_edge
        )

    def _on_edge(self, chip, gpio, level, timestamp) -> None:  # noqa: ARG002 - lgpio signature
        if self._endstop_callback is not None and self.endstop_triggered():
            self._endstop_callback()

    def close(self) -> None:
        self._stop_pulses()
        self.set_enabled(False)
        if self._cb_handle is not None:
            try:
                self._cb_handle.cancel()
            except Exception:  # noqa: BLE001
                pass
            self._cb_handle = None
