"""Backends: the per-machine resources an axis driver needs.

A backend owns whatever is shared between the two axes — an open gpiochip
handle, the E-stop input — and hands out one :class:`AxisDriver` per axis.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Callable

from ..config import AxisConfig, RotatorConfig
from .driver import AxisDriver
from .sim_driver import Mechanics, SimulatedAxisDriver

log = logging.getLogger(__name__)


class Backend(ABC):
    name = "backend"

    @abstractmethod
    def axis_driver(self, config: AxisConfig, name: str) -> AxisDriver: ...

    def estop_triggered(self) -> bool:
        return False

    def arm_estop(self, callback: Callable[[], None] | None) -> None:
        pass

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return self.name


class SimulatedBackend(Backend):
    """No hardware. Optionally attaches a mechanical model per axis."""

    name = "simulated"

    def __init__(self, mechanics: dict[str, Mechanics] | None = None, *, clock=None):
        self.mechanics = mechanics or {}
        self.clock = clock
        self.drivers: dict[str, SimulatedAxisDriver] = {}

    def axis_driver(self, config: AxisConfig, name: str) -> AxisDriver:
        driver = SimulatedAxisDriver(
            mechanics=self.mechanics.get(name),
            dir_setup_s=config.dir_setup_us / 1e6,
            clock=self.clock,
        )
        self.drivers[name] = driver
        return driver

    def describe(self) -> str:
        return "simulated (no hardware)"


class LgpioBackend(Backend):
    """Raspberry Pi via the kernel gpiochip character device."""

    name = "lgpio"

    def __init__(self, config: RotatorConfig, *, clock=None):
        from .lgpio_driver import LgpioAxisDriver, LgpioChip

        self._driver_cls = LgpioAxisDriver
        self.config = config
        self.clock = clock
        self.chip = LgpioChip(config.gpiochip if config.gpiochip >= 0 else None)
        self._estop_cb: Callable[[], None] | None = None
        self._estop_handle = None
        if config.estop_pin >= 0:
            self.chip.claim_alert(config.estop_pin, pull_up=True, debounce_us=2000)

    def axis_driver(self, config: AxisConfig, name: str) -> AxisDriver:
        return self._driver_cls(config, self.chip, clock=self.clock)

    def estop_triggered(self) -> bool:
        if self.config.estop_pin < 0:
            return False
        lgpio = self.chip.lgpio
        level = lgpio.gpio_read(self.chip.handle, self.config.estop_pin)
        # Wire E-stop as normally-closed to ground: released reads 0, pressed
        # (or unplugged, or broken) reads 1.
        return level == 1

    def arm_estop(self, callback: Callable[[], None] | None) -> None:
        self._estop_cb = callback
        if self.config.estop_pin < 0 or self._estop_handle is not None:
            return
        lgpio = self.chip.lgpio
        self._estop_handle = lgpio.callback(
            self.chip.handle, self.config.estop_pin, lgpio.BOTH_EDGES, self._on_edge
        )

    def _on_edge(self, chip, gpio, level, timestamp) -> None:  # noqa: ARG002 - lgpio signature
        if self._estop_cb is not None and self.estop_triggered():
            self._estop_cb()

    def close(self) -> None:
        if self._estop_handle is not None:
            try:
                self._estop_handle.cancel()
            except Exception:  # noqa: BLE001
                pass
        self.chip.close()

    def describe(self) -> str:
        return f"lgpio on /dev/gpiochip{self.chip.number}"


class PioBackend(Backend):
    """RP1 PIO — hardware-timed pulse trains on a Pi 5.

    Not implemented yet. PIO is the right long-term answer for silent
    high-microstep drives, but the Python path to it (piolib via ctypes, or
    Adafruit's Blinka binding) needs a hardware spike before anything depends
    on it. Until then the selector never picks this backend on its own, and
    asking for it explicitly fails loudly rather than silently downgrading.
    """

    name = "pio"

    def __init__(self, config: RotatorConfig, *, clock=None):
        raise NotImplementedError(
            "The RP1 PIO backend is not implemented yet. Use backend: lgpio "
            "(the default on a Pi 5) — see docs/HARDWARE.md."
        )

    def axis_driver(self, config: AxisConfig, name: str) -> AxisDriver:  # pragma: no cover
        raise NotImplementedError
