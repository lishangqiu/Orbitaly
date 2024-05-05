"""Optional: run the lgpio path against a *real* kernel gpiochip.

The kernel's ``gpio-sim`` module creates virtual gpiochips through configfs.
They appear as genuine ``/dev/gpiochipN`` devices, so lgpio talks to them
exactly as it will talk to the Pi's RP1 lines — claiming, writing and reading
go through the real character-device ioctls rather than a Python fake.

This needs ``CONFIG_GPIO_SIM`` and write access to configfs (root), so it is
marked ``gpiosim`` and skips cleanly everywhere else. It does not run on WSL2,
whose kernel has no gpio-sim module; run it on the Pi or any normal Linux host:

    sudo modprobe gpio-sim
    sudo .venv/bin/python -m pytest -m gpiosim
"""
from __future__ import annotations

import glob
import os
from pathlib import Path

import pytest

pytestmark = pytest.mark.gpiosim

CONFIGFS = Path("/sys/kernel/config/gpio-sim")


class SimChip:
    """A virtual gpiochip, created and torn down through configfs."""

    def __init__(self, name: str = "orbitaly-test", lines: int = 32):
        self.root = CONFIGFS / name
        self.bank = self.root / "bank0"
        self.bank.mkdir(parents=True)
        (self.bank / "num_lines").write_text(str(lines))
        (self.root / "live").write_text("1")
        self.chip_name = (self.bank / "chip_name").read_text().strip()
        self.number = int(self.chip_name.removeprefix("gpiochip"))

    def value_path(self, offset: int) -> Path:
        matches = glob.glob(
            f"/sys/devices/platform/gpio-sim.*/{self.chip_name}/sim_gpio{offset}/value"
        )
        if not matches:
            raise FileNotFoundError(f"no sysfs value node for line {offset}")
        return Path(matches[0])

    def read_line(self, offset: int) -> int:
        return int(self.value_path(offset).read_text().strip())

    def close(self) -> None:
        try:
            (self.root / "live").write_text("0")
        finally:
            self.bank.rmdir()
            self.root.rmdir()


@pytest.fixture
def sim_chip():
    if not CONFIGFS.is_dir():
        pytest.skip("gpio-sim not available (needs CONFIG_GPIO_SIM and configfs mounted)")
    if not os.access(CONFIGFS, os.W_OK):
        pytest.skip("gpio-sim configfs is not writable — run as root")
    chip = SimChip()
    try:
        yield chip
    finally:
        chip.close()


def test_lgpio_drives_a_real_kernel_gpiochip(sim_chip):
    pytest.importorskip("lgpio")
    from orbitaly.config import AxisConfig, PinConfig
    from orbitaly.motion.lgpio_driver import LgpioAxisDriver, LgpioChip
    from orbitaly.motion.segment import Segment

    config = AxisConfig(pins=PinConfig(step=0, dir=1, enable=2, endstop=-1))
    chip = LgpioChip(sim_chip.number)
    driver = LgpioAxisDriver(config, chip)
    try:
        assert sim_chip.read_line(2) == 1, "enable is active-low, so it idles high"
        driver.set_enabled(True)
        assert sim_chip.read_line(2) == 0

        driver._set_dir(True)
        assert sim_chip.read_line(1) == 1
        driver._set_dir(False)
        assert sim_chip.read_line(1) == 0

        # A real pulse train through the real library.
        driver._hand_off(Segment(1, 20, 2000))
        assert chip.lgpio.tx_busy(chip.handle, 0, chip.lgpio.TX_PWM)
    finally:
        driver.close()
        chip.close()
