"""Raspberry Pi GPIO backend: two StepperAxis on step/dir/enable pins.

``RPi.GPIO`` is imported lazily so the rest of the codebase runs anywhere.
Install with the ``gpio`` extra:  pip install orbitaly[gpio]
"""
from __future__ import annotations

from ..config import AxisConfig, RotatorConfig
from .base import Rotator, RotatorState
from .stepper import PulseDriver, StepperAxis

PULSE_WIDTH_S = 5e-6  # comfortably above A4988/DRV8825/TMC minimums


class GpioPulseDriver(PulseDriver):
    def __init__(self, config: AxisConfig):
        import RPi.GPIO as GPIO  # noqa: N814

        self._gpio = GPIO
        self._pins = config.pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pins.step, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._pins.dir, GPIO.OUT, initial=GPIO.LOW)
        if self._pins.enable >= 0:
            GPIO.setup(self._pins.enable, GPIO.OUT, initial=GPIO.HIGH)
        if self._pins.endstop >= 0:
            GPIO.setup(self._pins.endstop, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def set_direction(self, forward: bool) -> None:
        self._gpio.output(self._pins.dir, self._gpio.HIGH if forward else self._gpio.LOW)

    def pulse(self) -> None:
        import time

        self._gpio.output(self._pins.step, self._gpio.HIGH)
        time.sleep(PULSE_WIDTH_S)
        self._gpio.output(self._pins.step, self._gpio.LOW)

    def set_enabled(self, enabled: bool) -> None:
        if self._pins.enable >= 0:
            # Most driver EN pins are active-low
            self._gpio.output(self._pins.enable, self._gpio.LOW if enabled else self._gpio.HIGH)

    def endstop_triggered(self) -> bool:
        if self._pins.endstop < 0:
            return False
        return self._gpio.input(self._pins.endstop) == self._gpio.LOW

    def close(self) -> None:
        self._gpio.cleanup([p for p in
                            (self._pins.step, self._pins.dir, self._pins.enable, self._pins.endstop)
                            if p >= 0])


class GpioRotator(Rotator):
    def __init__(self, config: RotatorConfig):
        self.config = config
        self._az = StepperAxis(config.azimuth, GpioPulseDriver(config.azimuth), name="az")
        self._el = StepperAxis(config.elevation, GpioPulseDriver(config.elevation), name="el")
        self._az.start()
        self._el.start()

    def goto(self, azimuth: float, elevation: float) -> None:
        self._az.goto(azimuth)
        self._el.goto(elevation)

    def jog(self, d_azimuth: float, d_elevation: float) -> None:
        self._az.goto(self._az.target_deg + d_azimuth)
        self._el.goto(self._el.target_deg + d_elevation)

    def stop(self) -> None:
        self._az.stop()
        self._el.stop()

    def home(self) -> None:
        self._az.home()
        self._el.home()

    def state(self) -> RotatorState:
        fault = "; ".join(f for f in (self._az.fault, self._el.fault) if f)
        return RotatorState(
            azimuth=self._az.position_deg,
            elevation=self._el.position_deg,
            target_azimuth=self._az.target_deg,
            target_elevation=self._el.target_deg,
            moving=self._az.moving or self._el.moving,
            homed=self._az.homed and self._el.homed,
            fault=fault,
        )

    def close(self) -> None:
        self._az.close()
        self._el.close()

    @property
    def azimuth_travel(self) -> tuple[float, float]:
        return (self.config.azimuth.min_deg, self.config.azimuth.max_deg)

    @property
    def elevation_travel(self) -> tuple[float, float]:
        return (self.config.elevation.min_deg, self.config.elevation.max_deg)


def make_rotator(config: RotatorConfig) -> Rotator:
    if config.backend == "gpio":
        return GpioRotator(config)
    from .simulated import SimulatedRotator

    return SimulatedRotator(config)
