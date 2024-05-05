"""StepperAxis motion logic against a fake pulse driver (no GPIO, no real time)."""
import time

import pytest

from orbitaly.config import AxisConfig, PinConfig
from orbitaly.hardware.stepper import StepperAxis


class FakeDriver:
    def __init__(self, endstop_after_steps=None):
        self.pulses = 0
        self.direction = True
        self.enabled = False
        self.endstop_after_steps = endstop_after_steps
        self._toward_endstop = 0

    def set_direction(self, forward):
        self.direction = forward

    def pulse(self):
        self.pulses += 1
        if self.endstop_after_steps is not None and not self.direction:
            self._toward_endstop += 1

    def set_enabled(self, enabled):
        self.enabled = enabled

    def endstop_triggered(self):
        return (
            self.endstop_after_steps is not None
            and self._toward_endstop >= self.endstop_after_steps
        )

    def close(self):
        pass


def make_axis(driver=None, **overrides):
    config = AxisConfig(
        min_deg=0.0,
        max_deg=360.0,
        max_speed_dps=90.0,
        accel_dps2=180.0,
        steps_per_rev=200,
        microsteps=1,
        gear_ratio=10.0,  # 5.555 steps/deg
        pins=PinConfig(),
    )
    for key, value in overrides.items():
        setattr(config, key, value)
    axis = StepperAxis(config, driver or FakeDriver(), name="test", sleep_fn=lambda s: None)
    return axis


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_steps_per_deg():
    axis = make_axis()
    assert axis.config.steps_per_deg == pytest.approx(200 * 1 * 10 / 360)


def test_goto_reaches_target():
    driver = FakeDriver()
    axis = make_axis(driver)
    axis.start()
    try:
        axis.goto(90.0)
        assert wait_until(lambda: not axis.moving)
        assert axis.position_deg == pytest.approx(90.0, abs=0.2)
        assert driver.pulses == pytest.approx(90.0 * axis.config.steps_per_deg, abs=1)
        assert driver.enabled
    finally:
        axis.close()


def test_goto_clamps_to_limits():
    axis = make_axis()
    axis.start()
    try:
        axis.goto(9999.0)
        assert axis.target_deg == pytest.approx(360.0)
        axis.goto(-45.0)
        assert axis.target_deg == pytest.approx(0.0)
    finally:
        axis.close()


def test_retarget_mid_move_reverses():
    axis = make_axis()
    axis.start()
    try:
        axis.goto(180.0)
        assert wait_until(lambda: axis.position_deg > 30.0)
        axis.goto(10.0)
        assert wait_until(lambda: not axis.moving)
        assert axis.position_deg == pytest.approx(10.0, abs=0.2)
    finally:
        axis.close()


def test_home_without_endstop_zeroes():
    axis = make_axis(home_position_deg=0.0)
    axis.start()
    try:
        axis.home()
        assert wait_until(lambda: axis.homed)
        assert axis.position_deg == pytest.approx(0.0)
        assert not axis.moving
    finally:
        axis.close()


def test_home_with_endstop():
    driver = FakeDriver(endstop_after_steps=50)
    axis = make_axis(driver, pins=PinConfig(step=1, dir=2, endstop=3))
    axis.start()
    try:
        axis.goto(0.0)  # ensure idle
        axis.home()
        assert wait_until(lambda: axis.homed)
        # Homing lands exactly at min travel
        assert axis.position_deg == pytest.approx(axis.config.min_deg)
        assert axis.fault == ""
    finally:
        axis.close()


def test_backlash_pulses_do_not_move_position():
    driver = FakeDriver()
    axis = make_axis(driver, backlash_deg=1.0)
    axis.start()
    try:
        axis.goto(10.0)
        assert wait_until(lambda: not axis.moving)
        pulses_forward = driver.pulses
        axis.goto(5.0)  # direction reversal: backlash takeup fires
        assert wait_until(lambda: not axis.moving)
        assert axis.position_deg == pytest.approx(5.0, abs=0.2)
        extra = driver.pulses - pulses_forward
        expected_motion = 5.0 * axis.config.steps_per_deg
        backlash = int(round(1.0 * axis.config.steps_per_deg))
        assert extra == pytest.approx(expected_motion + backlash, abs=2)
    finally:
        axis.close()
