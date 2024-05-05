"""The real Pi code path, against a fake that enforces a real Pi's rules.

Nothing here is simulated above the lgpio call boundary: this is the same
:class:`LgpioAxisDriver` that will run on the Pi 5, claiming lines, queueing
pulse trains and reading an endstop alert.
"""
import pytest

from orbitaly.config import AxisConfig, PinConfig
from orbitaly.motion.axis import StepperAxis
from orbitaly.motion.clock import VirtualClock
from orbitaly.motion.lgpio_driver import LgpioAxisDriver, LgpioChip

from fakes.fake_lgpio import FakeLgpio
from fakes.mechanics import VirtualAxis

STEP, DIR, ENABLE, ENDSTOP = 17, 27, 22, 5
STEPS_PER_DEG = 200 * 1 * 10 / 360.0


def make_config(**overrides) -> AxisConfig:
    config = AxisConfig(
        min_deg=-10.0,
        max_deg=360.0,
        max_speed_dps=90.0,
        accel_dps2=180.0,
        steps_per_rev=200,
        microsteps=1,
        gear_ratio=10.0,
        pins=PinConfig(step=STEP, dir=DIR, enable=ENABLE, endstop=ENDSTOP),
    )
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


class Stack:
    """Supervisor -> real lgpio driver -> fake lgpio -> virtual rotator."""

    def __init__(self, monkeypatch, config=None, mechanics=None, **fake_kwargs):
        self.clock = VirtualClock()
        self.fake = FakeLgpio(clock=self.clock, **fake_kwargs).install(monkeypatch)
        self.config = config or make_config()
        self.chip = LgpioChip(0)
        self.driver = LgpioAxisDriver(self.config, self.chip, clock=self.clock)
        self.mechanics = mechanics or VirtualAxis(steps_per_deg=STEPS_PER_DEG)
        self.mechanics.endstop_pin = self.config.pins.endstop
        self.fake.attach_axis(STEP, self.mechanics, DIR)
        self.fake.sync_switches()
        self.axis = StepperAxis(self.config, self.driver, name="az", clock=self.clock)
        self.axis._set_enabled(True)

    def run(self, seconds: float, tick: float = 0.002) -> None:
        for _ in range(max(1, int(seconds / tick))):
            self.clock.advance(tick)
            self.fake.advance()
            self.axis.tick()

    def run_until(self, predicate, timeout_s: float = 60.0, tick: float = 0.002) -> bool:
        elapsed = 0.0
        while elapsed < timeout_s:
            self.clock.advance(tick)
            self.fake.advance()
            self.axis.tick()
            elapsed += tick
            if predicate():
                return True
        return False

    def settle(self, timeout_s: float = 60.0) -> bool:
        return self.run_until(lambda: not self.axis.moving, timeout_s)


# -- line claiming ----------------------------------------------------------


def test_claiming_a_line_the_kernel_already_owns_fails_loudly(monkeypatch):
    """The classic gpiochip surprise: enable SPI, and GPIO 10/11 are gone.

    RPi.GPIO would happily stomp on them; the character-device interface will
    not, so the failure has to be legible.
    """
    FakeLgpio(clock=VirtualClock(), busy_pins=(STEP,)).install(monkeypatch)
    chip = LgpioChip(0)
    with pytest.raises(Exception, match="GPIO busy"):
        LgpioAxisDriver(make_config(), chip, clock=VirtualClock())


def test_two_axes_sharing_a_pin_is_caught_at_startup(monkeypatch):
    FakeLgpio(clock=VirtualClock()).install(monkeypatch)
    chip = LgpioChip(0)
    LgpioAxisDriver(make_config(), chip, clock=VirtualClock())
    with pytest.raises(ValueError, match="claimed twice"):
        LgpioAxisDriver(make_config(), chip, clock=VirtualClock())


def test_enable_line_is_active_low_by_default(monkeypatch):
    stack = Stack(monkeypatch)
    assert stack.fake.lines[ENABLE].level == 0  # pulled low = driver enabled
    stack.driver.set_enabled(False)
    assert stack.fake.lines[ENABLE].level == 1


def test_enable_polarity_is_configurable(monkeypatch):
    stack = Stack(monkeypatch, config=make_config(enable_active_low=False))
    stack.driver.set_enabled(True)
    assert stack.fake.lines[ENABLE].level == 1


# -- pulse trains -----------------------------------------------------------


def test_a_move_emits_exactly_the_right_number_of_pulses(monkeypatch):
    stack = Stack(monkeypatch)
    stack.axis.goto(90.0)
    assert stack.settle()
    expected = round(90.0 * STEPS_PER_DEG)
    assert stack.fake.pulses_emitted == expected
    assert stack.mechanics.position_deg == pytest.approx(90.0, abs=1 / STEPS_PER_DEG)


def test_no_electrical_contract_violations_on_a_reversal(monkeypatch):
    """Pulse width and direction setup time are the two things that silently
    cost steps on real hardware, so the fake watches both."""
    stack = Stack(monkeypatch)
    stack.axis.goto(40.0)
    assert stack.settle()
    stack.axis.goto(5.0)
    assert stack.settle()
    assert stack.fake.violations == []


def test_pulse_width_below_the_driver_minimum_is_reported(monkeypatch):
    """Guards the guard: if the driver ever emitted a 0 us pulse we would know."""
    stack = Stack(monkeypatch, config=make_config(pulse_width_us=0.0), min_pulse_us=2.0)
    stack.axis.goto(2.0)
    stack.settle()
    assert any("below the driver minimum" in v for v in stack.fake.violations)


def test_direction_pin_is_never_flipped_under_live_pulses(monkeypatch):
    stack = Stack(monkeypatch)
    for target in (30.0, 2.0, 25.0, 1.0):
        stack.axis.goto(target)
        stack.run(0.3)
    stack.settle()
    assert not any("while step pulses were in flight" in v for v in stack.fake.violations)


def test_abort_halts_the_pulse_train(monkeypatch):
    stack = Stack(monkeypatch)
    stack.axis.goto(300.0)
    stack.run(0.3)
    assert stack.fake.queues[STEP]
    stack.axis.emergency_stop("test")
    assert stack.fake.queues[STEP] == []
    before = stack.mechanics.position_deg
    stack.run(0.5)
    assert stack.mechanics.position_deg == pytest.approx(before)


# -- endstop ----------------------------------------------------------------


def test_endstop_alert_stops_the_axis_mid_move(monkeypatch):
    mechanics = VirtualAxis(steps_per_deg=STEPS_PER_DEG, endstop_deg=-5.0, hard_min_deg=-8.0)
    stack = Stack(monkeypatch, mechanics=mechanics)
    stack.axis.goto(-9.0)
    assert stack.run_until(lambda: stack.axis.fault != "")
    assert "endstop" in stack.axis.fault
    assert mechanics.steps_lost == 0, "stopped before the hard stop"


def test_normally_closed_endstop_reads_untripped_when_the_switch_is_closed(monkeypatch):
    mechanics = VirtualAxis(steps_per_deg=STEPS_PER_DEG, endstop_deg=-5.0)
    stack = Stack(monkeypatch, mechanics=mechanics)
    assert stack.fake.lines[ENDSTOP].level == 0
    assert not stack.driver.endstop_triggered()


def test_a_disconnected_normally_closed_endstop_reads_as_tripped(monkeypatch):
    """NC wiring is recommended precisely because this is the failure mode:
    a broken wire looks like a limit, not like open sky."""
    stack = Stack(monkeypatch)
    stack.fake.set_input(ENDSTOP, 1)  # pull-up wins when the loop is broken
    assert stack.driver.endstop_triggered()


def test_endstop_debounce_is_configured_on_the_line(monkeypatch):
    stack = Stack(monkeypatch, config=make_config(endstop_debounce_ms=7.0))
    assert stack.fake.lines[ENDSTOP].debounce_us == 7000


# -- homing over real driver calls ------------------------------------------


def test_homing_runs_end_to_end_through_the_driver(monkeypatch):
    mechanics = VirtualAxis(
        steps_per_deg=STEPS_PER_DEG, position_deg=25.0, endstop_deg=0.0, hard_min_deg=-4.0
    )
    stack = Stack(monkeypatch, config=make_config(min_deg=0.0), mechanics=mechanics)
    stack.axis.home()
    assert stack.run_until(lambda: stack.axis.homed, timeout_s=60.0)
    assert stack.axis.position_deg == pytest.approx(0.0)
    assert mechanics.position_deg == pytest.approx(0.0, abs=0.2)
    assert mechanics.steps_lost == 0
    assert stack.fake.violations == []
