"""A stand-in for the ``lgpio`` module that enforces a real Pi's rules.

The point is not to let the code under test pass — it is to make it fail the
way a Raspberry Pi would:

- writing or pulsing a line that was never claimed is an error
- claiming a line twice is an error
- claiming a line the kernel has given to SPI/I2C/UART fails with "GPIO busy",
  which is the single most common surprise when moving from RPi.GPIO to the
  gpiochip interface
- a step pulse narrower than the driver chip's minimum, or a direction change
  while pulses are still going out, is recorded as a violation — those do not
  raise on real hardware, they just silently cost you steps

Pulses are handed to an attached mechanical model, so a test can drive the
genuine :class:`~orbitaly.motion.lgpio_driver.LgpioAxisDriver` against a
virtual rotator and check where the antenna actually ended up.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orbitaly.motion.segment import Segment

SET_PULL_UP = 32
SET_PULL_DOWN = 64
SET_PULL_NONE = 128
BOTH_EDGES = 3
RISING_EDGE = 1
FALLING_EDGE = 2
TX_PWM = 0
TX_WAVE = 1


class error(Exception):  # noqa: N801 - mirrors lgpio's own lowercase name
    pass


@dataclass
class _Pulse:
    gpio: int
    on_us: int
    off_us: int
    cycles: int
    start: float
    end: float
    direction: int


@dataclass
class _Line:
    mode: str  # "output" | "alert" | "input"
    level: int = 0
    debounce_us: int = 0


@dataclass
class FakeLgpio:
    """Module-shaped fake. Install it with :meth:`install`."""

    clock: object
    chips: tuple[int, ...] = (0,)
    #: lines the kernel has already given to another driver
    busy_pins: tuple[int, ...] = ()
    #: step/dir driver chip minimums, from the datasheet
    min_pulse_us: float = 1.0
    min_dir_setup_us: float = 0.2
    queue_depth: int = 8

    # wiring: step pin -> (mechanics, dir pin)
    axes: dict[int, tuple[object, int]] = field(default_factory=dict)

    lines: dict[int, _Line] = field(default_factory=dict)
    queues: dict[int, list[_Pulse]] = field(default_factory=dict)
    callbacks: list[tuple[int, object]] = field(default_factory=list)
    violations: list[str] = field(default_factory=list)
    pulses_emitted: int = 0
    _open: set[int] = field(default_factory=set)
    _last_dir_change: dict[int, float] = field(default_factory=dict)

    # -- module constants ---------------------------------------------------
    SET_PULL_UP = SET_PULL_UP
    SET_PULL_DOWN = SET_PULL_DOWN
    SET_PULL_NONE = SET_PULL_NONE
    BOTH_EDGES = BOTH_EDGES
    RISING_EDGE = RISING_EDGE
    FALLING_EDGE = FALLING_EDGE
    TX_PWM = TX_PWM
    TX_WAVE = TX_WAVE
    error = error

    def install(self, monkeypatch) -> "FakeLgpio":
        import sys

        monkeypatch.setitem(sys.modules, "lgpio", self)
        return self

    def attach_axis(self, step_pin: int, mechanics, dir_pin: int) -> None:
        self.axes[step_pin] = (mechanics, dir_pin)

    # -- chip ---------------------------------------------------------------

    def gpiochip_open(self, chip: int) -> int:
        if chip not in self.chips:
            raise error(f"can not open gpiochip{chip}")
        self._open.add(chip)
        return 1000 + chip

    def gpiochip_close(self, handle: int) -> None:
        self._open.discard(handle - 1000)

    # -- lines --------------------------------------------------------------

    def gpio_claim_output(self, handle: int, gpio: int, level: int = 0) -> None:
        self._claim(gpio, "output")
        self.lines[gpio] = _Line("output", level)

    def gpio_claim_input(self, handle: int, gpio: int, lFlags: int = 0) -> None:  # noqa: N803
        self._claim(gpio, "input")
        self.lines[gpio] = _Line("input", 1 if lFlags == SET_PULL_UP else 0)

    def gpio_claim_alert(self, handle: int, gpio: int, eFlags: int, lFlags: int = 0) -> None:  # noqa: N803
        self._claim(gpio, "alert")
        self.lines[gpio] = _Line("alert", 1 if lFlags == SET_PULL_UP else 0)

    def gpio_set_debounce_micros(self, handle: int, gpio: int, micros: int) -> None:
        self._require(gpio)
        self.lines[gpio].debounce_us = micros

    def gpio_free(self, handle: int, gpio: int) -> None:
        self.lines.pop(gpio, None)

    def gpio_write(self, handle: int, gpio: int, level: int) -> None:
        line = self._require(gpio)
        if line.mode != "output":
            raise error(f"GPIO {gpio} is not an output")
        if line.level != level and self._is_dir_pin(gpio):
            if self._pulsing_on_axis_of(gpio):
                self.violations.append(
                    f"direction changed on GPIO {gpio} while step pulses were in flight"
                )
            self._last_dir_change[gpio] = self.clock.monotonic()
        line.level = level

    def gpio_read(self, handle: int, gpio: int) -> int:
        return self._require(gpio).level

    # -- test-side line control --------------------------------------------

    def set_input(self, gpio: int, level: int) -> None:
        """Drive an input from outside, as a switch or an E-stop would."""
        line = self._require(gpio)
        if line.level == level:
            return
        line.level = level
        for cb_gpio, func in list(self.callbacks):
            if cb_gpio == gpio:
                func(0, gpio, level, self.clock.monotonic())

    # -- pulse train --------------------------------------------------------

    def tx_pulse(
        self, handle: int, gpio: int, on_us: int, off_us: int, offset: int = 0, cycles: int = 0
    ) -> int:
        line = self._require(gpio)
        if line.mode != "output":
            raise error(f"GPIO {gpio} is not an output")
        queue = self.queues.setdefault(gpio, [])
        if len(queue) >= self.queue_depth:
            raise error("PWM queue full")
        if on_us < self.min_pulse_us:
            self.violations.append(
                f"step pulse of {on_us}us on GPIO {gpio} is below the driver minimum "
                f"of {self.min_pulse_us}us"
            )
        direction = self._direction_for(gpio)
        self._check_dir_setup(gpio)
        now = self.clock.monotonic()
        start = max(now, queue[-1].end if queue else now)
        duration = cycles * (on_us + off_us) / 1e6
        queue.append(_Pulse(gpio, on_us, off_us, cycles, start, start + duration, direction))
        return self.queue_depth - len(queue)

    def tx_pwm(self, handle: int, gpio: int, freq: float, duty: float, *args) -> int:
        self._require(gpio)
        if freq == 0:
            self.queues[gpio] = []  # documented way to halt a train
        return self.queue_depth

    def tx_busy(self, handle: int, gpio: int, kind: int = TX_PWM) -> int:
        self.advance()
        return 1 if self.queues.get(gpio) else 0

    def tx_room(self, handle: int, gpio: int, kind: int = TX_PWM) -> int:
        self.advance()
        return self.queue_depth - len(self.queues.get(gpio, []))

    def callback(self, handle: int, gpio: int, edge: int, func):
        entry = (gpio, func)
        self.callbacks.append(entry)

        class _Handle:
            def cancel(_self) -> None:  # noqa: N805
                if entry in self.callbacks:
                    self.callbacks.remove(entry)

        return _Handle()

    # -- clock --------------------------------------------------------------

    def advance(self) -> None:
        """Retire finished pulse trains and turn the motors."""
        now = self.clock.monotonic()
        for gpio, queue in self.queues.items():
            while queue and queue[0].end <= now:
                pulse = queue.pop(0)
                self.pulses_emitted += pulse.cycles
                axis = self.axes.get(gpio)
                if axis is None:
                    continue
                mechanics, _ = axis
                period = max(1, pulse.on_us + pulse.off_us)
                mechanics.apply(
                    Segment(pulse.direction or 1, pulse.cycles, period)
                )
        self.sync_switches()

    def sync_switches(self) -> None:
        """Endstops are wired to inputs; a moving axis can close one."""
        for mechanics, _dir_pin in self.axes.values():
            endstop = getattr(mechanics, "endstop_pin", None)
            if endstop is None or endstop not in self.lines:
                continue
            # Normally-closed to ground: closed reads 0, tripped/open reads 1.
            self.set_input(endstop, 1 if mechanics.endstop_triggered() else 0)

    # -- internals ----------------------------------------------------------

    def _claim(self, gpio: int, mode: str) -> None:
        if gpio in self.busy_pins:
            raise error(f"GPIO busy: {gpio} is owned by another driver (SPI/I2C/UART overlay?)")
        if gpio in self.lines:
            raise error(f"GPIO not allocated: {gpio} is already claimed")

    def _require(self, gpio: int) -> _Line:
        line = self.lines.get(gpio)
        if line is None:
            raise error(f"GPIO not allocated: {gpio} was never claimed")
        return line

    def _is_dir_pin(self, gpio: int) -> bool:
        return any(dir_pin == gpio for _, dir_pin in self.axes.values())

    def _pulsing_on_axis_of(self, dir_pin: int) -> bool:
        self.advance()
        for step_pin, (_, pin) in self.axes.items():
            if pin == dir_pin and self.queues.get(step_pin):
                return True
        return False

    def _direction_for(self, step_pin: int) -> int:
        axis = self.axes.get(step_pin)
        if axis is None:
            return 1
        _, dir_pin = axis
        line = self.lines.get(dir_pin)
        return 1 if line and line.level else -1

    def _check_dir_setup(self, step_pin: int) -> None:
        axis = self.axes.get(step_pin)
        if axis is None:
            return
        _, dir_pin = axis
        changed = self._last_dir_change.get(dir_pin)
        if changed is None:
            return
        elapsed_us = (self.clock.monotonic() - changed) * 1e6
        if elapsed_us + 1e-9 < self.min_dir_setup_us:
            self.violations.append(
                f"pulsed {elapsed_us:.2f}us after a direction change on GPIO {dir_pin}; "
                f"the driver needs {self.min_dir_setup_us}us"
            )
