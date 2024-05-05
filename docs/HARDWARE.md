# Hardware, wiring and commissioning

This is the document to read before connecting Orbitaly to something that can
break itself. It covers what runs where, how to wire it, and the order to
bring it up in.

## What generates the step pulses

Orbitaly does not toggle a GPIO pin once per step from Python. The planner
turns a move into **segments** — `(direction, step count, period)` — and a
backend hands each segment to hardware that emits exactly that many pulses at
that rate. Python runs at tens of hertz feeding segments; nothing in the
timing loop is interpreted.

The consequence worth internalising: **step counts are exact, timing is
approximate.** Jitter in the period costs smoothness and a little acoustic
noise. It cannot cost you position, because position is derived from the
number of pulses hardware reports as executed, never from elapsed time.

| Backend | Timing | Exact counts | Where it runs |
|---|---|---|---|
| `lgpio` | C thread, µs resolution | yes | **Any Pi, including Pi 5** |
| `pio` | RP1 hardware | yes | Pi 5 — *not implemented yet* |
| `simulated` | virtual | yes | anywhere |
| ~~pigpio~~ | DMA | yes | Pi 4 and older only — **cannot work on Pi 5** |
| ~~RPi.GPIO~~ | Python | — | **broken on Pi 5** |

### Why not pigpio or RPi.GPIO

The Pi 5 moved the GPIO block into the RP1 southbridge. Libraries that poke
BCM peripheral registers directly — RPi.GPIO and pigpio both — have no
supported path to those pins, and pigpio's DMA waveform engine in particular
cannot be ported. The kernel's gpiochip character device is the supported
interface, and `lgpio` is what speaks it.

`lgpio.tx_pulse` takes an exact cycle count and runs the train in a C thread.
At the reference gearing the azimuth axis tops out at 1600 steps/s — a 625 µs
period — against tens of microseconds of jitter. That is a few percent of
velocity ripple and no position error at all.

### Why not hardware PWM

The Pi's PWM peripheral makes a beautiful, rock-steady square wave and has no
idea how many cycles it emitted. Without an exact count there is no position,
so it is not usable for a step generator. Mentioned here because it looks
attractive right up until you need to know where the antenna is.

## Wiring

```
Pi GPIO ──step/dir/enable──▶ A4988/DRV8825/TMC2209 ──▶ NEMA17/23 ──▶ gearbox ──▶ antenna
   ▲                                                                     │
   └────────────── endstop switch (normally closed) ◀────────────────────┘
```

Default pin map (BCM numbering, from `config.example.yaml`):

| Signal | Azimuth | Elevation |
|---|---|---|
| step | 17 | 23 |
| dir | 27 | 24 |
| enable | 22 | 25 |
| endstop | 5 | 6 |

E-stop is unassigned by default; set `rotator.estop_pin` if you fit one.

**Avoid pins the kernel has already claimed.** With SPI enabled, GPIO 7–11
belong to `spi0`; with I²C, GPIO 2–3; with the serial console, GPIO 14–15.
RPi.GPIO used to let you stomp on those. The character-device interface does
not: claiming one fails with `GPIO busy`. `orbitaly doctor` warns about pins
that clash with the standard overlays.

### Endstops: wire them normally closed

An NC switch idles closed to ground and reads 0. Tripping it opens the loop
and the internal pull-up takes the line to 1. So does a cut wire, a corroded
connector or an unplugged switch — every one of those failures reads as "at
the limit" and stops motion, rather than as "clear sky" and driving into the
stop. That is the whole argument, and it is why `endstop_normally_closed`
defaults to true.

If you must use NO switches, set `endstop_normally_closed: false` per axis and
understand that a broken wire is then invisible.

### Endstops are limits, not just homing aids

The switch is monitored by an interrupt at all times. A trip during normal
motion aborts the pulse train immediately, latches a fault, and blocks further
motion *toward* that limit while still allowing you to jog away.

## Safety interlocks

| Interlock | Config | Behaviour |
|---|---|---|
| Homing gate | `require_homing` (default: on for hardware) | No motion until the axis knows where it is |
| Live endstops | `endstop_normally_closed`, `endstop_debounce_ms` | Immediate abort, latched fault |
| E-stop | `estop_pin` | Both axes abort, enable lines drop |
| Watchdog | `watchdog_s` | Stops if whatever was steering stops heartbeating |
| Idle disable | `idle_disable_s` | Releases motor current at rest (a worm drive holds; a spur drive does not) |

**Emergency stops cost you your position reference.** Cutting a pulse train
mid-segment means the backend cannot say how many of those pulses the motor
received. Rather than carry on with soft limits measured from a number that
might be wrong, the axis drops its homed flag and demands re-homing. Ordinary
retargeting never does this: a new target is planned from the end of what
hardware has already been given, so nothing in flight is ever discarded.

## Commissioning order

Do not skip to the end. Each step assumes the previous one passed.

**1. Check the machine.**

```bash
orbitaly doctor -c /etc/orbitaly/config.yaml
```

Confirm it picked the backend you expect. If it says `simulated` on a Pi,
something is wrong — usually a missing `lgpio` or a user not in the `gpio`
group — and nothing will move.

**2. Measure the pulse train, motors disconnected.**

Unplug the motors from the drivers. Jumper the azimuth step output to a spare
input, then:

```bash
orbitaly selftest --loopback --axis az --pin 6
```

This emits a known number of pulses and counts the edges that come back. The
count must be exact at every rate. If only the fastest run under-counts,
suspect the edge counter rather than the pulse train — lgpio's alert queue
drops events well below the rate the driver can emit.

Note the period spread it reports. That is your real jitter figure on your
hardware, and it is the number to quote if you ever wonder whether to chase
the PIO backend.

**3. Set the driver current** to the motor's rating before it turns anything.
A stepper driven above its rated current gets hot and loses torque; below, it
stalls under load. Both cost steps.

**4. Home with the gearbox uncoupled.** Reconnect the motors but leave them
off the mechanism. Press Home. Confirm the axis seeks, backs off, and
re-approaches slowly. The second approach is what sets the datum, which is why
it is worth watching once.

**5. Couple it up and jog.** Small jogs first, both directions. Check that
positive azimuth turns the way you expect — if not, set `invert_dir` rather
than rewiring.

**6. Trip the endstops by hand** while the axis is moving slowly toward them.
Motion must stop immediately and a fault must appear in the dashboard.

**7. Track a high pass.** Watch the pointing error near TCA. If the mount
cannot keep up, the tracker will lag and recover — that is a speed limit, not
a fault.

## Tuning

- `microsteps` — resolution comes mostly from the gearbox, so more
  microstepping mainly buys quieter running at a higher step rate. Doubling
  microsteps halves your timing headroom. At 8 microsteps the azimuth axis has
  ~600 µs per step; at 32 it has ~156 µs.
- `start_speed_sps` — the rate the axis may start and stop at instantly. Raise
  it if the ends of moves feel needlessly slow; lower it if the motor stalls
  when starting.
- `segment_ms` — how much motion hardware holds at once. Smaller reacts to a
  retarget sooner; larger tolerates more scheduling latency.
- `realtime_priority` — SCHED_FIFO for the axis threads. Only useful on a busy
  Pi, and needs `CAP_SYS_NICE` (the supplied systemd unit grants it).

## Running the tests without hardware

The whole motion stack is exercised on any machine:

```bash
pytest                    # planner, supervisor, interlocks, whole passes
pytest -m gpiosim         # real kernel gpiochips (needs CONFIG_GPIO_SIM, root)
```

`tests/fakes/fake_lgpio.py` enforces the rules a real Pi enforces — unclaimed
lines, double claims, pins owned by overlays, pulses narrower than the driver
minimum, direction changes under live pulses. `tests/fakes/mechanics.py` is a
rotator that can lose steps, so a clean test run means the planner never asked
for motion the mechanism could not deliver.

What none of that can tell you is what the Pi really put on the wire. Only
step 2 above does that.
