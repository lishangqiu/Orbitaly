# Orbitaly — Design & Implementation Plan

Antenna-tracker control software for an amateur-radio ground station.
Think **gpredict**, but with a modern web UI, richer features, and — critically —
**direct stepper-motor integration** (no gcode, no rotctld middleman required).

---

## 1. Goals

1. **Orbital prediction** for all amateur-radio satellites: real-time position
   (azimuth / elevation / range / doppler) and pass prediction (AOS, TCA, LOS,
   max elevation) from up-to-date TLEs.
2. **Antenna control**: an azimuth/elevation rotator driven *directly* — the
   software owns the step/dir pulse generation, acceleration ramps, homing,
   soft limits, gear ratios, and backlash compensation. No gcode translation
   layer, no external motion controller firmware needed.
3. **Better UI than gpredict**: a real-time web dashboard (polar sky plot,
   pass timeline, doppler readouts, one-click tracking) usable from any
   browser on the LAN — including a phone standing next to the antenna.
4. **More features**: doppler computation per-transponder, smart azimuth
   unwrapping for overhead passes, pass scheduling groundwork, simulated
   hardware mode for development without a rotator attached.

## 2. Non-goals (v1)

- Rig CAT control / automatic doppler tuning of a transceiver (roadmap §10).
- SDR waterfall display (roadmap).
- Multi-rotator / multi-station support (roadmap).
- Windows service packaging — v1 targets Linux (Raspberry Pi is the reference
  deployment; any Linux box works in simulation mode).

## 3. Architecture overview

```
                ┌────────────────────────────────────────────┐
                │                Web browser                  │
                │   dashboard: sky plot, passes, controls     │
                └───────▲───────────────────────▲────────────┘
                        │ REST (commands/query) │ WebSocket (state @1 Hz)
┌───────────────────────┴───────────────────────┴────────────────────┐
│                        FastAPI application                          │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────────────────┐  │
│  │  TLE manager  │  │  Predictor    │  │  Tracker (state machine)│  │
│  │ fetch/cache/  │─▶│ skyfield/SGP4 │─▶│ IDLE→ACQUIRING→TRACKING │  │
│  │ refresh       │  │ az/el/doppler │  │ az-unwrap, park, jog    │  │
│  └──────────────┘  └───────────────┘  └───────────┬─────────────┘  │
│                                                   │ target az/el    │
│                                       ┌───────────▼─────────────┐  │
│                                       │  Rotator HAL (ABC)      │  │
│                                       └──┬─────────────┬────────┘  │
└──────────────────────────────────────────┼─────────────┼───────────┘
                                ┌──────────▼───┐   ┌─────▼─────────────┐
                                │ Simulated    │   │ GPIO stepper       │
                                │ rotator      │   │ 2× StepperAxis     │
                                │ (dev/test)   │   │ step/dir + homing  │
                                └──────────────┘   └───────────────────┘
```

One Python process. The tracker and each stepper axis run in their own
threads; the web layer is async (uvicorn). State flows one way:
predictor → tracker → HAL, and a snapshot of everything is broadcast to
browsers over WebSocket about once per second.

## 4. Technology choices

| Concern            | Choice                | Why                                                    |
|--------------------|-----------------------|--------------------------------------------------------|
| Language           | Python 3.10+          | Skyfield ecosystem, runs fine on a Pi, easy to hack on |
| Orbit propagation  | **Skyfield** (SGP4)   | Maintained, accurate, handles TLE epochs/timescales    |
| Web framework      | **FastAPI + uvicorn** | Async WebSockets, typed models, tiny footprint         |
| Frontend           | Vanilla JS + canvas   | No build step; single static page; works on a Pi       |
| TLE source         | Celestrak (amateur group), pluggable | Canonical source for ham satellite TLEs |
| Config             | YAML                  | Human-editable station/hardware description            |
| GPIO               | lgpio (optional extra)| Kernel gpiochip access; the only supported path on a Pi 5 |
| Rig control        | hamlib `rigctld` over TCP | ~250 radios for free, nothing to implement per model |

**Pulse generation (revised — see §9 M4).** Python never times individual
steps. The planner emits *segments* — `(direction, step count, period)` — and
a backend hands each to hardware that emits exactly that many pulses, so
Python runs at tens of hertz feeding segments instead of thousands of hertz
toggling a pin. `lgpio.tx_pulse` provides exact cycle counts from a C thread
and works on every Pi including the 5; RP1 PIO fits the same interface for
hardware-exact timing later.

The property this buys: **step counts are exact, timing is approximate.**
Position is derived from pulses hardware reports as executed, never from
elapsed time, so jitter costs smoothness and never accuracy.

## 5. Module design

### 5.1 `orbitaly.config`
- Dataclasses mirroring `config.yaml`: station (lat/lon/alt, name),
  TLE sources & refresh interval, tracker settings (update rate, min
  elevation, park position), rotator selection + per-axis hardware config
  (pins, steps/rev, microsteps, gear ratio, max speed/accel, travel limits,
  homing switch, backlash).
- `load_config(path)` with defaults so the app runs with zero config
  (simulated rotator, sensible station placeholder).

### 5.2 `orbitaly.core.tle` — TLE manager
- Fetch TLE sets over HTTPS (Celestrak `group=amateur` by default; any URL
  list works, e.g. SatNOGS exports).
- Parse into `{norad_id: (name, line1, line2)}`; merge multiple sources.
- Disk cache (`~/.cache/orbitaly/tle.json` or configured path) with fetch
  timestamp; serve stale cache when offline; background refresh every N hours.
- Built-in transponder database (`transponders.json`) for common ham sats
  (ISS, SO-50, AO-91, RS-44, …) giving uplink/downlink frequencies for
  doppler readouts; user-extendable from config.

### 5.3 `orbitaly.core.predictor` — orbital math
- Wraps Skyfield: `EarthSatellite` per TLE + a `wgs84` topos for the station.
- `observe(sat, t) -> Observation`: az, el, slant range, range-rate,
  doppler factor, sub-satellite lat/lon, footprint radius, eclipse/sunlit.
- `next_passes(sat, hours, min_elevation) -> [Pass]` using
  `find_events`; each `Pass` has AOS/TCA/LOS times, max elevation,
  AOS/LOS azimuths, and a sampled az/el profile (for sky-plot rendering and
  azimuth unwrap planning).
- Doppler: `f_observed = f_emitted * (1 - range_rate/c)` applied to each
  transponder frequency (downlink shifts opposite to uplink correction).

### 5.4 `orbitaly.core.tracker` — tracking state machine
States: `IDLE → ACQUIRING → TRACKING → (IDLE | PARKED)`, plus `MANUAL`.
- **ACQUIRING**: satellite below horizon → pre-position to the pass's AOS
  azimuth / 0° elevation so the pass starts on target.
- **TRACKING**: at `update_rate` (default 1 s), get az/el from predictor and
  command the rotator; drop back to ACQUIRING/IDLE at LOS.
- **Azimuth unwrapping**: the raw azimuth jumps 359°→0° on north-crossing
  passes. The tracker keeps a *continuous* azimuth (cumulative, no wraps) and
  maps it into the rotator's configured travel (e.g. −90°…+450°) choosing the
  starting offset that lets the entire upcoming pass play out without hitting
  a travel limit — the same trick 450°-capable az rotators exist for.
- **MANUAL**: nudge/goto commands from the UI; tracking suspended.
- `park()` returns to the configured park position.

### 5.5 `orbitaly.hardware` — Rotator HAL
- `base.Rotator` (ABC): `goto(az, el)`, `jog(daz, del_)`, `stop()`,
  `home()`, `park(az, el)`, `state -> RotatorState` (commanded + actual
  position, moving flags, homed flags, fault string).
- `simulated.SimulatedRotator`: the original continuous kinematic model, kept
  as `backend: kinematic` for pure-UI demos. It does not exercise the planner,
  so it is no longer the default.

Motion itself now lives in `orbitaly.motion` (§5.5b); `hardware.gpio` is a
compatibility shim so `backend: gpio` in an old config still starts.

### 5.5b `orbitaly.motion` — planning, backends, supervision
- `segment.Segment` / `AxisKinematics`: the unit of work and the limits.
- `planner`: pure functions, no clock. Trapezoidal profiles in step space that
  always end at rest, replan from a moving start, and emit ramps as a
  staircase whose stair height is bounded by what a motor can swallow.
- `driver.AxisDriver` / `BufferedAxisDriver`: segment-level electrical
  interface, plus the bookkeeping that keeps ~60 ms queued in hardware and
  serialises every direction change.
- `lgpio_driver`, `sim_driver`, `pio_driver`: the backends.
- `axis.StepperAxis`: supervisor — replans on retarget, homes (fast seek, back
  off, slow approach), and owns the interlocks. Retargeting never aborts
  in-flight pulses; it replans from the end of what hardware already has,
  which is why tracking can retarget every second without drifting.
- `rotator.StepperRotator`: two axes as a `Rotator`, plus E-stop and the
  heartbeat watchdog.
- `detect`: probes the machine and picks a backend, reportably.

### 5.5c `orbitaly.radio`, `orbitaly.net`, `orbitaly.core.scheduler`
- `radio`: `Rig` ABC, a rigctld TCP client using the extended-response
  protocol, a simulated rig, and `DopplerTuner` (full-duplex TX/RX correction,
  per-mode deadbands, CAT rate limiting, PTT awareness).
- `net.rotctld_server`: hamlib rotator protocol on TCP 4533.
- `core.scheduler`: watch list → conflict-resolved pass plan → tracker
  handover, yielding immediately to manual control.

### 5.6 `orbitaly.api` + `orbitaly.app` — web layer
REST (JSON):
- `GET  /api/satellites` — catalog w/ live az/el/range, sorted by elevation
- `GET  /api/satellites/{id}` — detail incl. transponders + doppler
- `GET  /api/satellites/{id}/passes?hours=24` — pass predictions w/ profiles
- `GET  /api/status` — tracker + rotator + TLE freshness snapshot
- `POST /api/track/{id}` / `POST /api/track/stop` — engage/disengage
- `POST /api/rotator/goto {az, el}` / `.../jog` / `.../stop` / `.../park` / `.../home`
- `POST /api/rotator/estop` / `.../fault/clear` — emergency stop and reset.
  Motion refused by an interlock answers **409** with the reason.
- `GET/POST /api/doppler` — 2 m band working frequencies (144–148 MHz) and
  live TX/RX corrected values, shift, and drift rate for the tracked satellite
- `GET/POST /api/rig` — rig tuning state and working frequencies (any band)
- `GET /api/schedule`, `POST /api/schedule/{enable,disable}` — unattended passes
- `POST /api/tle/refresh` — force TLE re-fetch

WebSocket `/ws`: pushes the `/api/status` snapshot (plus tracked-satellite
observation & doppler) at 1 Hz to every connected browser.

### 5.7 Frontend (`orbitaly/static/`)
Single page, three columns, dark theme:
- **Sky plot** (canvas polar chart): horizon rings, tracked satellite dot,
  upcoming pass arc, antenna-pointing crosshair — the at-a-glance view.
- **Satellite list**: searchable, sorted by "up now / next pass", elevation
  badges, click → detail + passes + *Track* button.
- **Control panel**: az/el readouts (commanded vs actual), manual nudge
  buttons (±0.5°, ±5°), goto form, Park / Stop / Home, tracker state chip,
  doppler-corrected uplink/downlink frequencies for the tracked bird.
- Reconnecting WebSocket client; no framework, no build step.

## 6. Repository layout

```
Orbitaly/
├── PLAN.md                  ← this document
├── README.md
├── pyproject.toml           # deps; optional extras: [pi] (lgpio), [dev]
├── config.example.yaml
├── deploy/                  # systemd unit + install-pi.sh
├── docs/HARDWARE.md         # wiring, interlocks, commissioning order
├── orbitaly/
│   ├── __main__.py          # serve | doctor | selftest
│   ├── config.py
│   ├── app.py               # FastAPI wiring + lifespan (threads up/down)
│   ├── api/{rest,ws}.py
│   ├── cli/{doctor,selftest}.py
│   ├── core/{tle,predictor,tracker,scheduler}.py
│   ├── data/transponders.json
│   ├── hardware/{base,simulated,gpio}.py   # gpio.py is a compat shim
│   ├── motion/              # segment planning, backends, supervision
│   ├── net/rotctld_server.py
│   ├── radio/{base,rigctld,simulated,tuner}.py
│   └── static/{index.html,app.js,style.css}
└── tests/
    ├── fakes/               # fake lgpio, virtual rotator, fake rigctld, clock
    ├── test_predictor.py    # known-TLE sanity: ISS az/el ranges, pass ordering
    ├── test_tracker.py      # unwrap logic, state transitions
    ├── test_planner.py      # exact counts, ramp shape, stopping distance
    ├── test_axis.py         # supervisor + interlocks against a virtual mechanism
    ├── test_lgpio_driver.py # the real Pi code path, contract-enforcing fake
    ├── test_pi_simulation.py# whole passes end to end
    ├── test_gpiosim.py      # opt-in: real kernel gpiochips
    └── test_{rig,rotctld,scheduler,cli}.py
```

## 7. Key algorithms (detail)

**Azimuth unwrap.** Given a pass's sampled azimuth profile `a₀…aₙ`, build the
continuous profile by adding ±360° whenever consecutive samples jump >180°.
Then choose `k ∈ {−1, 0, +1}` such that `[min(profile), max(profile)] + 360k`
fits inside `[az_min, az_max]` (rotator travel). If none fits (rotator with
exactly 360°), fall back to raw azimuth and accept one mid-pass wrap slew —
and surface a UI warning.

**Trapezoidal profile (segment form).** A move is planned whole, in step
space, and always ends at rest: accelerate from the current rate, cruise at
`v_max`, decelerate to the pull-in rate. Each step is assigned the *lower* of
the ideal ramp velocities at its two ends, so the staircase sits under the
ideal curve and the acceleration limit is approached but never exceeded — and
the first step of an acceleration, and the last of a deceleration, land at the
pull-in rate, which is what makes starting and stopping clean. Consecutive
steps are merged while they stay within `max_jump_sps`, turning a 27,000-step
slew into a few hundred segments.

Because every plan terminates at rest, stopping is always safe: let the queue
drain. And because a retarget replans from the *committed* end of the queue
rather than aborting, no in-flight pulses are ever discarded — the reason 1 Hz
tracking updates accumulate no error.

**Backlash.** Track last motion direction per axis; on reversal emit the
configured backlash steps as a non-counting segment — pulses that turn the
motor and wind up the slack without changing logical position.

**Doppler.** Skyfield gives range-rate directly from the topocentric
position/velocity; `Δf = −f·ṙ/c`. Downlink: observed = f·(1 − ṙ/c).
Uplink correction: transmit f·(1 + ṙ/c) to be heard on-frequency.

## 8. Configuration example

```yaml
station: {name: "Home QTH", latitude: 40.44, longitude: -79.94, altitude_m: 300}
tle:
  sources: ["https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle"]
  refresh_hours: 6
tracker: {update_rate_s: 1.0, min_elevation_deg: 5.0, park: {az: 0, el: 90}}
rotator:
  backend: simulated          # simulated | gpio
  azimuth:  {min_deg: -90, max_deg: 450, max_speed_dps: 6, accel_dps2: 4,
             steps_per_rev: 200, microsteps: 8, gear_ratio: 60,
             pins: {step: 17, dir: 27, enable: 22, endstop: 5}}
  elevation: {min_deg: 0, max_deg: 180, max_speed_dps: 4, accel_dps2: 4,
              steps_per_rev: 200, microsteps: 8, gear_ratio: 40,
              pins: {step: 23, dir: 24, enable: 25, endstop: 6}}
```

## 9. Milestones

1. **M1 — Core math** ✅: TLE manager, predictor, passes, doppler + tests
2. **M2 — Motion** ✅: StepperAxis, simulated + GPIO rotators, tracker w/ unwrap + tests
3. **M3 — Web** ✅: REST/WS API, dashboard UI
4. **M4 — Field hardening** ✅: segment-based motion (`orbitaly.motion`), lgpio
   backend for the Pi 5, safety interlocks, Pi-behaviour simulation harness,
   `orbitaly doctor` / `selftest --loopback`, systemd unit, install docs
5. **M5 — Radio** ✅: rigctld CAT doppler tuning, rotctld-compatible server,
   pass scheduler

### M4 note: the pulse backend changed

M4 originally planned a pigpio waveform backend. **pigpio cannot run on a
Raspberry Pi 5** — the RP1 southbridge moved the GPIO block out from under
every register-poking library, RPi.GPIO included — so the target hardware
ruled out both the old backend and its planned replacement.

What shipped instead: the planner emits `(direction, steps, period)` segments
and a backend hands each to hardware that emits exactly that many pulses.
`lgpio.tx_pulse` does this in a C thread with exact cycle counts, which keeps
Python out of the timing loop and makes step counts — and therefore position —
exact regardless of jitter. RP1 PIO fits the same interface and remains the
quality tier; it needs a hardware spike before anything depends on it.

That segment boundary turned out to matter more than expected. The reference
station does not wire the drivers to the Pi at all — it goes
`Pi ──USB──▶ Arduino ──step/dir──▶ drivers` — and a segment is already a
compact, self-contained wire message, so the `serial` backend slotted in at the
same seam with the planner untouched. It is also strictly better on one point:
a microcontroller counts its own ISR ticks, so an abort reports an **exact**
executed count where lgpio can only say "somewhere in this segment". Design and
protocol: `firmware/README.md`; the firmware itself is an exact pulse executor
with no trajectory intelligence in it, which is the whole distinction from
K3NG-style rotator controllers.

## 10. Roadmap beyond v1

- RP1 PIO backend: hardware-exact pulse timing for silent high-microstep drives
- SatNOGS DB sync for transponders; observation upload
- Sky plot polish: sun/moon, multi-satellite view, ground-track world map
- Per-transponder rig tuning driven from the transponder list
- Calibration wizard: measure backlash and gear ratio from the UI
```
