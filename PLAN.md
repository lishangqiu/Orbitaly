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
| GPIO               | RPi.GPIO / pigpio (optional extra) | Direct step-pulse generation on a Pi      |

Python software-timed stepping is jittery above a few hundred Hz; the plan is:
`RPi.GPIO` software pulses are fine for typical az/el tracking rates (a
rotator slews a few °/s through high gear reduction → modest step rates), and
a `pigpio` waveform backend is the documented upgrade path for high
microstepping rates. The `StepperAxis` pulse generator is pluggable so both
fit behind the same interface — and so tests can run with a fake clock.

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
- `simulated.SimulatedRotator`: kinematic model honoring max speed/accel —
  the default backend; the whole app is developable with no hardware.
- `stepper.StepperAxis`: one motor axis, all in software:
  - degrees ↔ steps via `steps_per_rev × microsteps × gear_ratio`
  - trapezoidal velocity profile (accel-limited ramps)
  - soft travel limits, homing to a limit/hall switch, backlash takeup
  - runs in its own thread; pulses go through a pluggable `PulseDriver`
    (real GPIO or a test double).
- `gpio.GpioRotator`: two `StepperAxis` (az, el) on step/dir/enable pins,
  optional endstop inputs. Imports `RPi.GPIO` lazily so the codebase runs
  anywhere.

### 5.6 `orbitaly.api` + `orbitaly.app` — web layer
REST (JSON):
- `GET  /api/satellites` — catalog w/ live az/el/range, sorted by elevation
- `GET  /api/satellites/{id}` — detail incl. transponders + doppler
- `GET  /api/satellites/{id}/passes?hours=24` — pass predictions w/ profiles
- `GET  /api/status` — tracker + rotator + TLE freshness snapshot
- `POST /api/track/{id}` / `POST /api/track/stop` — engage/disengage
- `POST /api/rotator/goto {az, el}` / `.../jog` / `.../stop` / `.../park` / `.../home`
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
├── pyproject.toml           # deps; optional extra: [gpio]
├── config.example.yaml
├── orbitaly/
│   ├── __main__.py          # python -m orbitaly [--config path]
│   ├── config.py
│   ├── app.py               # FastAPI wiring + lifespan (threads up/down)
│   ├── api/{rest,ws}.py
│   ├── core/{tle,predictor,tracker}.py
│   ├── data/transponders.json
│   ├── hardware/{base,simulated,stepper,gpio}.py
│   └── static/{index.html,app.js,style.css}
└── tests/
    ├── test_predictor.py    # known-TLE sanity: ISS az/el ranges, pass ordering
    ├── test_tracker.py      # unwrap logic, state transitions
    └── test_stepper.py      # deg↔steps, ramp profile, limits, homing (fake driver)
```

## 7. Key algorithms (detail)

**Azimuth unwrap.** Given a pass's sampled azimuth profile `a₀…aₙ`, build the
continuous profile by adding ±360° whenever consecutive samples jump >180°.
Then choose `k ∈ {−1, 0, +1}` such that `[min(profile), max(profile)] + 360k`
fits inside `[az_min, az_max]` (rotator travel). If none fits (rotator with
exactly 360°), fall back to raw azimuth and accept one mid-pass wrap slew —
and surface a UI warning.

**Trapezoidal profile.** Per axis-thread iteration: current velocity `v`,
target position `x_t`. Decel distance `d = v²/2a`. If `|x_t − x| ≤ d`,
decelerate, else accelerate toward `v_max` (sign toward target). Step period
`1/(v·steps_per_deg)`. Retargeting mid-move is safe because the loop only
ever reasons about *current* state → new target.

**Backlash.** Track last motion direction per axis; on reversal add the
configured backlash steps before counting real travel.

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

1. **M1 — Core math** ✅ (this commit): TLE manager, predictor, passes, doppler + tests
2. **M2 — Motion** ✅: StepperAxis, simulated + GPIO rotators, tracker w/ unwrap + tests
3. **M3 — Web** ✅: REST/WS API, dashboard UI
4. **M4 — Field hardening** (next): pigpio pulse backend, homing UX, calibration
   wizard, systemd unit, install docs on real hardware
5. **M5 — Radio** (roadmap): hamlib CAT doppler tuning, rotctld-compatible server
   so legacy tools can *also* drive Orbitaly

## 10. Roadmap beyond v1

- Rig control: hamlib bindings, per-transponder doppler tuning of a real radio
- rotctld protocol emulation (Orbitaly as a drop-in gpredict rotor backend)
- Pass scheduler: queue passes across satellites, auto-track the best pass
- SatNOGS DB sync for transponders; observation upload
- Sky plot polish: sun/moon, multi-satellite view, ground-track world map
- pigpio/PIO waveform stepping for silent high-microstep drives
```
