# 🛰 Orbitaly

Antenna-tracker control software for amateur-radio ground stations.

Like **gpredict**, but with a modern real-time web UI, doppler-corrected
frequency readouts, smart azimuth unwrapping for overhead passes — and
**direct stepper-motor control**: Orbitaly generates the step/dir pulses,
acceleration ramps, homing, soft limits, and backlash compensation itself.
No gcode, no motion-controller firmware, no rotctld required.

See [PLAN.md](PLAN.md) for the full architecture and design document.

## Features

- **Orbital prediction** for all amateur-radio satellites (TLEs auto-fetched
  from Celestrak, cached offline, refreshed in the background)
- **Pass prediction**: AOS / TCA / LOS, max elevation, duration, az profile
- **Live tracking**: pick a satellite, Orbitaly pre-positions the antenna at
  the AOS azimuth and follows the pass at 1 Hz
- **Smart azimuth unwrapping**: north-crossing passes are mapped into extended
  rotator travel (e.g. −90°…450°) so the antenna never does a 360° slew
  mid-pass
- **Doppler**: corrected uplink/downlink frequencies per transponder, live
- **2 m band doppler panel**: set any working uplink/downlink pair in
  144–148 MHz and get live TX/RX corrected frequencies, shift, and drift
  rate (Hz/s) for the tracked satellite — the numbers you dial into the rig
- **Direct stepper integration**: trapezoidal ramps, endstop homing, gear
  ratios, microstepping, backlash compensation — driven straight from
  Raspberry Pi GPIO (A4988 / DRV8825 / TMC step-dir drivers)
- **Web dashboard**: polar sky plot, satellite catalog, pass timeline, manual
  jog/goto controls — works from any browser on the LAN, including a phone
- **Simulation mode**: full kinematic rotator model; develop and demo with no
  hardware attached

## Quick start

```bash
pip install -e .            # or: pip install -e .[gpio] on a Raspberry Pi
python -m orbitaly          # starts on http://localhost:8000 (simulated rotator)
```

Open http://localhost:8000, hit **Refresh TLEs** once (or wait — it fetches
on startup), click a satellite, and press **Track**.

## Configuration

```bash
cp config.example.yaml config.yaml   # edit station location + hardware
python -m orbitaly -c config.yaml
```

The example file documents every option: station coordinates, TLE sources,
rotator backend (`simulated` or `gpio`), and per-axis stepper hardware
(pins, steps/rev, microstepping, gear ratio, travel limits, homing, backlash).

## Hardware

The reference deployment is a Raspberry Pi wired to two step/dir stepper
drivers (azimuth + elevation) through worm or spur gear reductions, with
optional endstop switches for homing:

```
Pi GPIO ──step/dir/enable──▶ A4988/DRV8825/TMC ──▶ NEMA17/23 ──▶ gearbox ──▶ antenna
   ▲                                                                │
   └────────────── endstop switch (homing) ◀────────────────────────┘
```

Antenna rotators slew a few degrees per second through high gear reduction,
so software-timed pulses are plenty; a pigpio waveform backend is on the
roadmap for silent high-microstep drives.

## API

Everything the UI does goes through a plain JSON API you can script against:

| Endpoint | Purpose |
|---|---|
| `GET /api/satellites` | catalog with live az/el, sorted by elevation |
| `GET /api/satellites/{id}/passes?hours=24` | pass predictions with az/el profiles |
| `GET /api/status` | tracker + rotator + TLE snapshot |
| `POST /api/track/{id}` / `POST /api/track/stop` | engage / disengage tracking |
| `POST /api/rotator/goto` `{"az":180,"el":45}` | manual pointing |
| `POST /api/rotator/jog` `{"d_az":0.5,"d_el":0}` | nudge |
| `POST /api/rotator/stop` / `park` / `home` | motion control |
| `WS /ws` | full state snapshot pushed at 1 Hz |

## Development

```bash
pip install -e .[dev]
pytest
```

The test suite covers the orbital math (against a fixed historical ISS TLE),
the azimuth-unwrap planner, and the stepper motion logic (ramps, limits,
homing, backlash) against a fake pulse driver — no hardware or network needed.

## Roadmap

Rig CAT control with automatic doppler tuning, a rotctld-compatible server
(so gpredict-era tools can drive Orbitaly hardware), pass scheduling across
satellites, SatNOGS transponder sync, and a pigpio pulse backend.
See [PLAN.md](PLAN.md) §9–10.
