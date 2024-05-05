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
  Raspberry Pi GPIO (A4988 / DRV8825 / TMC step-dir drivers). Pulses are
  generated as hardware-timed *segments*, not from a Python loop, so step
  counts are exact on a Pi 5 and Python never sits in the timing path
- **Rig control**: automatic doppler tuning of a real radio through hamlib's
  `rigctld` — full-duplex TX/RX correction with per-mode deadbands
- **rotctld server**: Orbitaly speaks hamlib's rotator protocol, so gpredict
  and other existing tools can drive its hardware
- **Pass scheduler**: watch a list of satellites, work the best pass
  unattended, hand back the moment anyone touches the controls
- **Safety interlocks**: homing gate, endstops monitored as live limits,
  E-stop input, heartbeat watchdog — and an honest position reference that
  demands re-homing after an emergency stop rather than guessing
- **Web dashboard**: polar sky plot, satellite catalog, pass timeline, manual
  jog/goto controls — works from any browser on the LAN, including a phone
- **Simulation mode**: the same planner, supervisor and interlocks that run on
  the Pi, driving a virtual rotator; develop and demo with no hardware

## Quick start

```bash
pip install -e .            # or: pip install -e '.[pi]' on a Raspberry Pi
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

The reference deployment is a Raspberry Pi 5 wired to two step/dir stepper
drivers (azimuth + elevation) through worm or spur gear reductions, with
normally-closed endstop switches for homing and limits:

```
Pi GPIO ──step/dir/enable──▶ A4988/DRV8825/TMC ──▶ NEMA17/23 ──▶ gearbox ──▶ antenna
   ▲                                                                │
   └────────── endstop switch (homing + live limit) ◀───────────────┘
```

Orbitaly generates motion as **segments** — `(direction, step count, period)`
— and hands each one to hardware that emits exactly that many pulses. Python
runs at tens of hertz feeding segments rather than thousands of hertz toggling
a pin, so step counts are exact and timing jitter costs smoothness, never
position.

On a Pi 5 this goes through `lgpio` and the kernel's gpiochip interface.
**RPi.GPIO and pigpio cannot work on a Pi 5** — the RP1 southbridge moved the
pins out from under both of them — so neither is used any more. An RP1 PIO
backend for hardware-exact timing is designed but not yet implemented.

Before connecting anything that can break itself, run `orbitaly doctor` and
work through the commissioning order in [docs/HARDWARE.md](docs/HARDWARE.md).

## Deployment

```bash
sudo ./deploy/install-pi.sh          # service user, venv, systemd unit
orbitaly doctor -c /etc/orbitaly/config.yaml
orbitaly selftest --loopback --axis az --pin 6   # measure the real pulse train
systemctl start orbitaly
```

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
| `POST /api/rotator/estop` / `fault/clear` | emergency stop, fault reset |
| `GET/POST /api/rig` | doppler tuning state and working frequencies |
| `GET /api/schedule` · `POST /api/schedule/{enable,disable}` | unattended pass scheduling |
| `WS /ws` | full state snapshot pushed at 1 Hz |

Plus a hamlib rotator server on TCP 4533 when `server.rotctld.enabled` is set,
so `rotctl`, gpredict and anything else that speaks the protocol can point the
antenna.

## Development

```bash
pip install -e '.[dev]'
pytest
```

The suite runs the whole stack on any machine, with no hardware and no
network: orbital math against a fixed historical ISS TLE, the motion planner,
the real lgpio driver against a fake that enforces a real Pi's rules
(unclaimed lines, pins owned by SPI, pulses below the driver minimum,
direction changes under live pulses), and complete satellite passes driven
through predictor → tracker → planner → driver → a virtual rotator that can
lose steps.

```bash
pytest -m gpiosim   # also drive real kernel gpiochips (needs CONFIG_GPIO_SIM, root)
```

None of that can prove what the Pi actually put on the wire. For that there is
`orbitaly selftest --loopback`, which jumpers the step output to an input,
counts the edges, and reports measured step counts and period jitter.

## Roadmap

An RP1 PIO backend for hardware-exact pulse timing on the Pi 5, SatNOGS
transponder sync, multi-satellite sky views and observation upload.
See [PLAN.md](PLAN.md) §9–10.
