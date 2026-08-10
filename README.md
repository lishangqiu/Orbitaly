# Orbitaly

Antenna tracker control for amateur-radio ground stations: orbital prediction,
pass scheduling, and direct stepper-motor az/el rotator control.

```bash
pip install -e .            # or: pip install -e '.[pi]' on a Raspberry Pi
python -m orbitaly          # http://localhost:8000, simulated rotator
```

Open the page, hit **Refresh TLEs**, pick a satellite, press **Track**.

## What it does

- **Prediction and tracking** — SGP4 passes with az/el profiles, a sky plot and
  a world map, live doppler for the frequencies you work
- **Rotator control** — motion planned as step segments, so step counts stay
  exact and timing jitter costs smoothness rather than position
- **Safety interlocks** — homing gate, endstops as live limits, E-stop input
  and a heartbeat watchdog
- **Pass scheduler** — work the best pass unattended, hand back the moment
  anyone touches the controls
- **Interop** — Orbitaly speaks hamlib's rotator protocol, so gpredict and
  friends can drive its hardware; a JSON API and a 1 Hz WebSocket cover the rest
- **Simulation mode** — the same planner and interlocks against a virtual
  rotator, so it all runs with no hardware attached

## Configuration

```bash
cp config.example.yaml config.yaml   # station location, hardware, UI theme
python -m orbitaly -c config.yaml
```

The example file documents every option. The reference build is a Raspberry Pi
talking over USB to an Arduino that drives two step/dir stepper drivers; a
direct-wired Pi with no Arduino works too. Run `orbitaly doctor` before
connecting anything that can break itself.

## Development

```bash
pip install -e '.[dev]'
pytest
```

The suite runs the whole stack on any machine — no hardware, no network.

MIT.
