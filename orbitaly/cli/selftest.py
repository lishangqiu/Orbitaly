"""``orbitaly selftest --loopback`` — measure the pulse train on real hardware.

Every timing claim in this project is a claim about what the Pi actually put
on the wire, and no amount of simulation can settle it. This does the only
honest test: jumper the step output to a spare input, emit a known number of
pulses, and count the edges that come back.

    Motors disconnected. Jumper GPIO17 (azimuth step) to GPIO6, then:
        orbitaly selftest --loopback --axis az --pin 6

Reports commanded versus counted steps, and the spread of measured periods.
The step *count* is what must be exact — position depends on it. Period spread
is quality: it costs smoothness and noise, never position.
"""
from __future__ import annotations

import argparse
import statistics
import time

from ..config import load_config


def run(args: argparse.Namespace) -> int:
    try:
        import lgpio
    except ImportError:
        print("lgpio is not installed. On a Pi:  pip install 'orbitaly[pi]'")
        return 1

    config = load_config(args.config)
    axis = config.rotator.azimuth if args.axis == "az" else config.rotator.elevation
    step_pin = axis.pins.step
    if args.pin == step_pin:
        print("The loopback pin must be a different GPIO from the step output.")
        return 1

    from ..motion.lgpio_driver import resolve_gpiochip

    chip_number = resolve_gpiochip(config.rotator.gpiochip if config.rotator.gpiochip >= 0 else None)
    handle = lgpio.gpiochip_open(chip_number)
    print(f"gpiochip{chip_number}: step on GPIO{step_pin}, counting on GPIO{args.pin}")
    print("Motors should be disconnected from their drivers for this test.\n")

    edges: list[int] = []

    def on_edge(chip, gpio, level, timestamp):  # noqa: ARG001 - lgpio signature
        if level == 1:
            edges.append(timestamp)

    failures = 0
    try:
        lgpio.gpio_claim_output(handle, step_pin, 0)
        lgpio.gpio_claim_alert(handle, args.pin, lgpio.RISING_EDGE)
        callback = lgpio.callback(handle, args.pin, lgpio.RISING_EDGE, on_edge)
        try:
            for rate_sps in _rates(axis, args):
                edges.clear()
                failures += _one_run(lgpio, handle, step_pin, axis, rate_sps, args.steps, edges)
        finally:
            callback.cancel()
    finally:
        lgpio.gpio_free(handle, step_pin)
        lgpio.gpio_free(handle, args.pin)
        lgpio.gpiochip_close(handle)

    print()
    if failures:
        print(f"{failures} run(s) did not deliver the exact step count — see above.")
        print("If only the fast runs miss, suspect the edge counter rather than the pulse train:")
        print("lgpio's alert queue drops events well below the rate the driver can emit.")
        return 1
    print("Step counts exact at every rate tested.")
    return 0


def _rates(axis, args: argparse.Namespace) -> list[float]:
    if args.rate:
        return [args.rate]
    top = axis.max_speed_dps * axis.steps_per_deg
    return [200.0, top / 2.0, top]


def _one_run(lgpio, handle, step_pin, axis, rate_sps, steps, edges) -> int:
    period_us = max(2, int(round(1e6 / rate_sps)))
    on_us = max(1, min(int(axis.pulse_width_us), period_us - 1))
    off_us = period_us - on_us

    lgpio.tx_pulse(handle, step_pin, on_us, off_us, 0, steps)
    deadline = time.monotonic() + steps * period_us / 1e6 + 2.0
    while lgpio.tx_busy(handle, step_pin, lgpio.TX_PWM) and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)  # let the last alerts land

    counted = len(edges)
    exact = counted == steps
    line = (
        f"{rate_sps:8.0f} steps/s ({period_us:6d} us): "
        f"commanded {steps}, counted {counted}"
    )
    if len(edges) > 2:
        intervals = [(b - a) / 1000.0 for a, b in zip(edges, edges[1:])]  # ns -> us
        median = statistics.median(intervals)
        spread = statistics.pstdev(intervals)
        worst = max(abs(i - period_us) for i in intervals)
        line += (
            f" · period {median:7.1f} us (sd {spread:5.1f}, worst error {worst:6.1f} us"
            f" = {100 * worst / period_us:.1f}%)"
        )
    print(("  ok " if exact else "  !! ") + line)
    return 0 if exact else 1


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--loopback",
        action="store_true",
        required=True,
        help="confirm the step output is jumpered to --pin and the motors are disconnected",
    )
    parser.add_argument("--axis", choices=("az", "el"), default="az")
    parser.add_argument("--pin", type=int, required=True, help="BCM pin wired back to step")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--rate", type=float, default=None, help="steps/s (default: sweep three)")
