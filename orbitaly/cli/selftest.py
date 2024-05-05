"""``orbitaly selftest`` — measure what the hardware actually did.

Every timing claim in this project is a claim about what really went onto a
wire, and no amount of simulation can settle it. Two modes, for the two places
pulses can come from:

``--loopback`` (direct-wired Pi builds)
    Jumper the step output to a spare input, emit a known number of pulses, and
    count the edges that come back::

        Motors disconnected. Jumper GPIO17 (azimuth step) to GPIO6, then:
            orbitaly selftest --loopback --axis az --pin 6

``--serial`` (the reference build: Pi ─USB─▶ Arduino)
    Exercise the link itself — round-trip latency, flow control under a
    deliberate overrun, resynchronisation after deliberate corruption, and
    whether the firmware's reported step counts match what was commanded::

            orbitaly selftest --serial

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
    if getattr(args, "serial", False):
        return run_serial(args)
    return run_loopback(args)


def run_loopback(args: argparse.Namespace) -> int:
    if args.pin is None:
        print("--loopback needs --pin: the GPIO the step output is jumpered to.")
        return 1
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


# --------------------------------------------------------------------------
# Serial link
# --------------------------------------------------------------------------

def run_serial(args: argparse.Namespace) -> int:
    """Exercise the USB link to the Arduino, hard.

    What this can and cannot settle is worth being precise about. It proves the
    *link* — framing, CRC recovery, flow control, latency — and it proves the
    firmware's step **accounting** matches what was commanded. It does not prove
    the firmware's electrical output: only a scope, or the loopback jumper on a
    board that can count its own edges, can do that. The distinction is the same
    one that runs through this whole project, so it is printed rather than
    glossed.
    """
    from ..motion import serial_protocol as sp
    from ..motion.serial_driver import SerialBackend, TransportError

    config = load_config(args.config)
    try:
        backend = SerialBackend(config.rotator)
    except TransportError as exc:
        print(f"Cannot reach the firmware: {exc}")
        return 1

    ident = backend.ident
    assert ident is not None
    port = getattr(backend.transport, "port", "?")
    print(f"{port}: firmware {ident.version_str}, protocol {ident.protocol}")
    print(
        f"  queue depth {ident.queue_depth}, {ident.axis_count} axes, "
        f"dir setup {ident.dir_setup_us} us, pulse width {ident.pulse_width_us} us"
    )
    print(
        "  pins az step/dir/en/stop = %s, el = %s"
        % (_pins(ident.pins[:4]), _pins(ident.pins[4:]))
    )
    print()

    failures = 0
    try:
        failures += _serial_latency(backend, args.pings)
        failures += _serial_flow_control(backend, sp)
        failures += _serial_resync(backend, sp)
        failures += _serial_step_counts(backend, sp, config, args)
    finally:
        backend.close()

    print()
    stats = backend.stats
    print(
        f"link totals: {stats['tx_messages']} sent, {stats['rx_messages']} received, "
        f"{stats['crc_errors']} CRC errors, {stats['resyncs']} resyncs"
    )
    if failures:
        print(f"\n{failures} check(s) failed — see above.")
        return 1
    print(
        "\nLink healthy and step accounting exact. This does NOT prove the pulse\n"
        "train itself: put a scope on the STEP pin, or count edges, before\n"
        "believing the timing."
    )
    return 0


def _pins(values) -> str:
    return "/".join("-" if v == 0xFF else str(v) for v in values)


def _serial_latency(backend, count: int) -> int:
    """PING round trips. The number that decides whether `pending_s` should ever
    incorporate link latency (PLAN-ARDUINO §14) — measure before deciding."""
    samples = []
    lost = 0
    for _ in range(count):
        start = time.monotonic()
        if backend.ping():
            samples.append((time.monotonic() - start) * 1000.0)
        else:
            lost += 1
    if not samples:
        print("  !! no PONG came back at all")
        return 1
    median = statistics.median(samples)
    worst = max(samples)
    ok = lost == 0 and worst < 50.0
    print(
        f"  {'ok ' if ok else '!! '} round trip: median {median:.2f} ms, worst {worst:.2f} ms, "
        f"{lost} lost of {count}"
    )
    return 0 if ok else 1


def _serial_flow_control(backend, sp) -> int:
    """Deliberately overrun the firmware queue and check it says no.

    A firmware that silently dropped the overflow would look identical here to
    one that queued it — right up until an antenna ended up somewhere the host
    did not think it was.
    """
    nacks = []
    previous = backend.link.handler

    def capture(message):
        if isinstance(message, sp.Nack):
            nacks.append(message)
        if previous is not None:
            previous(message)

    backend.link.handler = capture
    try:
        depth = backend.queue_depth
        for seq in range(depth + 4):
            # One step each, slowly, so the queue fills rather than draining.
            backend.send_safe(sp.Seg(sp.AXIS_AZ, seq % (sp.SEQ_MAX + 1), 1, 1, 200_000))
        backend.wait_for(lambda: len(nacks) >= 4, 1.0)
    finally:
        backend.link.handler = previous
    backend.send_safe(sp.Abort(sp.AXIS_ALL))
    backend.wait_for(lambda: False, 0.1)

    full = [n for n in nacks if n.reason == sp.NackReason.QUEUE_FULL]
    ok = len(full) >= 1
    print(
        f"  {'ok ' if ok else '!! '} flow control: {len(full)} QUEUE_FULL refusals "
        f"when {backend.queue_depth + 4} segments were pushed at a {backend.queue_depth}-slot queue"
    )
    return 0 if ok else 1


def _serial_resync(backend, sp) -> int:
    """Feed the firmware garbage and a corrupted frame, then a good one.

    This is the recovery path that decides whether a marginal cable degrades
    gracefully or takes the station down.
    """
    before = backend.link.stats["rx_messages"]
    corrupt = bytearray(sp.Ping(0x5A).encode())
    corrupt[-1] ^= 0xFF
    try:
        backend.transport.write(b"\x00\xff\xa5\x13garbage")
        backend.transport.write(bytes(corrupt))
    except Exception as exc:  # noqa: BLE001
        print(f"  !! resync: could not write to the port ({exc})")
        return 1
    ok = backend.ping()
    after = backend.link.stats["rx_messages"]
    print(
        f"  {'ok ' if ok else '!! '} resync: link answered after garbage + a bad CRC "
        f"({after - before} frames read back)"
    )
    return 0 if ok else 1


def _serial_step_counts(backend, sp, config, args) -> int:
    """Command an exact number of steps and check the firmware echoes it.

    Motors should be disconnected: this moves whatever is attached.
    """
    axis_index = sp.AXIS_AZ if args.axis == "az" else sp.AXIS_EL
    axis = config.rotator.azimuth if args.axis == "az" else config.rotator.elevation
    top = axis.max_speed_dps * axis.steps_per_deg
    failures = 0

    for rate_sps in ([args.rate] if args.rate else [200.0, top / 2.0, top]):
        period_us = max(2, int(round(1e6 / rate_sps)))
        reports: list = []
        previous = backend.link.handler

        def capture(message, _reports=reports):
            if isinstance(message, (sp.Done, sp.Aborted)):
                _reports.append(message)
            if previous is not None:
                previous(message)

        backend.link.handler = capture
        try:
            backend.send_safe(sp.Enable(axis_index, 1))
            backend.send_safe(sp.Seg(axis_index, 0, 1, args.steps, period_us))
            expected_s = args.steps * period_us / 1e6
            backend.wait_for(lambda: bool(reports), expected_s + 2.0)
        finally:
            backend.link.handler = previous

        if not reports:
            print(f"  !! {rate_sps:8.0f} steps/s: no completion report came back")
            failures += 1
            continue
        report = reports[0]
        counted = report.steps if isinstance(report, sp.Done) else report.steps_done
        exact = isinstance(report, sp.Done) and counted == args.steps
        note = "" if isinstance(report, sp.Done) else f" (ABORTED: {report.cause_name})"
        print(
            f"  {'ok ' if exact else '!! '}{rate_sps:8.0f} steps/s ({period_us:6d} us): "
            f"commanded {args.steps}, firmware reported {counted}{note}"
        )
        failures += 0 if exact else 1

    backend.send_safe(sp.Enable(axis_index, 0))
    return failures


def add_arguments(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--loopback",
        action="store_true",
        help="confirm the step output is jumpered to --pin and the motors are disconnected",
    )
    mode.add_argument(
        "--serial",
        action="store_true",
        help="exercise the USB link to the Arduino (latency, flow control, resync, step counts)",
    )
    parser.add_argument("--axis", choices=("az", "el"), default="az")
    parser.add_argument(
        "--pin", type=int, default=None, help="BCM pin wired back to step (--loopback only)"
    )
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--rate", type=float, default=None, help="steps/s (default: sweep three)")
    parser.add_argument(
        "--pings", type=int, default=50, help="round trips to time (--serial only)"
    )
