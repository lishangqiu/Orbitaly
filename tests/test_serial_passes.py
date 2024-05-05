"""The accuracy table, re-flown with an Arduino in the loop.

PLAN-ARDUINO §10 makes this the gate on milestone 3: whole-pass simulations
must match the lgpio-era numbers with zero lost steps. The serial link is
allowed to cost frames, latency and CPU. It is not allowed to cost pointing.

Everything above the driver is the same code as `test_pi_simulation.py` — same
predictor, tracker, planner and supervisor, same TLE, same passes. The only
substitution is where pulses come from: a `FakeFirmware` speaking the real byte
protocol instead of `SimulatedBackend`'s clock arithmetic.
"""
from __future__ import annotations

import pytest

from orbitaly.motion.serial_driver import SerialBackend

from fakes.fake_firmware import FakeFirmware, FirmwareTransport
from test_pi_simulation import (
    MODERATE_PASS_AOS,
    NORTH_CROSSING_AOS,
    OVERHEAD_PASS_AOS,
    PassRun,
    rotator_config,
)


def serial_backend(config, mechanics, clock):
    """A real SerialBackend over a fake Arduino, stepped by the pass runner."""
    firmware = FakeFirmware(mechanics=mechanics, clock=clock)
    backend = SerialBackend(config, transport=FirmwareTransport(firmware), clock=clock)
    backend.firmware = firmware  # so tests can reach in
    return backend, lambda: firmware.advance(clock.monotonic())


def serial_run(aos: float, config=None, **kwargs) -> PassRun:
    return PassRun(aos, config=config or rotator_config(), backend_factory=serial_backend, **kwargs)


def drain(run: PassRun, timeout_s: float = 60.0, tick: float = 0.01) -> bool:
    """Run the motion stack (but not the tracker) until the antenna is at rest.

    `PassRun.run` stops mid-plan: it ends by ticking the supervisors, which hand
    over segments the firmware has not executed yet, and the plan itself may
    still have a slew left in it. Neither is something a station would see — it
    is where the loop stopped. Draining to rest is the only state in which
    "nothing should still be outstanding" is a meaningful thing to assert.
    """
    elapsed = 0.0
    while elapsed < timeout_s:
        run.clock.advance(tick)
        run.before_motion_tick()
        run.rotator.tick()
        elapsed += tick
        if not run.rotator.state().moving:
            return True
    return False


def test_a_normal_pass_over_serial_matches_the_direct_drive_numbers():
    run = serial_run(MODERATE_PASS_AOS)
    run.run(120 + 601 + 30)
    assert run.worst_error() < 0.6
    assert run.mechanics["az"].steps_lost == 0
    assert run.mechanics["el"].steps_lost == 0
    assert run.mechanics["az"].stalls == 0


def test_a_steep_pass_over_serial_is_still_tracked_accurately():
    """75 degrees is where azimuth rate blows up — the case most likely to
    expose a link that cannot keep segments flowing."""
    run = serial_run(OVERHEAD_PASS_AOS)
    run.run(120 + 657 + 30)
    assert run.worst_error() < 2.0
    assert run.rotator.state().fault == ""
    assert run.mechanics["az"].steps_lost == 0


def test_a_north_crossing_pass_over_serial_never_unwinds_the_cable():
    run = serial_run(NORTH_CROSSING_AOS)
    run.run(120 + 613 + 30)
    assert run.worst_error() < 1.0
    assert run.azimuth_travelled() < 360.0
    assert run.mechanics["az"].steps_lost == 0


def test_the_serial_pass_agrees_with_the_direct_drive_pass_sample_for_sample():
    """The strongest form of the claim: not merely 'also accurate', but the same
    trajectory. Both backends are fed identical plans by identical supervisors,
    so any divergence is the link — and there should not be one, because segment
    counts are exact on both."""
    direct = PassRun(MODERATE_PASS_AOS)
    direct.run(120 + 300)
    serial = serial_run(MODERATE_PASS_AOS)
    serial.run(120 + 300)

    assert len(direct.samples) == len(serial.samples)
    worst = max(
        abs(a["az"] - b["az"]) + abs(a["el"] - b["el"])
        for a, b in zip(direct.samples, serial.samples)
    )
    # Not bit-identical, and it should not be. A segment handed to the simulated
    # driver starts being executed at once; one handed to the firmware starts on
    # its next step-generator pass, up to one motion tick later. That is real
    # transport latency and the bound is what it can produce at full speed:
    config = rotator_config()
    tick_s = 0.01  # PassRun's motion tick
    budget = (config.azimuth.max_speed_dps + config.elevation.max_speed_dps) * tick_s
    assert worst < budget, f"backends diverged by {worst:.4f} deg, over the {budget:.2f} deg budget"
    # The sharper claim — that the divergence is lag and not lost position — is
    # made by test_reported_position_matches_the_mechanism_at_the_end_of_a_pass.


def test_the_link_stays_healthy_for_a_whole_pass():
    run = serial_run(MODERATE_PASS_AOS)
    run.run(120 + 601 + 30)
    assert drain(run), "the antenna never came to rest"
    stats = run.backend.stats
    assert stats["crc_errors"] == 0
    assert stats["decode_errors"] == 0
    assert stats["resets"] == 0
    assert stats["fault"] == ""
    assert run.backend.firmware.nacks == [], "flow control should never need a QUEUE_FULL"
    # Every segment handed over was reported back: nothing is still outstanding
    # at the end of a pass that finished at rest.
    for axis in run.rotator.axes:
        assert axis.driver._outstanding == {}


def test_the_wire_carries_the_traffic_the_plan_budgeted_for():
    """PLAN-ARDUINO §4 sized the link at ~100 frames/s worst case against
    ~11 kB/s available at 115200 baud. Worth checking against a real pass rather
    than trusting the arithmetic."""
    run = serial_run(OVERHEAD_PASS_AOS)
    seconds = 120 + 657 + 30
    run.run(seconds)
    stats = run.backend.stats
    # 14 bytes per SEG on the wire, and reports are smaller still.
    tx_bytes_per_s = stats["tx_messages"] * 14 / seconds
    assert tx_bytes_per_s < 11_000 / 4, (
        f"{tx_bytes_per_s:.0f} B/s leaves too little headroom at 115200 baud"
    )


@pytest.mark.parametrize("aos", [MODERATE_PASS_AOS, OVERHEAD_PASS_AOS])
def test_reported_position_matches_the_mechanism_at_the_end_of_a_pass(aos):
    """Position is derived from counts the firmware reported as executed. After
    a whole pass those had better still describe the physical antenna."""
    run = serial_run(aos)
    run.run(120 + 400)
    for name, axis in (("az", run.rotator.az), ("el", run.rotator.el)):
        assert axis.position_deg == pytest.approx(
            run.mechanics[name].position_deg, abs=1e-6
        ), f"{name} drifted from the mechanism"
