"""The serial backend against a firmware model that fights back.

Everything here runs through the real byte protocol — real framing, real CRCs,
real flow control — against `fakes/fake_firmware.py`. The failures that matter
on a ground station are not "does a segment execute"; they are what happens when
the cable comes out mid-pass, when the board browns out, when a bit flips, and
whether the position the software reports afterwards is one you would point a
beam with. Those are the tests.

Two of them exist to guard the guard (PLAN-ARDUINO §9): a fake with an infinite
queue would make flow control vacuous, and a fault injector that does not
actually corrupt anything would make the whole recovery suite pass by doing
nothing.
"""
from __future__ import annotations

import pytest

from orbitaly.config import AxisConfig, RotatorConfig
from orbitaly.motion import serial_protocol as sp
from orbitaly.motion.segment import Segment
from orbitaly.motion.serial_driver import SerialBackend, TransportError

from fakes.fake_firmware import FakeFirmware, FaultyTransport, FirmwareTransport
from fakes.harness import SerialAxisHarness
from fakes.mechanics import VirtualAxis


def axis_config(**overrides) -> AxisConfig:
    config = AxisConfig(
        min_deg=-90.0, max_deg=450.0, max_speed_dps=6.0, accel_dps2=4.0, gear_ratio=60.0
    )
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


def harness(mechanics=None, **kwargs) -> SerialAxisHarness:
    config = kwargs.pop("config", None) or axis_config()
    return SerialAxisHarness(config, mechanics, **kwargs)


def bare_backend(firmware=None, transport=None, **config_kwargs):
    """A backend with no supervisor above it, for protocol-level tests."""
    from orbitaly.motion.clock import VirtualClock

    clock = VirtualClock()
    firmware = firmware or FakeFirmware(clock=clock)
    transport = transport or FirmwareTransport(firmware)
    config = RotatorConfig(backend="serial", **config_kwargs)
    backend = SerialBackend(config, transport=transport, clock=clock)
    return backend, firmware, clock


# --------------------------------------------------------------------------
# Handshake
# --------------------------------------------------------------------------

def test_the_handshake_reports_what_the_board_is():
    backend, firmware, _ = bare_backend()
    assert backend.ident is not None
    assert backend.ident.protocol == sp.PROTOCOL_VERSION
    assert backend.ident.queue_depth == firmware.queue_depth
    assert backend.ident.has(sp.Caps.EXACT_ABORT)
    assert "firmware 1.0.0" in backend.describe()


def test_the_boot_ident_and_the_handshake_ident_are_not_a_reboot():
    """The firmware announces itself at boot *and* answers HELLO. Reading the
    second one as a reset would fault a perfectly healthy station at startup."""
    backend, _, _ = bare_backend()
    assert backend.resets == 0
    assert backend.fault == ""


def test_firmware_speaking_another_protocol_version_is_refused():
    """A stale flash must fail in `doctor`, not halfway through a pass."""
    from orbitaly.motion.clock import VirtualClock

    clock = VirtualClock()
    firmware = FakeFirmware(clock=clock)
    original = firmware._emit_ident

    def wrong_version():
        original()
        # rewrite the protocol byte of the IDENT just emitted
        frame = bytearray(firmware.out)
        payload_start = len(frame) - (sp.Ident.STRUCT.size + 2)
        frame[payload_start] = sp.PROTOCOL_VERSION + 1
        body = bytes(frame[1:-2])
        frame[-2:] = sp.crc16_ccitt(body).to_bytes(2, "little")
        firmware.out = frame

    firmware._emit_ident = wrong_version
    firmware.out.clear()
    wrong_version()
    with pytest.raises(TransportError, match="protocol"):
        SerialBackend(
            RotatorConfig(backend="serial"),
            transport=FirmwareTransport(firmware),
            clock=clock,
        )


def test_a_silent_port_fails_the_handshake_rather_than_hanging():
    class Silent:
        port = "fake://silent"

        def write(self, data):
            pass

        def read(self, max_bytes=4096):
            return b""

        def close(self):
            pass

    config = RotatorConfig(backend="serial")
    config.serial.connect_timeout_s = 0.2
    from orbitaly.motion.clock import VirtualClock

    with pytest.raises(TransportError, match="no IDENT"):
        SerialBackend(config, transport=Silent(), clock=VirtualClock())


def test_commands_before_a_handshake_are_refused_by_the_firmware():
    """Guard the guard, protocol edition: a firmware that executed anything sent
    at it would make the version check decorative."""
    firmware = FakeFirmware()
    firmware.out.clear()
    firmware.on_host_bytes(sp.Seg(sp.AXIS_AZ, 0, 1, 10, 625).encode())
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    assert any(
        isinstance(r, sp.Nack) and r.reason == sp.NackReason.NOT_READY for r in replies
    )
    assert firmware.axes[0].queue == firmware.axes[0].queue.__class__()


# --------------------------------------------------------------------------
# Motion and position
# --------------------------------------------------------------------------

def test_a_move_arrives_where_it_was_asked_to():
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config)
    run.axis.goto(10.0)
    assert run.settle(60)
    assert run.axis.position_deg == pytest.approx(10.0, abs=0.01)
    # Position is derived from pulses the firmware reported, so it must agree
    # with where the mechanism physically ended up.
    assert mechanics.position_deg == pytest.approx(run.axis.position_deg, abs=1e-9)
    assert mechanics.steps_lost == 0


def test_position_comes_from_reported_counts_not_from_elapsed_time():
    """The invariant the whole backend is built on. If the host inferred
    position from the clock, a firmware that executed fewer steps than asked
    would go unnoticed — so here the firmware is made to under-deliver."""
    config = axis_config()
    run = harness(config=config)
    driver = run.driver
    driver.enqueue(Segment(direction=1, steps=100, period_us=1000))
    run.run(0.05)  # let it be sent and started
    outstanding = list(driver._outstanding.items())
    assert outstanding, "the segment should have reached the firmware"
    seq, _ = outstanding[0]
    driver.on_done(sp.Done(sp.AXIS_AZ, seq, 60))  # firmware says only 60 went out
    assert driver.consumed_steps() == 60
    # ...and the 40 that never happened are given back, so the planner replans
    # from where the antenna is rather than where it was told to be.
    assert driver.committed_steps() == 60


def test_a_retarget_mid_move_lands_on_the_new_target():
    """Invariant 2: replan from the committed end of the queue, never by
    aborting. Nothing may be discarded in flight."""
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config)
    run.axis.goto(30.0)
    run.run(2.0)
    run.axis.goto(12.0)
    assert run.settle(120)
    assert run.axis.position_deg == pytest.approx(12.0, abs=0.02)
    assert mechanics.steps_lost == 0


def test_an_oversized_segment_is_split_rather_than_truncated():
    """`steps` is a u16 on the wire; a silent truncation would be a position
    error nobody would ever find."""
    run = harness()
    run.driver.enqueue(Segment(direction=1, steps=200_000, period_us=100))
    assert run.driver.committed_steps() == 200_000
    assert all(s.steps <= 0xFFFF for s in run.driver._queue)
    assert sum(s.steps for s in run.driver._queue) == 200_000


# --------------------------------------------------------------------------
# Flow control
# --------------------------------------------------------------------------

def test_the_host_never_overruns_the_firmware_queue():
    run = harness()
    for _ in range(40):
        run.driver.enqueue(Segment(direction=1, steps=200, period_us=1000))
    run.run(0.05)
    depth = run.backend.queue_depth
    assert len(run.driver._outstanding) <= depth
    assert run.firmware.nacks == [], "flow control should make QUEUE_FULL unnecessary"


def test_the_firmware_refuses_an_overrun_rather_than_dropping_it_silently():
    """Guard the guard: a fake with an unbounded queue would make every
    flow-control test above vacuous."""
    firmware = FakeFirmware()
    firmware.on_host_bytes(sp.Hello().encode())
    firmware.take_output()
    for seq in range(firmware.queue_depth + 3):
        firmware.on_host_bytes(sp.Seg(sp.AXIS_AZ, seq, 1, 100, 1000).encode())
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    nacks = [r for r in replies if isinstance(r, sp.Nack)]
    assert len(nacks) == 3
    assert all(n.reason == sp.NackReason.QUEUE_FULL for n in nacks)
    assert len(firmware.axes[0].queue) == firmware.queue_depth


def test_a_queue_full_nack_puts_the_segment_back_rather_than_losing_it():
    run = harness()
    driver = run.driver
    segment = Segment(direction=1, steps=100, period_us=1000)
    driver.enqueue(segment)
    run.run(0.02)
    seq = next(iter(driver._outstanding))
    before = driver.committed_steps()
    driver.on_nack(sp.Nack(sp.AXIS_AZ, seq, sp.NackReason.QUEUE_FULL))
    # Ordering is position: a refused segment goes back at the *head*.
    assert driver._queue[0] is segment
    assert driver.committed_steps() == before


def test_a_resent_segment_is_acknowledged_but_not_executed_twice():
    """The idempotence that makes CRC recovery safe for position."""
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    firmware = FakeFirmware(mechanics={"az": mechanics})
    firmware.on_host_bytes(sp.Hello().encode())
    firmware.take_output()

    frame = sp.Seg(sp.AXIS_AZ, 5, 1, 50, 1000).encode()
    firmware.on_host_bytes(frame)
    firmware.on_host_bytes(frame)  # the host never got the first ACK
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    acks = [r for r in replies if isinstance(r, sp.Ack) and r.seq == 5]
    assert len(acks) == 2, "a re-send must be acknowledged, or the host retries forever"
    assert firmware.duplicates_rejected == 1
    assert len(firmware.axes[0].queue) == 1, "but it must only be queued once"


def test_a_duplicated_frame_on_the_wire_does_not_move_the_antenna_twice():
    """The same property, end to end through the fault injector."""
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config, wrap_transport=FaultyTransport)
    run.transport.duplicate_next_write = True
    run.axis.goto(5.0)
    assert run.settle(60)
    assert run.axis.position_deg == pytest.approx(5.0, abs=0.02)
    assert mechanics.position_deg == pytest.approx(run.axis.position_deg, abs=1e-9)


# --------------------------------------------------------------------------
# Aborts — the reason this backend exists
# --------------------------------------------------------------------------

def test_an_abort_mid_segment_is_exact():
    """lgpio cannot say how many pulses of a cut train reached the motor, so its
    aborts force a re-home. A microcontroller counts its own ISR ticks."""
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config)
    run.axis.goto(30.0)
    run.run(1.0)
    assert run.driver.busy()
    result = run.driver.abort()
    assert result.exact is True
    assert result.consumed_steps == pytest.approx(
        mechanics.position_deg * config.steps_per_deg, abs=1
    )


def test_an_abort_with_nothing_in_flight_is_still_answered():
    run = harness()
    result = run.driver.abort()
    assert result.exact is True
    assert result.consumed_steps == 0


def test_an_abort_the_link_cannot_confirm_is_reported_inexact():
    """Honest degradation: if we cannot hear the answer we do not get to claim
    the count is right."""
    run = harness(wrap_transport=FaultyTransport)
    run.axis.goto(30.0)
    run.run(0.5)
    run.transport.unplugged = True
    result = run.driver.abort()
    assert result.exact is False


def test_an_endstop_cut_leaves_the_axis_still_homed():
    """Invariant 3a — new on this backend. The firmware reports the exact count
    at the cut, so the position reference survives a limit hit; only an E-stop,
    which drops the enables, forces a re-home."""
    config = axis_config(min_deg=0.0, max_deg=360.0)
    mechanics = VirtualAxis(
        steps_per_deg=config.steps_per_deg, position_deg=3.0, endstop_deg=0.0, hard_min_deg=-1.0
    )
    run = harness(mechanics, config=config)
    run.axis.home()
    assert run.run_until(lambda: run.axis.homed, 200)

    run.axis.goto(20.0)
    assert run.settle(120)
    run.axis.goto(0.0)
    assert run.run_until(lambda: run.axis.fault != "", 120)
    assert "endstop" in run.axis.fault
    assert run.axis.homed is True, "an exact abort must not cost the position reference"
    assert "re-home" not in run.axis.fault


def test_backing_away_from_a_tripped_endstop_is_allowed():
    # The switch sits *inside* the travel limits, so the commanded move really
    # does drive through it rather than stopping politely at min_deg. The
    # supervisor's origin has to agree with where the mechanism actually is, or
    # it thinks it is already at the target and never moves at all.
    config = axis_config(min_deg=0.0, max_deg=360.0, home_position_deg=2.0)
    mechanics = VirtualAxis(
        steps_per_deg=config.steps_per_deg, position_deg=2.0, endstop_deg=0.5, hard_min_deg=-1.0
    )
    run = harness(mechanics, config=config)
    run.axis.goto(0.0)
    assert run.run_until(lambda: run.axis.fault != "", 120)
    run.axis.jog(+5.0)
    assert run.run_until(lambda: mechanics.position_deg > 2.0, 120)
    assert not mechanics.endstop_triggered()


def test_the_firmware_refuses_motion_further_into_a_closed_switch():
    firmware = FakeFirmware(
        mechanics={"az": VirtualAxis(steps_per_deg=100.0, position_deg=0.0, endstop_deg=0.0)}
    )
    firmware.on_host_bytes(sp.Hello().encode())
    firmware.take_output()
    firmware.on_host_bytes(sp.Seg(sp.AXIS_AZ, 0, -1, 10, 1000).encode())
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    assert any(
        isinstance(r, sp.Nack) and r.reason == sp.NackReason.INHIBITED for r in replies
    )
    # ...but away from it is fine, or homing could never re-approach.
    firmware.on_host_bytes(sp.Seg(sp.AXIS_AZ, 1, 1, 10, 1000).encode())
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    assert any(isinstance(r, sp.Ack) for r in replies)


# --------------------------------------------------------------------------
# E-stop
# --------------------------------------------------------------------------

def test_an_estop_stops_both_axes_and_drops_the_enables():
    config = RotatorConfig(
        backend="serial",
        azimuth=axis_config(),
        elevation=axis_config(min_deg=0.0, max_deg=180.0),
        watchdog_s=0.0,
    )
    from fakes.harness import SerialRotatorHarness

    mechanics = {
        "az": VirtualAxis(steps_per_deg=config.azimuth.steps_per_deg),
        "el": VirtualAxis(steps_per_deg=config.elevation.steps_per_deg),
    }
    run = SerialRotatorHarness(config, mechanics)
    run.rotator.goto(40.0, 30.0)
    run.run(1.0)
    run.firmware.set_estop(True)
    run.run(0.2)
    state = run.rotator.state()
    assert "E-stop" in state.fault
    assert all(not axis.enabled for axis in run.firmware.axes)
    assert run.backend.estop_triggered()


# --------------------------------------------------------------------------
# Failure modes decided in advance (PLAN-ARDUINO §7)
# --------------------------------------------------------------------------

def test_a_reboot_mid_pass_faults_the_axes_and_demands_a_re_home():
    config = axis_config(min_deg=0.0, max_deg=360.0)
    mechanics = VirtualAxis(
        steps_per_deg=config.steps_per_deg, position_deg=3.0, endstop_deg=0.0, hard_min_deg=-1.0
    )
    run = harness(mechanics, config=config)
    run.axis.home()
    assert run.run_until(lambda: run.axis.homed, 200)
    run.axis.goto(30.0)
    run.run(1.0)

    run.firmware.reboot()
    assert run.run_until(lambda: run.backend.resets > 0, 5)
    run.run(0.2)
    assert run.axis.homed is False, "a lost firmware queue is a lost position reference"
    assert "re-home" in run.axis.fault

    # The link itself must come back, or a brownout ends the session for good.
    assert run.run_until(lambda: run.backend.ident is not None and run.firmware.ready, 5)
    run.axis.home()
    assert run.run_until(lambda: run.axis.homed, 200), "re-homing must work after a reboot"


def test_an_unplugged_cable_latches_a_fault_and_stops_the_antenna():
    config = axis_config()
    run = harness(VirtualAxis(steps_per_deg=config.steps_per_deg), config=config,
                  wrap_transport=FaultyTransport)
    run.axis.goto(40.0)
    run.run(1.0)
    run.transport.unplugged = True
    assert run.run_until(lambda: run.backend.fault != "", 10)
    assert run.run_until(lambda: run.axis.fault != "", 10)
    assert not run.axis.moving


def test_the_firmware_drains_to_rest_when_the_host_goes_quiet():
    """Link loss is safe *because* every plan terminates at rest. The firmware
    does not abort — it finishes what it holds and stops in a known place."""
    firmware = FakeFirmware()
    firmware.on_host_bytes(sp.Hello().encode())
    firmware.on_host_bytes(sp.Enable(sp.AXIS_ALL, 1).encode())
    firmware.on_host_bytes(sp.Seg(sp.AXIS_AZ, 0, 1, 100, 1000).encode())
    firmware.take_output()

    firmware.clock.advance(0.2)
    firmware.advance(firmware.clock.monotonic())
    assert firmware.axes[0].current is None or firmware.axes[0].current.steps_done > 0

    # Now the host disappears.
    from fakes.fake_firmware import FW_WATCHDOG_S, IDLE_DISABLE_S

    firmware.clock.advance(FW_WATCHDOG_S + IDLE_DISABLE_S + 1.0)
    firmware.advance(firmware.clock.monotonic())
    assert firmware.watchdog_tripped
    assert not firmware.axes[0].enabled, "enables drop once the queue has drained"
    replies = [sp.decode_message(f) for f in sp.FrameDecoder().feed(firmware.take_output())]
    assert not any(isinstance(r, sp.Aborted) for r in replies), "draining is not aborting"


def test_the_host_keeps_the_firmware_watchdog_fed_while_parked():
    run = harness()
    from fakes.fake_firmware import FW_WATCHDOG_S

    run.run(FW_WATCHDOG_S * 2)
    assert not run.firmware.watchdog_tripped
    assert run.backend.fault == ""


# --------------------------------------------------------------------------
# Corruption
# --------------------------------------------------------------------------

def test_the_fault_injector_actually_corrupts():
    """Guard the guard: an injector that quietly passed bytes through would make
    every recovery test below pass by doing nothing."""
    firmware = FakeFirmware()
    transport = FaultyTransport(FirmwareTransport(firmware))

    # Same frame twice: once through the injector armed, once clean.
    firmware.out.clear()
    firmware._send(sp.Pong(7))
    transport.flip_bits_in_next = 1
    corrupted = transport.read()

    firmware.out.clear()
    firmware._send(sp.Pong(7))
    clean = transport.read()

    assert transport.corruptions == 1
    assert corrupted != clean, "the injector must actually change the bytes"
    assert sp.FrameDecoder().feed(clean), "the control frame must decode"
    assert sp.FrameDecoder().feed(corrupted) == [], "the corrupted one must not"

    # ...and dropping a byte must be equally real.
    firmware.out.clear()
    firmware._send(sp.Pong(7))
    transport.drop_bytes_in_next = 1
    shortened = transport.read()
    assert transport.drops == 1
    assert len(shortened) == len(clean) - 1


def test_a_corrupted_reply_costs_one_frame_and_the_link_carries_on():
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config, wrap_transport=FaultyTransport)
    run.axis.goto(8.0)
    run.run(0.3)
    run.transport.flip_bits_in_next = 1
    assert run.settle(120)
    assert run.transport.corruptions == 1
    assert run.backend.link.decoder.crc_errors >= 1
    # The move still completes: a lost DONE is caught up by the next report, and
    # nothing about position was inferred from the missing frame.
    assert run.axis.position_deg == pytest.approx(8.0, abs=0.05)
    assert mechanics.steps_lost == 0


def test_a_lost_completion_report_is_recovered_by_the_next_one():
    """Segments execute in order, so a report about a later sequence number is
    the firmware telling us the earlier ones finished. Without this, one flipped
    bit strands a segment and the axis is `busy` forever."""
    run = harness()
    driver = run.driver
    # 30 ms each, 90 ms total: three fit inside the commitment window, and none
    # finishes during the short pump below.
    for _ in range(3):
        driver.enqueue(Segment(direction=1, steps=30, period_us=1000))
    run.run(0.01)
    seqs = list(driver._outstanding)
    assert len(seqs) == 3

    # The first two DONEs never arrive; only the third does.
    driver.on_done(sp.Done(sp.AXIS_AZ, seqs[2], 30))
    assert driver._outstanding == {}
    assert driver.consumed_steps() == 90
    assert not driver.busy()


def test_a_lost_report_on_the_last_segment_is_recovered_by_asking():
    """The case the ordering rule cannot cover: nothing follows the last
    segment, so the host asks outright rather than waiting forever. Time decides
    when to ask; the firmware still decides what happened."""
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config, wrap_transport=FaultyTransport)
    run.driver.enqueue(Segment(direction=1, steps=20, period_us=1000))
    run.run(0.01)
    assert run.driver.busy()

    # Corrupt whatever comes back next — which is this segment's DONE.
    run.transport.flip_bits_in_next = 1
    assert run.run_until(lambda: not run.driver.busy(), 10), "the axis stayed busy forever"
    assert run.transport.corruptions == 1
    assert run.driver.consumed_steps() == 20
    assert run.axis.position_deg == pytest.approx(
        mechanics.position_deg, abs=1e-9
    ), "recovery must not invent a position the mechanism never reached"


def test_dropped_bytes_resynchronise():
    config = axis_config()
    mechanics = VirtualAxis(steps_per_deg=config.steps_per_deg)
    run = harness(mechanics, config=config, wrap_transport=FaultyTransport)
    run.axis.goto(8.0)
    run.run(0.3)
    run.transport.drop_bytes_in_next = 2
    assert run.settle(120)
    assert run.transport.drops == 2
    assert run.axis.position_deg == pytest.approx(8.0, abs=0.05)


# --------------------------------------------------------------------------
# Diagnostics
# --------------------------------------------------------------------------

def test_status_reports_what_the_board_thinks_is_going_on():
    run = harness()
    run.driver.enqueue(Segment(direction=1, steps=2000, period_us=1000))
    run.run(0.02)
    state = run.backend.request_state()
    assert state is not None
    assert state.axes[0].queue_len >= 1
    assert state.estop == 0


def test_ping_round_trips():
    backend, _, _ = bare_backend()
    assert backend.ping() is True


def test_ping_fails_when_the_board_is_gone():
    firmware = FakeFirmware()
    transport = FaultyTransport(FirmwareTransport(firmware))
    backend, _, _ = bare_backend(firmware=firmware, transport=transport)
    transport.unplugged = True
    assert backend.ping() is False
