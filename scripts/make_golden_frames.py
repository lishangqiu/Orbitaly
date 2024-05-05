#!/usr/bin/env python3
"""Emit the frame vectors both protocol implementations are checked against.

    python scripts/make_golden_frames.py

Why this exists
    The serial protocol is written twice — once in Python
    (`orbitaly/motion/serial_protocol.py`) and once in C++
    (`firmware/orbitaly_rotator/framing.cpp`), because half of it has to run on
    an 8-bit microcontroller. Two implementations of one protocol will drift
    unless something forces them to agree on actual bytes, so both are fed the
    same file: `firmware/test/golden_frames.txt`.

    Python's side is checked by `tests/test_serial_protocol.py`; the C++ side by
    `firmware/test/test_firmware_core.cpp`, which pytest compiles and runs when
    a host C++ compiler is available (and skips, visibly, when it is not).

    The vectors are generated from the Python implementation, which would make
    them circular on their own. They are not, because the file also carries the
    published CRC-16/CCITT-FALSE check value for "123456789" (0x29B1) — an
    external anchor neither implementation can satisfy by being consistently
    wrong.

Format
    Deliberately not JSON: the C++ reader is 40 lines of `sscanf` and has no
    dependencies, which is the same reason the wire protocol is bespoke bytes.

        CRC    <data-hex> <crc-hex>
        FRAME  <type-hex> <payload-hex> <encoded-frame-hex>
        STREAM <input-hex> <expected-type-sequence-hex> <expected-crc-errors>
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orbitaly.motion import serial_protocol as sp  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "firmware" / "test" / "golden_frames.txt"

#: The published check value for CRC-16/CCITT-FALSE. This is the anchor that
#: keeps the whole exercise from being self-referential.
CRC_CHECK_INPUT = b"123456789"
CRC_CHECK_VALUE = 0x29B1


def _messages():
    """One of every message, with values chosen to expose byte-order errors.

    Nothing here is round: a period of 1406 us and a step count of 27000 both
    span two bytes with distinct halves, so a little-endian/big-endian mix-up
    fails loudly instead of passing on a palindrome.
    """
    return [
        ("hello", sp.Hello(sp.PROTOCOL_VERSION)),
        # The reference azimuth segment: 6 deg/s at 266.7 steps/deg = 625 us.
        ("seg_az_forward", sp.Seg(sp.AXIS_AZ, 0, 1, 32, 625)),
        # Elevation's reference rate, and a negative direction to pin down the
        # signed byte.
        ("seg_el_reverse", sp.Seg(sp.AXIS_EL, 254, -1, 14, 1406)),
        # A whole slew in one segment: 27000 steps, and a period long enough to
        # need all four bytes.
        ("seg_wide_fields", sp.Seg(sp.AXIS_AZ, 7, 1, 27000, 4_000_000)),
        ("abort_all", sp.Abort(sp.AXIS_ALL)),
        ("abort_az", sp.Abort(sp.AXIS_AZ)),
        ("enable_on", sp.Enable(sp.AXIS_EL, 1)),
        ("clear_fault", sp.ClearFault(sp.AXIS_AZ)),
        ("ping", sp.Ping(0xC3)),
        ("status", sp.Status()),
        (
            "ident",
            sp.Ident(
                protocol=sp.PROTOCOL_VERSION,
                fw_major=1,
                fw_minor=0,
                fw_patch=0,
                queue_depth=8,
                axis_count=2,
                caps=(
                    sp.Caps.EXACT_ABORT
                    | sp.Caps.ENDSTOP_NC
                    | sp.Caps.ENABLE_ACTIVE_LOW
                    | sp.Caps.ESTOP_FITTED
                    | sp.Caps.AZ_ENDSTOP
                    | sp.Caps.EL_ENDSTOP
                ),
                dir_setup_us=20,
                pulse_width_us=5,
                endstop_debounce_ms=5,
                pins=(3, 4, 5, 6, 7, 8, 9, 10),
            ),
        ),
        ("ack", sp.Ack(sp.AXIS_AZ, 12, 7)),
        ("nack_queue_full", sp.Nack(sp.AXIS_EL, 12, sp.NackReason.QUEUE_FULL)),
        ("done", sp.Done(sp.AXIS_AZ, 12, 27000)),
        ("aborted_endstop", sp.Aborted(sp.AXIS_EL, 9, 431, sp.AbortCause.ENDSTOP)),
        ("aborted_nothing_in_flight", sp.Aborted(sp.AXIS_AZ, sp.SEQ_NONE, 0, sp.AbortCause.HOST)),
        ("event_endstop", sp.Event(sp.EventKind.ENDSTOP_AZ, 1)),
        ("pong", sp.Pong(0xC3)),
    ]


def _streams():
    """Byte streams that exercise the resynchronisation rules.

    These are the cases that decide whether a corrupted link recovers or wedges,
    and they are why the decoder is a buffer-and-rescan rather than a state
    machine: a state machine that has committed to a bogus length has no way
    back to the frame sitting right behind it.
    """
    hello = sp.Hello().encode()
    ping = sp.Ping(0x11).encode()
    ack = sp.Ack(sp.AXIS_AZ, 3, 8).encode()

    corrupt_crc = bytearray(ping)
    corrupt_crc[-1] ^= 0xFF

    bad_length = bytearray(ack)
    bad_length[1] = 0xFE  # impossible length: > MAX_PAYLOAD

    return [
        ("clean_back_to_back", hello + ping + ack, [sp.MsgType.HELLO, sp.MsgType.PING, sp.MsgType.ACK], 0),
        # Leading noise, including a false start byte, before a real frame.
        ("leading_noise", bytes([0x00, 0xFF, 0xA5, 0x13]) + hello, [sp.MsgType.HELLO], 0),
        # A frame with a broken CRC, followed by a good one: exactly one CRC
        # error, and the good frame still arrives.
        ("crc_error_then_recovery", bytes(corrupt_crc) + hello, [sp.MsgType.HELLO], 1),
        # An impossible length field must not stall the reader waiting for bytes
        # that are never coming.
        ("bad_length_then_recovery", bytes(bad_length) + hello, [sp.MsgType.HELLO], 0),
        # A start byte inside a payload must not be mistaken for a frame start.
        ("sof_inside_payload", sp.Ping(sp.SOF).encode() + ping, [sp.MsgType.PING, sp.MsgType.PING], 0),
        # Truncated tail: the complete frame comes out, the partial one waits.
        ("truncated_tail", hello + ping[:4], [sp.MsgType.HELLO], 0),
    ]


def _hex(data: bytes) -> str:
    """Hex, with '-' for empty — see the format note in the file header."""
    return data.hex() if data else "-"


def main() -> int:
    if sp.crc16_ccitt(CRC_CHECK_INPUT) != CRC_CHECK_VALUE:
        raise SystemExit(
            "refusing to generate vectors: this CRC is not CRC-16/CCITT-FALSE "
            f"(check value came out 0x{sp.crc16_ccitt(CRC_CHECK_INPUT):04X}, want 0x{CRC_CHECK_VALUE:04X})"
        )

    lines = [
        "# Golden frame vectors for the Orbitaly serial protocol.",
        "# Generated by scripts/make_golden_frames.py — do not edit by hand.",
        "# Read by tests/test_serial_protocol.py and firmware/test/test_firmware_core.cpp.",
        "#",
        "# CRC    <data-hex> <crc-hex>",
        "# FRAME  <type-hex> <payload-hex> <encoded-frame-hex>",
        "# STREAM <input-hex> <expected-types-hex> <expected-crc-errors>",
        "#",
        "# An empty hex field is written '-', so every record has the same number",
        "# of whitespace-separated fields and the C++ reader stays a sscanf.",
        "",
    ]

    # The external anchor first, so a reader that stops early still checks it.
    lines.append(f"CRC {_hex(CRC_CHECK_INPUT)} {CRC_CHECK_VALUE:04x}")
    for data in (b"", b"\x00", b"\xff", bytes(range(16))):
        lines.append(f"CRC {_hex(data)} {sp.crc16_ccitt(data):04x}")
    lines.append("")

    for name, message in _messages():
        encoded = message.encode()
        payload = encoded[3 : 3 + encoded[1]]
        lines.append(f"FRAME {int(message.TYPE):02x} {_hex(payload)} {_hex(encoded)}  # {name}")
    lines.append("")

    for name, stream, expected, crc_errors in _streams():
        types = "".join(f"{int(t):02x}" for t in expected) or "-"
        lines.append(f"STREAM {_hex(stream)} {types} {crc_errors}  # {name}")
    lines.append("")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines))
    print(f"Wrote {OUT} ({len(lines)} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
