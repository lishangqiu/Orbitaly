"""The wire protocol, and the two implementations of it agreeing.

The protocol exists twice — Python on the Pi, C++ on the Arduino — because half
of it has to run on an 8-bit microcontroller. That is a standing invitation to
drift, so this module does three jobs:

1. checks the Python implementation against externally-anchored values (the
   published CRC-16/CCITT-FALSE check value, and hand-derivable field layouts);
2. checks it against the golden vectors the C++ side is also fed;
3. parses `firmware/orbitaly_rotator/protocol.h` and asserts every constant and
   every message length matches, in both directions — a field added on one side
   and forgotten on the other fails here rather than on the bench.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from orbitaly.motion import serial_protocol as sp

PROTOCOL_H = Path(__file__).parent.parent / "firmware" / "orbitaly_rotator" / "protocol.h"
QUEUE_H = Path(__file__).parent.parent / "firmware" / "orbitaly_rotator" / "queue.h"
GOLDEN = Path(__file__).parent.parent / "firmware" / "test" / "golden_frames.txt"


def parse_defines(path: Path) -> dict[str, int]:
    """Pull `#define ORB_NAME value` out of a header. Integers only."""
    text = path.read_text()
    found: dict[str, int] = {}
    for name, value in re.findall(r"^#define\s+(ORB_\w+)\s+(0[xX][0-9a-fA-F]+|\d+)\s*$", text, re.M):
        found[name] = int(value, 0)
    return found


# --------------------------------------------------------------------------
# The CRC, anchored outside this repository
# --------------------------------------------------------------------------

def test_the_crc_is_really_crc16_ccitt_false():
    """0x29B1 over "123456789" is the published check value for this CRC.

    Without this, both implementations could be consistently wrong and every
    other test in this file would still pass — they only ever compare the two
    against each other.
    """
    assert sp.crc16_ccitt(b"123456789") == 0x29B1


def test_the_crc_reacts_to_every_byte_and_to_order():
    assert sp.crc16_ccitt(b"") == 0xFFFF
    assert sp.crc16_ccitt(b"\x00\x01") != sp.crc16_ccitt(b"\x01\x00")
    assert sp.crc16_ccitt(b"\x00") != sp.crc16_ccitt(b"\x00\x00")


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------

def test_a_frame_has_the_documented_layout():
    frame = sp.encode_frame(sp.MsgType.PING, b"\xc3")
    assert frame[0] == sp.SOF
    assert frame[1] == 1                      # payload length
    assert frame[2] == sp.MsgType.PING
    assert frame[3] == 0xC3
    # CRC covers len..payload, little-endian, and nothing else.
    expected = sp.crc16_ccitt(bytes([1, int(sp.MsgType.PING), 0xC3]))
    assert frame[4] | (frame[5] << 8) == expected
    assert len(frame) == sp.FRAME_OVERHEAD + 1


def test_an_oversized_payload_is_refused_rather_than_truncated():
    with pytest.raises(sp.ProtocolError):
        sp.encode_frame(sp.MsgType.PING, b"\x00" * (sp.MAX_PAYLOAD + 1))


def test_frames_survive_being_delivered_one_byte_at_a_time():
    """The real arrival pattern on a UART. A decoder that only works on whole
    frames works only in tests."""
    frame = sp.Seg(sp.AXIS_AZ, 3, -1, 27000, 4_000_000).encode()
    decoder = sp.FrameDecoder()
    out = []
    for byte in frame:
        out += decoder.feed(bytes([byte]))
    assert len(out) == 1
    message = sp.decode_message(out[0])
    assert message == sp.Seg(sp.AXIS_AZ, 3, -1, 27000, 4_000_000)


def test_a_start_byte_inside_a_payload_is_not_mistaken_for_a_frame_start():
    frame = sp.Ping(sp.SOF).encode()
    assert sp.SOF in frame[3:]
    decoder = sp.FrameDecoder()
    frames = decoder.feed(frame + sp.Ping(1).encode())
    assert [f.type for f in frames] == [sp.MsgType.PING, sp.MsgType.PING]
    assert decoder.crc_errors == 0


def test_a_corrupted_frame_costs_one_frame_and_the_link_recovers():
    bad = bytearray(sp.Ping(0x11).encode())
    bad[-1] ^= 0xFF
    decoder = sp.FrameDecoder()
    frames = decoder.feed(bytes(bad) + sp.Hello().encode())
    assert [f.type for f in frames] == [sp.MsgType.HELLO]
    assert decoder.crc_errors == 1


def test_an_impossible_length_does_not_stall_the_decoder():
    """The failure that wedges naive state machines: a corrupted length field
    makes them wait for bytes that are never coming, so every frame behind it
    is lost too."""
    bad = bytearray(sp.Ack(sp.AXIS_AZ, 3, 8).encode())
    bad[1] = 0xFE  # far past MAX_PAYLOAD
    decoder = sp.FrameDecoder()
    frames = decoder.feed(bytes(bad) + sp.Hello().encode())
    assert [f.type for f in frames] == [sp.MsgType.HELLO]


def test_every_single_bit_corruption_of_a_frame_is_caught():
    """A CRC that let one through would be worse than none, because the result
    would be trusted."""
    frame = sp.Seg(sp.AXIS_EL, 12, 1, 500, 1406).encode()
    undetected = []
    for index in range(1, len(frame)):  # the start byte is outside the CRC
        for bit in range(8):
            corrupted = bytearray(frame)
            corrupted[index] ^= 1 << bit
            decoded = sp.FrameDecoder().feed(bytes(corrupted))
            if decoded and decoded[0] == sp.Frame(frame[2], frame[3:-2]):
                undetected.append((index, bit))
    assert undetected == []


def test_the_decoder_counts_what_it_threw_away():
    decoder = sp.FrameDecoder()
    decoder.feed(b"\x00\x11\x22" + sp.Hello().encode())
    assert decoder.resyncs >= 1


def test_a_partial_frame_is_held_not_dropped():
    frame = sp.Hello().encode()
    decoder = sp.FrameDecoder()
    assert decoder.feed(frame[:-1]) == []
    assert decoder.pending_bytes == len(frame) - 1
    assert len(decoder.feed(frame[-1:])) == 1


# --------------------------------------------------------------------------
# Messages
# --------------------------------------------------------------------------

ROUND_TRIP = [
    sp.Hello(sp.PROTOCOL_VERSION),
    sp.Seg(sp.AXIS_AZ, 0, 1, 32, 625),
    sp.Seg(sp.AXIS_EL, sp.SEQ_MAX, -1, 65535, 4_294_967_295),
    sp.Abort(sp.AXIS_ALL),
    sp.Enable(sp.AXIS_EL, 1),
    sp.ClearFault(sp.AXIS_AZ),
    sp.Ping(0xC3),
    sp.Status(),
    sp.Ack(sp.AXIS_AZ, 12, 7),
    sp.Nack(sp.AXIS_EL, 12, sp.NackReason.QUEUE_FULL),
    sp.Done(sp.AXIS_AZ, 12, 27000),
    sp.Aborted(sp.AXIS_EL, 9, 431, sp.AbortCause.ENDSTOP),
    sp.Event(sp.EventKind.ENDSTOP_AZ, 1),
    sp.Pong(0xC3),
]


@pytest.mark.parametrize("message", ROUND_TRIP, ids=lambda m: type(m).__name__)
def test_every_message_round_trips(message):
    frames = sp.FrameDecoder().feed(message.encode())
    assert len(frames) == 1
    assert sp.decode_message(frames[0]) == message


def test_ident_round_trips_with_its_pin_table():
    ident = sp.Ident(
        protocol=sp.PROTOCOL_VERSION,
        fw_major=1,
        fw_minor=2,
        fw_patch=3,
        queue_depth=8,
        axis_count=2,
        caps=sp.Caps.EXACT_ABORT | sp.Caps.ENDSTOP_NC,
        dir_setup_us=20,
        pulse_width_us=5,
        endstop_debounce_ms=5,
        pins=(3, 4, 5, 6, 7, 8, 9, sp.PIN_NONE),
    )
    decoded = sp.decode_message(sp.FrameDecoder().feed(ident.encode())[0])
    assert decoded == ident
    assert decoded.version_str == "1.2.3"
    assert decoded.has(sp.Caps.EXACT_ABORT)
    assert not decoded.has(sp.Caps.ESTOP_FITTED)
    assert decoded.pins[7] == sp.PIN_NONE


def test_state_round_trips_and_is_self_describing():
    state = sp.State(
        axes=(sp.AxisState(3, sp.AxisFlags.ENABLED, 0), sp.AxisState(0, sp.AxisFlags.INHIBIT_NEG, 1)),
        estop=0,
    )
    decoded = sp.decode_message(sp.FrameDecoder().feed(state.encode())[0])
    assert decoded == state


def test_state_rejects_a_payload_that_contradicts_its_own_axis_count():
    with pytest.raises(sp.ProtocolError):
        sp.State.decode(b"\x02\x00\x00\x00\x00")  # claims two axes, carries one


def test_a_negative_direction_survives_as_a_signed_byte():
    """`dir` is the one signed field on the wire; an unsigned read turns -1 into
    255 and the antenna goes the wrong way."""
    encoded = sp.Seg(sp.AXIS_AZ, 0, -1, 10, 625).encode()
    assert encoded[3 + 2] == 0xFF
    assert sp.decode_message(sp.FrameDecoder().feed(encoded)[0]).direction == -1


def test_message_types_carry_their_direction_in_the_high_bit():
    for msg_type in sp.MsgType:
        assert msg_type.from_firmware == (msg_type >= 0x80)
    assert sp.MsgType.SEG.from_firmware is False
    assert sp.MsgType.DONE.from_firmware is True


def test_an_unknown_message_type_is_rejected_not_guessed():
    with pytest.raises(sp.ProtocolError):
        sp.decode_message(sp.Frame(type=0x7E, payload=b""))


def test_a_payload_that_does_not_fit_its_type_is_rejected():
    with pytest.raises(sp.ProtocolError):
        sp.Done.decode(b"\x00\x01")  # DONE needs four bytes


def test_sequence_numbers_wrap_without_ever_producing_the_reserved_value():
    seq = 0
    seen = set()
    for _ in range(600):
        seen.add(seq)
        seq = sp.next_seq(seq)
    assert sp.SEQ_NONE not in seen
    assert max(seen) == sp.SEQ_MAX
    assert sp.next_seq(sp.SEQ_MAX) == 0


# --------------------------------------------------------------------------
# The golden vectors, shared with the C++ implementation
# --------------------------------------------------------------------------

def _golden_records():
    assert GOLDEN.exists(), f"{GOLDEN} is missing — run scripts/make_golden_frames.py"
    for line in GOLDEN.read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            yield line.split()


def _unhex(field: str) -> bytes:
    return b"" if field == "-" else bytes.fromhex(field)


def test_the_golden_vectors_still_describe_this_implementation():
    """Regenerating the vectors must be a deliberate act.

    If a protocol change is intended, `python scripts/make_golden_frames.py`
    updates the file and the diff shows exactly which bytes moved — which is the
    review the firmware side deserves.
    """
    checked = {"CRC": 0, "FRAME": 0, "STREAM": 0}
    for record in _golden_records():
        kind = record[0]
        if kind == "CRC":
            assert sp.crc16_ccitt(_unhex(record[1])) == int(record[2], 16)
        elif kind == "FRAME":
            msg_type, payload, encoded = int(record[1], 16), _unhex(record[2]), _unhex(record[3])
            assert sp.encode_frame(msg_type, payload) == encoded
            frames = sp.FrameDecoder().feed(encoded)
            assert len(frames) == 1
            assert frames[0].type == msg_type and frames[0].payload == payload
        elif kind == "STREAM":
            decoder = sp.FrameDecoder()
            frames = decoder.feed(_unhex(record[1]))
            expected = list(_unhex(record[2]))
            assert [f.type for f in frames] == expected
            assert decoder.crc_errors == int(record[3])
        else:
            pytest.fail(f"unknown record kind {kind!r} in {GOLDEN}")
        checked[kind] += 1
    # Guard the guard: an empty or truncated vector file would otherwise make
    # this test pass by checking nothing.
    assert checked["CRC"] >= 3 and checked["FRAME"] >= 15 and checked["STREAM"] >= 5, checked


# --------------------------------------------------------------------------
# The two implementations agree
# --------------------------------------------------------------------------

#: Constants that exist in both, and how to find the Python one.
_SCALARS = {
    "ORB_SOF": lambda: sp.SOF,
    "ORB_PROTOCOL_VERSION": lambda: sp.PROTOCOL_VERSION,
    "ORB_MAX_PAYLOAD": lambda: sp.MAX_PAYLOAD,
    "ORB_FRAME_OVERHEAD": lambda: sp.FRAME_OVERHEAD,
    "ORB_SEQ_NONE": lambda: sp.SEQ_NONE,
    "ORB_SEQ_MAX": lambda: sp.SEQ_MAX,
    "ORB_PIN_NONE": lambda: sp.PIN_NONE,
    "ORB_AXIS_AZ": lambda: sp.AXIS_AZ,
    "ORB_AXIS_EL": lambda: sp.AXIS_EL,
    "ORB_AXIS_ALL": lambda: sp.AXIS_ALL,
}

#: Prefixed families: every member of the Python enum must appear in the header
#: with the same value, and vice versa.
_FAMILIES = [
    ("ORB_MSG_", sp.MsgType),
    ("ORB_NACK_", sp.NackReason),
    ("ORB_ABORT_", sp.AbortCause),
    ("ORB_EVENT_", sp.EventKind),
    ("ORB_CAP_", sp.Caps),
    ("ORB_FLAG_", sp.AxisFlags),
]


def test_the_header_defines_every_scalar_with_the_same_value():
    defines = parse_defines(PROTOCOL_H)
    for name, getter in _SCALARS.items():
        assert name in defines, f"{name} is missing from protocol.h"
        assert defines[name] == getter(), f"{name}: header says {defines[name]}, Python says {getter()}"


@pytest.mark.parametrize("prefix,enum", _FAMILIES, ids=[p for p, _ in _FAMILIES])
def test_the_header_and_python_agree_on_every_enum_member(prefix, enum):
    defines = parse_defines(PROTOCOL_H)
    header = {name[len(prefix):]: value for name, value in defines.items() if name.startswith(prefix)}
    python = {member.name: int(member) for member in enum}
    # Both directions: a member added to one side and forgotten on the other is
    # the failure this whole test exists for.
    assert header == python


def test_the_header_agrees_on_every_payload_length():
    """A field width that differs between the two sides decodes as garbage that
    still passes CRC, which is the worst kind of wrong."""
    defines = parse_defines(PROTOCOL_H)
    lengths = {name[len("ORB_LEN_"):]: value for name, value in defines.items() if name.startswith("ORB_LEN_")}
    expected = {}
    for msg_type, cls in sp.DECODERS.items():
        if cls is sp.State:
            continue  # variable-length by design; its own round-trip test covers it
        expected[sp.MsgType(msg_type).name] = cls.STRUCT.size
    assert lengths == expected


def test_the_header_matches_the_documented_frame_geometry():
    defines = parse_defines(PROTOCOL_H)
    # The plan's throughput argument depends on a SEG being small; IDENT is the
    # only message near the payload ceiling.
    assert defines["ORB_LEN_SEG"] == 9
    assert defines["ORB_LEN_IDENT"] <= defines["ORB_MAX_PAYLOAD"]
    assert max(v for k, v in defines.items() if k.startswith("ORB_LEN_")) <= defines["ORB_MAX_PAYLOAD"]


def test_the_firmware_queue_depth_is_what_the_host_expects():
    """The host's flow control gates on this number; if the two disagree the
    host either starves the queue or overruns it."""
    depth = parse_defines(QUEUE_H)["ORB_QUEUE_DEPTH"]
    from orbitaly.motion.serial_driver import DEFAULT_QUEUE_DEPTH

    assert depth == DEFAULT_QUEUE_DEPTH
