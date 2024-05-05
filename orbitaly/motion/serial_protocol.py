"""The wire protocol between Orbitaly and the Arduino pulse executor.

Pure bytes. No I/O, no threads, no motion policy — this module knows how to
turn a message into a frame and a stream of bytes back into messages, and
nothing else. That is what makes it the testable core: every framing rule,
every field width and every resynchronisation path can be exercised without a
serial port in sight.

The same constants exist a second time in ``firmware/orbitaly_rotator/protocol.h``
and ``tests/test_serial_protocol.py`` asserts the two agree field by field.
Two implementations of one protocol is the cost of putting half of it on an
8-bit microcontroller; silent drift between them is not.

Frame layout, little-endian throughout::

    0xA5 | len (u8) | type (u8) | payload[len] | crc16-ccitt (u16)

The CRC covers ``len``, ``type`` and the payload — everything but the start
byte, which carries no information a CRC could protect. A receiver that fails
a CRC drops one byte and rescans for the next ``0xA5``, so a corrupted length
field costs at most one frame rather than desynchronising the link forever.

Message types have their high bit set when the firmware is the sender. That is
not decoration: on a loopback jumper, or with two hosts accidentally on one
bus, a frame coming back the way it went out is recognisable as wrong-way
traffic instead of being parsed as a plausible command.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar

#: Start-of-frame byte. 0xA5 is the conventional choice: alternating bits, so
#: it is unlikely to appear in a stuck-line or floating-input failure.
SOF = 0xA5

#: Bumped for any change to field widths, message types or semantics. The
#: handshake refuses a mismatch, so a stale flash fails in ``orbitaly doctor``
#: rather than halfway through a pass.
PROTOCOL_VERSION = 1

#: Longest payload any message carries (IDENT, at 20 bytes). A whole frame is
#: therefore at most 25 bytes and fits inside the AVR's 64-byte UART buffer
#: with room for two more behind it. Only IDENT is anywhere near this; SEG —
#: the only message sent at rate — is 9 bytes of payload, 14 on the wire.
MAX_PAYLOAD = 24

FRAME_OVERHEAD = 5  # SOF + len + type + crc16

#: Sequence numbers are per-axis and wrap. 0xFF is reserved so ABORTED can say
#: "nothing was in flight" without inventing a second message.
SEQ_NONE = 0xFF
SEQ_MAX = 0xFE

#: Pin numbers in IDENT use this for "not fitted".
PIN_NONE = 0xFF

AXIS_AZ = 0
AXIS_EL = 1
AXIS_ALL = 0xFF


class MsgType(IntEnum):
    # host -> firmware
    HELLO = 0x01
    SEG = 0x02
    ABORT = 0x03
    ENABLE = 0x04
    CLEAR_FAULT = 0x05
    PING = 0x06
    STATUS = 0x07
    # firmware -> host
    IDENT = 0x81
    ACK = 0x82
    NACK = 0x83
    DONE = 0x84
    ABORTED = 0x85
    EVENT = 0x86
    STATE = 0x87
    PONG = 0x88

    @property
    def from_firmware(self) -> bool:
        return bool(self & 0x80)


class NackReason(IntEnum):
    BAD_CRC = 1
    BAD_LENGTH = 2
    BAD_TYPE = 3
    BAD_AXIS = 4
    QUEUE_FULL = 5
    INHIBITED = 6
    BAD_VERSION = 7
    NOT_READY = 8  # a command arrived before a version-matched HELLO


class AbortCause(IntEnum):
    HOST = 1
    ENDSTOP = 2
    ESTOP = 3
    WATCHDOG = 4


class EventKind(IntEnum):
    ENDSTOP_AZ = 1
    ENDSTOP_EL = 2
    ESTOP = 3


class Caps(IntEnum):
    """Firmware capability and wiring-convention bits, reported in IDENT."""

    EXACT_ABORT = 0x0001       # ABORTED carries a true executed-pulse count
    ENDSTOP_NC = 0x0002        # endstops are normally-closed (the fail-safe wiring)
    ENABLE_ACTIVE_LOW = 0x0004  # A4988 / DRV8825 / TMC2209 convention
    ESTOP_FITTED = 0x0008
    AZ_ENDSTOP = 0x0010
    EL_ENDSTOP = 0x0020


class AxisFlags(IntEnum):
    """Per-axis bits in a STATE report."""

    INHIBIT_NEG = 0x01  # motion toward the endstop is refused
    INHIBIT_POS = 0x02
    ENABLED = 0x04
    FAULT = 0x08


class ProtocolError(Exception):
    """A frame decoded cleanly but its payload does not fit its type."""


# --------------------------------------------------------------------------
# CRC
# --------------------------------------------------------------------------

def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.

    Written bitwise rather than with a lookup table on purpose — this is the
    reference the AVR implementation is checked against, and 256 entries of
    table would cost more of the ATmega's flash than the loop costs in time at
    1.5 kB/s.
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Frame:
    type: int
    payload: bytes


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload of {len(payload)} bytes exceeds MAX_PAYLOAD={MAX_PAYLOAD}")
    body = bytes((len(payload), int(msg_type))) + payload
    return bytes((SOF,)) + body + struct.pack("<H", crc16_ccitt(body))


class FrameDecoder:
    """Streaming frame reassembler. Feed it bytes, take frames out.

    Counts what it rejected as well as what it accepted: a link that is
    working but resyncing constantly looks identical to a healthy one unless
    somebody keeps score, and ``selftest --serial`` reports these.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    #: parse outcomes
    _OK, _INCOMPLETE, _BAD = 0, 1, 2

    def _parse_at(self, offset: int) -> tuple[int, Frame | None, int]:
        """Try to read a frame starting at ``offset``. Never mutates the buffer."""
        available = len(self._buf) - offset
        if available < 3:
            return (self._INCOMPLETE, None, 0)  # need SOF, len, type
        length = self._buf[offset + 1]
        if length > MAX_PAYLOAD:
            return (self._BAD, None, 0)
        total = FRAME_OVERHEAD + length
        if available < total:
            return (self._INCOMPLETE, None, 0)
        body = bytes(self._buf[offset + 1 : offset + 3 + length])
        (crc_rx,) = struct.unpack_from("<H", self._buf, offset + 3 + length)
        if crc16_ccitt(body) != crc_rx:
            return (self._BAD, None, 0)
        frame = Frame(
            type=self._buf[offset + 2],
            payload=bytes(self._buf[offset + 3 : offset + 3 + length]),
        )
        return (self._OK, frame, total)

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        frames: list[Frame] = []
        while True:
            start = self._buf.find(SOF)
            if start < 0:
                # No start byte anywhere: everything held is noise.
                if self._buf:
                    self.resyncs += 1
                    self._buf.clear()
                break
            if start:
                self.resyncs += 1
                del self._buf[:start]

            status, frame, total = self._parse_at(0)
            if status == self._OK:
                assert frame is not None
                frames.append(frame)
                del self._buf[:total]
                continue
            if status == self._BAD:
                # Bad CRC or an impossible length: drop the false start byte and
                # rescan. This is also what stops a corrupted length field
                # stalling the decoder forever, waiting for bytes that are never
                # coming.
                if self._buf[1] <= MAX_PAYLOAD:
                    self.crc_errors += 1
                else:
                    self.resyncs += 1
                del self._buf[0]
                continue

            # Incomplete. That is the *normal* case for a frame still arriving —
            # but it is also what a false start byte looks like when its length
            # field happens to be plausible, and a real frame may be sitting
            # right behind it. So before settling in to wait, check whether a
            # later start byte already yields a complete, CRC-valid frame; if it
            # does, the thing we were waiting on was never a frame at all.
            # Without this, whether a frame is seen depends on how the UART
            # happened to chunk the bytes.
            recovered = self._recover_from_later_sof()
            if recovered is None:
                break
            self.resyncs += 1
            del self._buf[:recovered]
        return frames

    def _recover_from_later_sof(self) -> int | None:
        """Offset of a later start byte that parses cleanly, if there is one."""
        offset = self._buf.find(SOF, 1)
        while offset >= 0:
            status, _, _ = self._parse_at(offset)
            if status == self._OK:
                return offset
            if status == self._INCOMPLETE:
                return None  # not enough bytes to judge; wait for more
            offset = self._buf.find(SOF, offset + 1)
        return None

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)


# --------------------------------------------------------------------------
# Messages
# --------------------------------------------------------------------------

class _Message:
    """Base for fixed-layout messages: one struct, one type byte."""

    TYPE: ClassVar[MsgType]
    STRUCT: ClassVar[struct.Struct]

    def encode(self) -> bytes:
        return encode_frame(self.TYPE, self._pack())

    def _pack(self) -> bytes:
        raise NotImplementedError

    @classmethod
    def decode(cls, payload: bytes):
        if len(payload) != cls.STRUCT.size:
            raise ProtocolError(
                f"{cls.__name__} wants {cls.STRUCT.size} payload bytes, got {len(payload)}"
            )
        return cls(*cls.STRUCT.unpack(payload))


@dataclass(frozen=True)
class Hello(_Message):
    TYPE = MsgType.HELLO
    STRUCT = struct.Struct("<B")
    version: int = PROTOCOL_VERSION

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.version)


@dataclass(frozen=True)
class Seg(_Message):
    """One :class:`~orbitaly.motion.segment.Segment` on the wire.

    ``counts`` deliberately does not travel. Whether pulses represent logical
    motion or backlash take-up is a planner concept; the firmware's job is to
    emit them either way, and giving it a field it must not act on invites it
    to act on it.
    """

    TYPE = MsgType.SEG
    STRUCT = struct.Struct("<BBbHI")
    axis: int
    seq: int
    direction: int
    steps: int
    period_us: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.seq, self.direction, self.steps, self.period_us)


@dataclass(frozen=True)
class Abort(_Message):
    TYPE = MsgType.ABORT
    STRUCT = struct.Struct("<B")
    axis: int = AXIS_ALL

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis)


@dataclass(frozen=True)
class Enable(_Message):
    TYPE = MsgType.ENABLE
    STRUCT = struct.Struct("<BB")
    axis: int
    on: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.on)


@dataclass(frozen=True)
class ClearFault(_Message):
    TYPE = MsgType.CLEAR_FAULT
    STRUCT = struct.Struct("<B")
    axis: int = AXIS_ALL

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis)


@dataclass(frozen=True)
class Ping(_Message):
    TYPE = MsgType.PING
    STRUCT = struct.Struct("<B")
    nonce: int = 0

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.nonce)


@dataclass(frozen=True)
class Status(_Message):
    TYPE = MsgType.STATUS
    STRUCT = struct.Struct("")

    def _pack(self) -> bytes:
        return b""

    @classmethod
    def decode(cls, payload: bytes) -> "Status":
        if payload:
            raise ProtocolError("STATUS takes no payload")
        return cls()


@dataclass(frozen=True)
class Ident(_Message):
    """Who the firmware is and how it is wired.

    Sent unsolicited at boot as well as in answer to HELLO — an IDENT nobody
    asked for means the board reset, which is the one event that invalidates
    every position the host believes in.
    """

    TYPE = MsgType.IDENT
    STRUCT = struct.Struct("<BBBBBBHHBB8B")
    protocol: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    queue_depth: int
    axis_count: int
    caps: int
    dir_setup_us: int
    pulse_width_us: int
    endstop_debounce_ms: int
    #: az step/dir/enable/endstop then el step/dir/enable/endstop; PIN_NONE = unfitted
    pins: tuple[int, ...] = (PIN_NONE,) * 8

    def _pack(self) -> bytes:
        return self.STRUCT.pack(
            self.protocol,
            self.fw_major,
            self.fw_minor,
            self.fw_patch,
            self.queue_depth,
            self.axis_count,
            self.caps,
            self.dir_setup_us,
            self.pulse_width_us,
            self.endstop_debounce_ms,
            *self.pins,
        )

    @classmethod
    def decode(cls, payload: bytes) -> "Ident":
        if len(payload) != cls.STRUCT.size:
            raise ProtocolError(
                f"IDENT wants {cls.STRUCT.size} payload bytes, got {len(payload)}"
            )
        fields = cls.STRUCT.unpack(payload)
        return cls(*fields[:10], pins=tuple(fields[10:]))

    @property
    def version_str(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    def has(self, cap: Caps) -> bool:
        return bool(self.caps & cap)


@dataclass(frozen=True)
class Ack(_Message):
    TYPE = MsgType.ACK
    STRUCT = struct.Struct("<BBB")
    axis: int
    seq: int
    slots_free: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.seq, self.slots_free)


@dataclass(frozen=True)
class Nack(_Message):
    TYPE = MsgType.NACK
    STRUCT = struct.Struct("<BBB")
    axis: int
    seq: int
    reason: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.seq, self.reason)

    @property
    def reason_name(self) -> str:
        try:
            return NackReason(self.reason).name.lower().replace("_", " ")
        except ValueError:
            return f"reason {self.reason}"


@dataclass(frozen=True)
class Done(_Message):
    TYPE = MsgType.DONE
    STRUCT = struct.Struct("<BBH")
    axis: int
    seq: int
    steps: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.seq, self.steps)


@dataclass(frozen=True)
class Aborted(_Message):
    """A segment was cut short, and by exactly how much.

    ``steps_done`` is what makes an abort exact on this backend where it could
    not be on lgpio: the microcontroller counts its own ISR ticks, so it knows
    what the motor received even when an endstop cuts the segment mid-flight.
    """

    TYPE = MsgType.ABORTED
    STRUCT = struct.Struct("<BBHB")
    axis: int
    seq: int
    steps_done: int
    cause: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.axis, self.seq, self.steps_done, self.cause)

    @property
    def cause_name(self) -> str:
        try:
            return AbortCause(self.cause).name.lower()
        except ValueError:
            return f"cause {self.cause}"


@dataclass(frozen=True)
class Event(_Message):
    TYPE = MsgType.EVENT
    STRUCT = struct.Struct("<BB")
    kind: int
    state: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.kind, self.state)


@dataclass(frozen=True)
class Pong(_Message):
    TYPE = MsgType.PONG
    STRUCT = struct.Struct("<B")
    nonce: int

    def _pack(self) -> bytes:
        return self.STRUCT.pack(self.nonce)


@dataclass(frozen=True)
class AxisState:
    queue_len: int
    flags: int
    endstop: int


@dataclass(frozen=True)
class State:
    """Variable-length: one triple per axis, then the E-stop level."""

    TYPE: ClassVar[MsgType] = MsgType.STATE
    axes: tuple[AxisState, ...]
    estop: int

    def encode(self) -> bytes:
        payload = bytes((len(self.axes),))
        for axis in self.axes:
            payload += bytes((axis.queue_len, axis.flags, axis.endstop))
        payload += bytes((self.estop,))
        return encode_frame(self.TYPE, payload)

    @classmethod
    def decode(cls, payload: bytes) -> "State":
        if not payload:
            raise ProtocolError("STATE payload is empty")
        count = payload[0]
        if len(payload) != 1 + 3 * count + 1:
            raise ProtocolError(
                f"STATE claims {count} axes but carries {len(payload)} payload bytes"
            )
        axes = tuple(
            AxisState(*payload[1 + 3 * i : 4 + 3 * i]) for i in range(count)
        )
        return cls(axes=axes, estop=payload[-1])


#: Every decodable message, by type byte.
DECODERS: dict[int, type] = {
    MsgType.HELLO: Hello,
    MsgType.SEG: Seg,
    MsgType.ABORT: Abort,
    MsgType.ENABLE: Enable,
    MsgType.CLEAR_FAULT: ClearFault,
    MsgType.PING: Ping,
    MsgType.STATUS: Status,
    MsgType.IDENT: Ident,
    MsgType.ACK: Ack,
    MsgType.NACK: Nack,
    MsgType.DONE: Done,
    MsgType.ABORTED: Aborted,
    MsgType.EVENT: Event,
    MsgType.STATE: State,
    MsgType.PONG: Pong,
}


def decode_message(frame: Frame):
    """Turn a validated frame into its message object.

    Raises :class:`ProtocolError` for an unknown type or a payload that does
    not fit — both of which are NACK-worthy on the firmware side and
    log-and-drop on the host side.
    """
    decoder = DECODERS.get(frame.type)
    if decoder is None:
        raise ProtocolError(f"unknown message type 0x{frame.type:02X}")
    return decoder.decode(frame.payload)


def next_seq(seq: int) -> int:
    """Advance a per-axis sequence number, skipping the reserved SEQ_NONE."""
    return 0 if seq >= SEQ_MAX else seq + 1
