"""A Python model of the Arduino sketch that fails the way the real one does.

Same discipline as `fake_lgpio.py`: a double that always cooperates proves
nothing. This one speaks the **real byte protocol** — the host talks to it
through the same framing, CRC and flow control it will use on the bench — and it
enforces the rules the firmware enforces:

* a bounded per-axis ring, and a NACK rather than a silent drop when it is full;
* sequence numbers it has already accepted are re-ACKed, never executed twice;
* direction changes cost `DIR_SETUP_US` of drained, stopped time;
* endstops are checked *per step*, so a segment gets cut mid-flight and reports
  the exact count that went out — the property this whole backend exists for;
* a watchdog that lets the queue drain and then drops the enables;
* pulses go into a `Mechanics` model, so the motor can stall and lose steps.

It runs on `VirtualClock` under the existing harness: no threads, no wall-clock
sleeps, identical ordering every run.

The one thing it deliberately does *not* model is instruction timing on an
ATmega. Whether the AVR can keep up is a hardware question that only
`selftest --serial` on real silicon can answer (PLAN-ARDUINO §10, milestone 6).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from orbitaly.motion import serial_protocol as sp
from orbitaly.motion.segment import Segment

#: Mirrors ORB_QUEUE_DEPTH in firmware/orbitaly_rotator/queue.h.
QUEUE_DEPTH = 8
RECENT_DEPTH = 8

FW_VERSION = (1, 0, 0)
DIR_SETUP_US = 20
PULSE_WIDTH_US = 5
ENDSTOP_DEBOUNCE_MS = 5

#: No valid frame for this long and the firmware stops trusting the host. It
#: drains rather than aborting: every plan terminates at rest, so draining
#: leaves the motor stopped at a *known* position, which an abort would not.
FW_WATCHDOG_S = 5.0
IDLE_DISABLE_S = 2.0


@dataclass
class _InFlight:
    seq: int
    direction: int
    steps: int
    period_us: int
    started_at: float
    #: when the host handed it over — a segment starts as soon as the axis is
    #: free from *this* moment, not whenever advance() next happens to be
    #: called, or a coarse test tick would silently delay the motor
    queued_at: float = 0.0
    steps_done: int = 0

    @property
    def period_s(self) -> float:
        return self.period_us / 1e6

    @property
    def ends_at(self) -> float:
        return self.started_at + self.steps * self.period_s

    def steps_by(self, now: float) -> int:
        if now <= self.started_at:
            return 0
        return min(self.steps, int((now - self.started_at) / self.period_s))


@dataclass
class _Axis:
    index: int
    mechanics: object | None = None
    queue: deque = field(default_factory=deque)
    recent: deque = field(default_factory=lambda: deque(maxlen=RECENT_DEPTH))
    current: _InFlight | None = None
    direction: int = 0          # the DIR pin's current state
    enabled: bool = False
    endstop: int = 0
    free_at: float = 0.0        # when the next segment may start (direction setup)
    pulses: int = 0

    def queued_seqs(self) -> list[int]:
        return [item.seq for item in self.queue]


class FakeFirmware:
    """The sketch, in Python, on a virtual clock."""

    def __init__(self, *, mechanics: dict | None = None, clock=None, queue_depth: int = QUEUE_DEPTH):
        from orbitaly.motion.clock import VirtualClock

        self.clock = clock or VirtualClock()
        self.queue_depth = queue_depth
        mechanics = mechanics or {}
        self.axes = [
            _Axis(index=sp.AXIS_AZ, mechanics=mechanics.get("az")),
            _Axis(index=sp.AXIS_EL, mechanics=mechanics.get("el")),
        ]
        self.pins = (2, 3, 4, 5, 6, 7, 8, 9)
        self.estop = 0
        self.ready = False           # set by a version-matched HELLO
        self.out = bytearray()
        self.decoder = sp.FrameDecoder()
        self.last_host_frame = self.clock.monotonic()
        self.watchdog_tripped = False
        self.booted_at = self.clock.monotonic()
        #: counters a test can assert on
        self.nacks: list[sp.Nack] = []
        self.duplicates_rejected = 0
        self.frames_in = 0
        self.bad_frames = 0
        self.idents_sent = 0
        self._last_ident = -1e9
        self._emit_ident()

    # -- transport surface --------------------------------------------------

    def on_host_bytes(self, data: bytes) -> None:
        """Bytes arriving from the host. Parses and answers immediately."""
        for frame in self.decoder.feed(data):
            self.frames_in += 1
            self.last_host_frame = self.clock.monotonic()
            self.watchdog_tripped = False
            try:
                message = sp.decode_message(frame)
            except sp.ProtocolError:
                self.bad_frames += 1
                self._send(sp.Nack(sp.AXIS_ALL, sp.SEQ_NONE, sp.NackReason.BAD_TYPE))
                continue
            self._handle(message)

    def take_output(self) -> bytes:
        data = bytes(self.out)
        self.out.clear()
        return data

    # -- time ---------------------------------------------------------------

    def advance(self, now: float) -> None:
        """Run the step generators forward to ``now``."""
        self._reannounce(now)
        for axis in self.axes:
            self._run_axis(axis, now)
        self._watchdog(now)

    def _reannounce(self, now: float) -> None:
        """Repeat the boot IDENT until the host says something.

        PLAN-ARDUINO §14 left this open; it is answered yes. Opening the port
        resets the board, so the host is often still settling its own end when
        the first IDENT goes out — announcing once would lose the race and the
        host would wait out its whole connect timeout for no reason.
        """
        if self.ready or now - self._last_ident < 1.0:
            return
        self._emit_ident()

    # -- message handling ---------------------------------------------------

    def _handle(self, message) -> None:
        if isinstance(message, sp.Hello):
            if message.version != sp.PROTOCOL_VERSION:
                self._send(sp.Nack(sp.AXIS_ALL, sp.SEQ_NONE, sp.NackReason.BAD_VERSION))
                return
            self.ready = True
            self._emit_ident()
            return

        if not self.ready:
            # Anything before a version-matched HELLO is refused. A host that
            # skipped the handshake has not proved it speaks this protocol, and
            # guessing is how a stale flash moves an antenna.
            self._send(sp.Nack(sp.AXIS_ALL, sp.SEQ_NONE, sp.NackReason.NOT_READY))
            return

        if isinstance(message, sp.Ping):
            self._send(sp.Pong(message.nonce))
        elif isinstance(message, sp.Status):
            self._emit_state()
        elif isinstance(message, sp.Seg):
            self._on_seg(message)
        elif isinstance(message, sp.Abort):
            self._on_abort(message)
        elif isinstance(message, sp.Enable):
            self._on_enable(message)
        elif isinstance(message, sp.ClearFault):
            self._on_clear_fault(message)

    def _axis(self, index: int) -> _Axis | None:
        for axis in self.axes:
            if axis.index == index:
                return axis
        return None

    def _on_seg(self, message: sp.Seg) -> None:
        axis = self._axis(message.axis)
        if axis is None:
            self._nack(message.axis, message.seq, sp.NackReason.BAD_AXIS)
            return
        if message.seq in axis.queued_seqs() or message.seq in axis.recent:
            # A re-send after a lost ACK. Re-ACK it; executing it twice would
            # move the antenna by a segment nobody asked for.
            self.duplicates_rejected += 1
            self._send(sp.Ack(axis.index, message.seq, self._free(axis)))
            return
        if message.direction == self._inhibited_dir(axis):
            self._nack(axis.index, message.seq, sp.NackReason.INHIBITED)
            return
        if len(axis.queue) + (1 if axis.current else 0) >= self.queue_depth:
            self._nack(axis.index, message.seq, sp.NackReason.QUEUE_FULL)
            return
        axis.queue.append(
            _InFlight(
                seq=message.seq,
                direction=message.direction,
                steps=message.steps,
                period_us=message.period_us,
                started_at=0.0,
                queued_at=self.clock.monotonic(),
            )
        )
        self._send(sp.Ack(axis.index, message.seq, self._free(axis)))

    def _on_abort(self, message: sp.Abort) -> None:
        targets = self.axes if message.axis == sp.AXIS_ALL else [self._axis(message.axis)]
        for axis in targets:
            if axis is not None:
                self._abort_axis(axis, sp.AbortCause.HOST)

    def _on_enable(self, message: sp.Enable) -> None:
        targets = self.axes if message.axis == sp.AXIS_ALL else [self._axis(message.axis)]
        for axis in targets:
            if axis is not None:
                axis.enabled = bool(message.on)

    def _on_clear_fault(self, message: sp.ClearFault) -> None:
        targets = self.axes if message.axis == sp.AXIS_ALL else [self._axis(message.axis)]
        for axis in targets:
            if axis is None:
                continue
            if self._endstop_level(axis) or self.estop:
                # Mirrors the HTTP 409: refuse while the condition is still there.
                self._nack(axis.index, sp.SEQ_NONE, sp.NackReason.INHIBITED)

    # -- step generation ----------------------------------------------------

    def _run_axis(self, axis: _Axis, now: float) -> None:
        guard = 0
        while True:
            guard += 1
            assert guard < 10_000, "fake firmware step generator failed to converge"
            if axis.current is None:
                if not axis.queue or self.estop:
                    return
                nxt = axis.queue[0]
                start = max(nxt.queued_at, axis.free_at)
                if nxt.direction != axis.direction:
                    # Drain, flip DIR, wait out the driver's setup time. On the
                    # real board this is why the host does not have to time a
                    # 20 us window over a 1-2 ms USB link.
                    start += DIR_SETUP_US / 1e6
                    axis.direction = nxt.direction
                if start > now:
                    axis.free_at = start
                    return
                nxt.started_at = start
                axis.current = axis.queue.popleft()
                continue

            current = axis.current
            reached = current.steps_by(now)
            if reached > current.steps_done:
                cut = self._emit_steps(axis, current, reached - current.steps_done)
                current.steps_done += cut
                if cut and self._endstop_level(axis) and current.steps_done < current.steps:
                    # The switch closed part way through. This is the case lgpio
                    # cannot report and this backend can.
                    self._trip_endstop(axis, current)
                    return
            if current.steps_done >= current.steps:
                axis.current = None
                axis.recent.append(current.seq)
                axis.free_at = current.ends_at
                self._send(sp.Done(axis.index, current.seq, current.steps_done))
                if self._endstop_level(axis):
                    self._trip_endstop(axis, None)
                    return
                continue
            return

    def _emit_steps(self, axis: _Axis, current: _InFlight, count: int) -> int:
        """Push ``count`` pulses into the mechanism, stopping at the switch.

        Applied one step at a time when a switch is fitted, because the firmware
        checks the endstop inside the step ISR — modelling it per segment would
        make every abort look exact by construction, which is the very thing
        under test.
        """
        axis.pulses += count
        if axis.mechanics is None:
            return count
        if not axis.mechanics.has_endstop():
            axis.mechanics.apply(
                Segment(direction=current.direction, steps=count, period_us=current.period_us)
            )
            return count
        for done in range(count):
            axis.mechanics.apply(
                Segment(direction=current.direction, steps=1, period_us=current.period_us)
            )
            if axis.mechanics.endstop_triggered():
                return done + 1
        return count

    def _trip_endstop(self, axis: _Axis, current: _InFlight | None) -> None:
        # The EVENT edge is emitted by _endstop_level, which is the single place
        # that owns `axis.endstop`. Deciding the edge here as well would mean two
        # readers racing on one variable — and the loser silently swallowing the
        # only notification the host ever gets.
        axis.endstop = 1
        if current is not None:
            axis.current = None
            axis.recent.append(current.seq)
            self._send(
                sp.Aborted(axis.index, current.seq, current.steps_done, sp.AbortCause.ENDSTOP)
            )
        self._drop_queue(axis)

    def _abort_axis(self, axis: _Axis, cause: int) -> None:
        current = axis.current
        if current is not None:
            axis.current = None
            axis.recent.append(current.seq)
            self._send(sp.Aborted(axis.index, current.seq, current.steps_done, cause))
        else:
            self._send(sp.Aborted(axis.index, sp.SEQ_NONE, 0, cause))
        self._drop_queue(axis)

    def _drop_queue(self, axis: _Axis) -> None:
        while axis.queue:
            axis.recent.append(axis.queue.popleft().seq)

    def _inhibited_dir(self, axis: _Axis) -> int:
        """Which direction is refused right now: into a closed switch, or none.

        Derived from the live switch level, never latched. The inhibit exists to
        stop the motor driving further into a switch that is *currently* closed;
        once it opens, the mechanical reason is gone. Latching it here would
        break homing, whose whole procedure is trip → back off → re-approach —
        the re-approach is motion toward a switch that has just opened.

        Latching the *fault* is the host's job, and the host does it: the axis
        supervisor keeps a fault until an operator clears it, while still
        allowing a jog away. Two latches would mean two things to clear.
        """
        return -1 if self._endstop_level(axis) else 0

    def _endstop_level(self, axis: _Axis) -> int:
        """Read the switch, reporting both edges. The only writer of ``endstop``."""
        if axis.mechanics is None or not axis.mechanics.has_endstop():
            return axis.endstop
        level = 1 if axis.mechanics.endstop_triggered() else 0
        if level != axis.endstop:
            axis.endstop = level
            kind = sp.EventKind.ENDSTOP_AZ if axis.index == sp.AXIS_AZ else sp.EventKind.ENDSTOP_EL
            self._send(sp.Event(kind, level))
        return level

    # -- interlocks ---------------------------------------------------------

    def set_estop(self, pressed: bool) -> None:
        """Operator hit (or released) the mushroom."""
        level = 1 if pressed else 0
        if level == self.estop:
            return
        self.estop = level
        self._send(sp.Event(sp.EventKind.ESTOP, level))
        if pressed:
            for axis in self.axes:
                self._abort_axis(axis, sp.AbortCause.ESTOP)
                axis.enabled = False  # enables drop: an unpowered stepper holds nothing

    def reboot(self) -> None:
        """Brownout, watchdog, replugged cable — the queue is gone.

        The unsolicited IDENT is the only signal the host gets, and it is the
        one event that invalidates every position it believes in.
        """
        for axis in self.axes:
            axis.queue.clear()
            axis.recent.clear()
            axis.current = None
            axis.direction = 0
            axis.enabled = False
        self.ready = False
        self.decoder = sp.FrameDecoder()
        self.booted_at = self.clock.monotonic()
        self._emit_ident()

    def _watchdog(self, now: float) -> None:
        if not self.ready:
            return
        if now - self.last_host_frame <= FW_WATCHDOG_S:
            return
        # Not an abort: the queue is guaranteed to end at rest, so letting it
        # drain loses nothing and gains a known stopping position.
        self.watchdog_tripped = True
        if all(axis.current is None and not axis.queue for axis in self.axes):
            if now - self.last_host_frame > FW_WATCHDOG_S + IDLE_DISABLE_S:
                for axis in self.axes:
                    axis.enabled = False

    # -- outbound -----------------------------------------------------------

    def _free(self, axis: _Axis) -> int:
        return max(0, self.queue_depth - len(axis.queue) - (1 if axis.current else 0))

    def _nack(self, axis_index: int, seq: int, reason: int) -> None:
        message = sp.Nack(axis_index, seq, reason)
        self.nacks.append(message)
        self._send(message)

    def _send(self, message) -> None:
        self.out.extend(message.encode())

    def _emit_ident(self) -> None:
        self.idents_sent += 1
        self._last_ident = self.clock.monotonic()
        caps = (
            sp.Caps.EXACT_ABORT
            | sp.Caps.ENDSTOP_NC
            | sp.Caps.ENABLE_ACTIVE_LOW
            | sp.Caps.ESTOP_FITTED
        )
        for axis in self.axes:
            if axis.mechanics is not None and axis.mechanics.has_endstop():
                caps |= sp.Caps.AZ_ENDSTOP if axis.index == sp.AXIS_AZ else sp.Caps.EL_ENDSTOP
        self._send(
            sp.Ident(
                protocol=sp.PROTOCOL_VERSION,
                fw_major=FW_VERSION[0],
                fw_minor=FW_VERSION[1],
                fw_patch=FW_VERSION[2],
                queue_depth=self.queue_depth,
                axis_count=len(self.axes),
                caps=caps,
                dir_setup_us=DIR_SETUP_US,
                pulse_width_us=PULSE_WIDTH_US,
                endstop_debounce_ms=ENDSTOP_DEBOUNCE_MS,
                pins=self.pins,
            )
        )

    def _emit_state(self) -> None:
        axes = []
        for axis in self.axes:
            flags = 0
            if self._inhibited_dir(axis) < 0:
                flags |= sp.AxisFlags.INHIBIT_NEG
            if axis.enabled:
                flags |= sp.AxisFlags.ENABLED
            queued = len(axis.queue) + (1 if axis.current else 0)
            axes.append(sp.AxisState(queued, flags, self._endstop_level(axis)))
        self._send(sp.State(axes=tuple(axes), estop=self.estop))


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------

class FirmwareTransport:
    """Host-side transport wired directly to a :class:`FakeFirmware`.

    Writes are delivered synchronously, so a request that the firmware can
    answer without executing pulses (HELLO, PING, ABORT) is answered before
    ``write`` returns. That is what lets the whole stack run on a virtual clock
    with no threads: nothing is ever waiting on a real timeout.
    """

    port = "fake://firmware"

    def __init__(self, firmware: FakeFirmware):
        self.firmware = firmware
        self.closed = False

    def write(self, data: bytes) -> None:
        if self.closed:
            from orbitaly.motion.serial_driver import TransportError

            raise TransportError("write to a closed port")
        self.firmware.on_host_bytes(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        if self.closed:
            from orbitaly.motion.serial_driver import TransportError

            raise TransportError("read from a closed port")
        return self.firmware.take_output()[:max_bytes]

    def close(self) -> None:
        self.closed = True


class FaultyTransport:
    """Wraps a transport and breaks it on demand.

    Every mode here maps to a failure the plan decided in advance (§7): bit
    flips and dropped bytes to the CRC/resync path, duplicated frames to the
    idempotent-resend rule, and an unplugged cable to the latched backend fault.
    """

    port = "fake://faulty"

    def __init__(self, inner):
        self.inner = inner
        self.flip_bits_in_next = 0     # corrupt N of the next inbound reads
        self.drop_bytes_in_next = 0
        self.duplicate_next_write = False
        self.unplugged = False
        self.corruptions = 0
        self.drops = 0

    def write(self, data: bytes) -> None:
        if self.unplugged:
            from orbitaly.motion.serial_driver import TransportError

            raise TransportError("cable unplugged")
        self.inner.write(data)
        if self.duplicate_next_write:
            # A re-send the host did not intend: the firmware must recognise the
            # sequence number and not move twice.
            self.duplicate_next_write = False
            self.inner.write(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        if self.unplugged:
            from orbitaly.motion.serial_driver import TransportError

            raise TransportError("cable unplugged")
        data = bytearray(self.inner.read(max_bytes))
        if data and self.flip_bits_in_next > 0:
            self.flip_bits_in_next -= 1
            index = len(data) // 2
            data[index] ^= 0x01
            self.corruptions += 1
        if data and self.drop_bytes_in_next > 0:
            self.drop_bytes_in_next -= 1
            del data[len(data) // 2]
            self.drops += 1
        return bytes(data)

    def close(self) -> None:
        self.inner.close()
