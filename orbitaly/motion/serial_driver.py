"""Serial motion backend — the reference deployment: Pi ─USB─▶ Arduino ─step/dir─▶ drivers.

The Pi does not drive the stepper drivers from its own GPIO. It plans, and an
Arduino executes: exactly N pulses at one rate, and an honest report of how many
went out. That division is the whole design (PLAN-ARDUINO §2–3) — all trajectory
intelligence stays here, and the firmware is deliberately too stupid to have an
opinion about where the antenna should point.

The segment boundary the motion stack was already split at turns out to suit a
serial link better than it suited lgpio:

* A segment is already a compact, self-contained wire message.
* **Aborts become exact.** lgpio cannot say how many pulses of a cut ``tx_pulse``
  train reached the motor, so an abort loses the position reference and forces
  re-homing. A microcontroller counts its own ISR ticks and reports
  ``ABORTED{steps_done}``, so ``homed`` survives an endstop hit.
* **Link loss is safe for free.** Every plan terminates at rest (invariant 1), so
  a yanked USB cable leaves the firmware draining a queue that ends stopped, at a
  position the host can still account for.

Where this deviates from the plan: there is **no reader thread**. The axis
supervisors already tick at ~500 Hz and call :meth:`SerialAxisDriver.pump`, so
servicing the link from there polls it far faster than the 1 Hz tracker needs,
with no extra thread to reason about and no locking between a reader and the
supervisors. Firmware-side reflexes still happen at interrupt latency; only the
host's *bookkeeping* waits up to one tick, and by then the motion has already
stopped (PLAN-ARDUINO §8).
"""
from __future__ import annotations

import logging
import threading
from collections import OrderedDict, deque
from pathlib import Path
from typing import Callable, Protocol

from ..config import AxisConfig, RotatorConfig
from .backend import Backend
from .driver import AbortResult, AxisDriver
from .segment import Segment
from . import serial_protocol as sp

log = logging.getLogger(__name__)

#: Must match ORB_QUEUE_DEPTH in firmware/orbitaly_rotator/queue.h. Asserted by
#: tests/test_serial_protocol.py; used as the flow-control ceiling until a real
#: IDENT says otherwise.
DEFAULT_QUEUE_DEPTH = 8

#: Wire speed. The throughput argument (PLAN-ARDUINO §4): worst case is both axes
#: at maximum segment rate, ~100 frames/s of 14 bytes ≈ 1.5 kB/s each way against
#: ~11 kB/s available. An order of magnitude of headroom, and a divisor the AVR's
#: clock hits with little error.
DEFAULT_BAUD = 115200

#: A u16 on the wire. Nothing the planner emits comes close (a segment is capped
#: at ``segment_ms`` of motion), but ``enqueue`` splits rather than trusting that.
MAX_SEGMENT_STEPS = 0xFFFF

_AXIS_INDEX = {"az": sp.AXIS_AZ, "el": sp.AXIS_EL}


class Transport(Protocol):
    """A byte pipe. Non-blocking reads; a write either happens or raises."""

    def write(self, data: bytes) -> None: ...

    def read(self, max_bytes: int = 4096) -> bytes: ...

    def close(self) -> None: ...


class TransportError(OSError):
    """The link failed. Always latched as a fault — never retried silently."""


# --------------------------------------------------------------------------
# Real serial port
# --------------------------------------------------------------------------

def probe_serial_ports() -> list[str]:
    """Candidate device paths, best first.

    ``/dev/serial/by-id`` first because it survives a replug and a second USB
    device appearing: ``/dev/ttyACM0`` is whichever board enumerated first this
    boot, which is exactly the sort of thing that silently repoints a rotator at
    the wrong hardware.
    """
    by_id = Path("/dev/serial/by-id")
    ports = [str(p) for p in sorted(by_id.glob("*"))] if by_id.is_dir() else []
    for pattern in ("ttyACM*", "ttyUSB*"):
        ports += [str(p) for p in sorted(Path("/dev").glob(pattern))]
    return ports


class SerialTransport:
    """pyserial, wrapped down to :class:`Transport`."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        try:
            import serial  # imported lazily: an optional extra, absent on dev machines
        except ImportError as exc:  # pragma: no cover - depends on the install
            raise TransportError(
                "pyserial is not installed — pip install 'orbitaly[serial]'"
            ) from exc
        self.port = port
        try:
            # timeout=0 makes read() return whatever is buffered instead of
            # blocking, which is what lets the axis tick poll the link.
            self._serial = serial.Serial(port, baud, timeout=0, write_timeout=2.0)
        except Exception as exc:  # noqa: BLE001 - pyserial raises several types
            raise TransportError(f"cannot open {port}: {exc}") from exc

    def write(self, data: bytes) -> None:
        try:
            self._serial.write(data)
        except Exception as exc:  # noqa: BLE001
            raise TransportError(f"write to {self.port} failed: {exc}") from exc

    def read(self, max_bytes: int = 4096) -> bytes:
        try:
            waiting = self._serial.in_waiting
            if not waiting:
                return b""
            return self._serial.read(min(waiting, max_bytes))
        except Exception as exc:  # noqa: BLE001
            raise TransportError(f"read from {self.port} failed: {exc}") from exc

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:  # noqa: BLE001 - closing a dead port must not mask the real fault
            pass


# --------------------------------------------------------------------------
# Link
# --------------------------------------------------------------------------

class SerialLink:
    """Messages over a transport. Knows framing; knows nothing about motion."""

    def __init__(self, transport: Transport, *, clock=None):
        from .clock import RealClock

        self.transport = transport
        self.clock = clock or RealClock()
        self.decoder = sp.FrameDecoder()
        self.handler: Callable[[object], None] | None = None
        self.tx_messages = 0
        self.rx_messages = 0
        self.decode_errors = 0
        self.last_tx = self.clock.monotonic()
        self.last_rx = self.clock.monotonic()

    def send(self, message) -> None:
        self.transport.write(message.encode())
        self.tx_messages += 1
        self.last_tx = self.clock.monotonic()

    def service(self) -> int:
        """Read whatever has arrived and dispatch it. Returns messages handled."""
        data = self.transport.read()
        if not data:
            return 0
        handled = 0
        for frame in self.decoder.feed(data):
            self.last_rx = self.clock.monotonic()
            try:
                message = sp.decode_message(frame)
            except sp.ProtocolError as exc:
                # A frame that passed CRC but does not parse means the two sides
                # disagree about the protocol — worth saying loudly, not worth
                # dying over mid-pass.
                self.decode_errors += 1
                log.warning("undecodable frame from firmware: %s", exc)
                continue
            self.rx_messages += 1
            handled += 1
            if self.handler is not None:
                self.handler(message)
        return handled

    @property
    def stats(self) -> dict:
        return {
            "tx_messages": self.tx_messages,
            "rx_messages": self.rx_messages,
            "crc_errors": self.decoder.crc_errors,
            "resyncs": self.decoder.resyncs,
            "decode_errors": self.decode_errors,
        }


# --------------------------------------------------------------------------
# Axis driver
# --------------------------------------------------------------------------

class SerialAxisDriver(AxisDriver):
    """One axis, executed by the firmware.

    A sibling of :class:`~orbitaly.motion.driver.BufferedAxisDriver`, not a
    subclass. That class *infers* completion from the clock because lgpio gives
    no completion signal; here the firmware says ``DONE{seq, steps}``. Inferring
    what hardware will literally tell us would be a step backwards, so the
    bookkeeping is driven by reports and the public surface is unchanged.
    """

    #: Motion committed to the firmware at once. Deeper would mean a retarget
    #: waits behind stale segments; the whole point of planning on the Pi is
    #: that the Pi can change its mind. Same argument as the lgpio MAX_IN_FLIGHT.
    COMMIT_S = 0.10

    #: How long past a segment's expected finish to wait before suspecting its
    #: report was lost. Generous: being early here costs a needless STATUS round
    #: trip, being late costs nothing but a stalled tick.
    REPORT_GRACE_S = 0.5

    def __init__(self, backend: "SerialBackend", index: int, config: AxisConfig, name: str):
        self._backend = backend
        self._index = index
        self._name = name
        self._config = config
        self._lock = backend.lock  # one lock for the backend and both axes
        self._queue: deque[Segment] = deque()
        self._outstanding: "OrderedDict[int, Segment]" = OrderedDict()
        self._sent_at: dict[int, float] = {}
        self._status_asked_at: float | None = None
        self._seq = 0
        self._consumed = 0
        self._committed = 0
        self._last_direction = 0
        self._enabled = False
        self._endstop_level = 0
        self._has_endstop = False
        #: Direction the firmware is currently refusing. Mirrors its reflex so we
        #: stop shoving segments at a board that is saying no — one NACK teaches
        #: us, and the supervisor's own limit handling does the rest.
        self._inhibit_dir = 0
        self._endstop_callback: Callable[[], None] | None = None
        self._abort_report: sp.Aborted | None = None
        #: set when the firmware rebooted or the link failed: segments that were
        #: accepted but never reported are unaccounted, so the next abort must
        #: not claim to be exact.
        self._position_lost = False

    # -- AxisDriver ---------------------------------------------------------

    def enqueue(self, segment: Segment) -> None:
        with self._lock:
            for piece in _split(segment):
                self._queue.append(piece)
                self._committed += piece.delta

    def consumed_steps(self) -> int:
        with self._lock:
            return self._consumed

    def committed_steps(self) -> int:
        with self._lock:
            return self._committed

    def pending_s(self) -> float:
        """Seconds of motion queued here or accepted by the firmware.

        An estimate, for pacing only — never for position. Segments already in
        flight are counted whole, so this over-reports slightly, which is the
        safe direction for a value used to decide whether to queue more.
        """
        with self._lock:
            return sum(s.duration_s for s in self._queue) + self._inflight_s()

    def busy(self) -> bool:
        with self._lock:
            return bool(self._queue or self._outstanding)

    def abort(self) -> AbortResult:
        with self._lock:
            for segment in self._queue:
                self._committed -= segment.delta
            self._queue.clear()
            self._abort_report = None
            lost = self._position_lost
        try:
            self._backend.send(sp.Abort(self._index))
        except TransportError:
            with self._lock:
                self._committed = self._consumed
                return AbortResult(self._consumed, exact=False)

        confirmed = self._backend.wait_for(
            lambda: self._abort_report is not None, self._backend.ABORT_TIMEOUT_S
        )
        with self._lock:
            self._committed = self._consumed
            # Exact unless something already cost us the reference: the firmware
            # counts its own ISR ticks, so it knows what the motor received even
            # when an endstop cut the segment mid-flight.
            return AbortResult(self._consumed, exact=bool(confirmed) and not lost)

    def committed_speed_sps(self) -> float:
        with self._lock:
            segment = self._last_committed_segment()
            return segment.speed_sps if segment else 0.0

    def committed_direction(self) -> int:
        with self._lock:
            segment = self._last_committed_segment()
            return segment.direction if segment else 0

    def last_direction(self) -> int:
        with self._lock:
            segment = self._last_committed_segment()
            return segment.direction if segment else self._last_direction

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled
        self._backend.send_safe(sp.Enable(self._index, 1 if enabled else 0))

    def has_endstop(self) -> bool:
        return self._has_endstop

    def endstop_triggered(self) -> bool:
        with self._lock:
            return bool(self._endstop_level)

    def arm_endstop(self, callback: Callable[[], None] | None) -> None:
        self._endstop_callback = callback

    def close(self) -> None:
        pass  # the backend owns the port

    # -- pump ---------------------------------------------------------------

    def pump(self) -> None:
        """Service the link, then hand over whatever the firmware has room for.

        This is what the axis supervisor calls every tick, and it is also the
        only thing that polls the link — see the module docstring.
        """
        self._backend.service()
        with self._lock:
            depth = self._backend.queue_depth
            while self._queue and len(self._outstanding) < depth:
                if self._inflight_s() >= self.COMMIT_S:
                    break
                segment = self._queue[0]
                if segment.direction == self._inhibit_dir:
                    break  # the firmware would refuse it; wait for the recovery
                seq = self._seq
                message = sp.Seg(
                    axis=self._index,
                    seq=seq,
                    direction=segment.direction,
                    steps=segment.steps,
                    period_us=segment.period_us,
                )
                try:
                    self._backend.send(message)
                except TransportError:
                    break  # the backend has latched the fault; stop feeding
                self._queue.popleft()
                self._outstanding[seq] = segment
                self._sent_at[seq] = self._backend.clock.monotonic()
                self._seq = sp.next_seq(seq)
        self._check_for_lost_reports()

    def _check_for_lost_reports(self) -> None:
        """Ask the firmware where it is, if a report looks like it went missing.

        The ordering rule in :meth:`_retire_older_than` recovers a lost report
        as soon as any later one arrives — which covers everything except the
        *last* segment of a move, where nothing follows to imply it. That case
        would leave the axis wedged at ``busy`` forever, so once a segment is
        overdue we ask outright. Time decides only when to *ask*; the firmware
        still decides what happened.
        """
        with self._lock:
            if not self._outstanding or self._status_asked_at is not None:
                return
            first = next(iter(self._outstanding))
            expected_end = self._sent_at.get(first, 0.0) + self._inflight_s()
            now = self._backend.clock.monotonic()
            if now < expected_end + self.REPORT_GRACE_S:
                return
            self._status_asked_at = now
        log.debug("%s: a segment report is overdue, asking the firmware", self._name)
        self._backend.send_safe(sp.Status())

    def on_state(self, axis_state: sp.AxisState) -> None:
        """A STATE snapshot: if the firmware holds nothing, nothing is pending.

        The firmware saying its queue is empty is a report about *our* segments,
        so retiring them here is still position from what hardware told us — not
        from how long we have been waiting.
        """
        with self._lock:
            asked_at = self._status_asked_at
            self._status_asked_at = None
            if asked_at is None or axis_state.queue_len:
                return
            for seq in list(self._outstanding):
                if self._sent_at.get(seq, 0.0) > asked_at:
                    continue  # handed over after we asked; the answer says nothing about it
                segment = self._outstanding.pop(seq)
                self._sent_at.pop(seq, None)
                self._consumed += segment.delta
                self._last_direction = segment.direction
                log.info(
                    "%s: seq %d had no report but the firmware queue is empty — "
                    "counting it complete",
                    self._name,
                    seq,
                )

    def _inflight_s(self) -> float:
        return sum(s.duration_s for s in self._outstanding.values())

    def _last_committed_segment(self) -> Segment | None:
        if self._queue:
            return self._queue[-1]
        if self._outstanding:
            return next(reversed(self._outstanding.values()))
        return None

    # -- report handling ----------------------------------------------------

    def on_ack(self, message: sp.Ack) -> None:
        # `slots_free` is diagnostic only. Flow control gates on the number of
        # segments outstanding here, which cannot race against a NACK-and-resend
        # the way arithmetic on a reported credit can (PLAN-ARDUINO §14).
        if message.seq not in self._outstanding:
            log.debug("%s: ACK for unknown seq %d", self._name, message.seq)

    def on_nack(self, message: sp.Nack) -> None:
        with self._lock:
            segment = self._outstanding.pop(message.seq, None)
            self._sent_at.pop(message.seq, None)
        reason = message.reason
        if segment is None:
            log.warning("%s: NACK (%s) for unknown seq %d", self._name, message.reason_name, message.seq)
            return
        if reason == sp.NackReason.QUEUE_FULL:
            # We were wrong about the credit, not about the motion. Put it back
            # at the head so ordering — and therefore position — is preserved.
            with self._lock:
                self._queue.appendleft(segment)
            return
        if reason == sp.NackReason.INHIBITED:
            # The firmware is refusing motion in this direction (endstop). Drop
            # what is queued; the supervisor's endstop path owns the recovery.
            with self._lock:
                first = self._inhibit_dir == 0
                self._inhibit_dir = segment.direction
                for queued in self._queue:
                    self._committed -= queued.delta
                self._queue.clear()
                self._committed -= segment.delta
            # Only the first one is news. A whole buffer's worth gets refused in
            # one burst, and a wall of identical warnings buries the endstop
            # fault that actually explains it.
            (log.warning if first else log.debug)(
                "%s: firmware refused a segment: %s", self._name, message.reason_name
            )
            return
        # Anything else means the two sides disagree about the protocol or the
        # wiring, which is not something to paper over while pointing an antenna.
        with self._lock:
            self._committed -= segment.delta
        self._backend.fail(f"{self._name}: firmware rejected a segment ({message.reason_name})")

    def on_done(self, message: sp.Done) -> None:
        with self._lock:
            self._retire_older_than(message.seq)
            self._retire(message.seq, message.steps)

    def _retire_older_than(self, seq: int) -> None:
        """Close out segments the firmware has evidently already finished.

        Reports can be lost — a flipped bit costs a whole frame — and a segment
        whose DONE never arrives would otherwise sit in ``_outstanding`` forever,
        leaving the axis permanently ``busy`` and the queue permanently full.

        The recovery does not guess. Segments execute strictly in order, and the
        firmware only starts one after finishing the last; had it cut one short
        it would have said ABORTED. So a report about a *later* sequence number
        is itself the firmware telling us the earlier ones completed in full.
        This is still position from reported counts — just from a report that
        arrived about a different segment.
        """
        if seq not in self._outstanding:
            return
        for older in list(self._outstanding):
            if older == seq:
                break
            segment = self._outstanding.pop(older)
            self._sent_at.pop(older, None)
            self._consumed += segment.delta
            self._last_direction = segment.direction
            log.debug("%s: seq %d retired by a later report", self._name, older)

    def on_aborted(self, message: sp.Aborted) -> None:
        with self._lock:
            if message.seq != sp.SEQ_NONE:
                self._retire_older_than(message.seq)
                self._retire(message.seq, message.steps_done)
            # Whatever else the firmware was holding is gone. It is not lost
            # position — those pulses never went out — so give the steps back to
            # `committed` rather than leaving the planner to replan from a
            # position the antenna never reached.
            for segment in self._outstanding.values():
                self._committed -= segment.delta
            self._outstanding.clear()
            self._sent_at.clear()
            self._status_asked_at = None
            for segment in self._queue:
                self._committed -= segment.delta
            self._queue.clear()
            self._abort_report = message
        if message.cause != sp.AbortCause.HOST:
            log.warning(
                "%s: firmware aborted after %d steps (%s)",
                self._name,
                message.steps_done,
                message.cause_name,
            )

    def on_endstop(self, level: int) -> None:
        with self._lock:
            was = self._endstop_level
            self._endstop_level = level
            self._has_endstop = True
            # The inhibit tracks the switch rather than latching, so homing's
            # trip → back off → re-approach can happen at all. Latching the
            # *fault* is the supervisor's job, and it does it.
            self._inhibit_dir = -1 if level else 0
        if level and not was and self._endstop_callback is not None:
            self._endstop_callback()

    def on_reset(self) -> None:
        """The board rebooted: its queue is gone and so is our accounting."""
        with self._lock:
            self._position_lost = True
            for segment in self._queue:
                self._committed -= segment.delta
            self._queue.clear()
            self._outstanding.clear()
            self._sent_at.clear()
            self._status_asked_at = None
            self._committed = self._consumed

    def clear_position_loss(self) -> None:
        """Called once the axis has re-homed and the reference means something."""
        with self._lock:
            self._position_lost = False

    def _retire(self, seq: int, steps_done: int) -> None:
        segment = self._outstanding.pop(seq, None)
        self._sent_at.pop(seq, None)
        if segment is None:
            log.debug("%s: report for unknown seq %d", self._name, seq)
            return
        steps = max(0, min(int(steps_done), segment.steps))
        self._consumed += segment.direction * steps if segment.counts else 0
        if steps != segment.steps:
            # Short segment: give the unexecuted remainder back to `committed`,
            # or the planner would replan from a position the antenna never
            # reached and quietly bake in the error.
            shortfall = segment.steps - steps
            self._committed -= segment.direction * shortfall if segment.counts else 0
        self._last_direction = segment.direction

    def state_snapshot(self) -> dict:
        with self._lock:
            return {
                "queued": len(self._queue),
                "outstanding": len(self._outstanding),
                "consumed_steps": self._consumed,
                "committed_steps": self._committed,
                "endstop": bool(self._endstop_level),
                "enabled": self._enabled,
                "position_lost": self._position_lost,
            }


def _split(segment: Segment) -> list[Segment]:
    """Break a segment into wire-sized pieces.

    ``steps`` is a u16 on the wire. Nothing the planner emits is anywhere near
    that, but ``enqueue`` is a public entry point and a silently truncated step
    count is a position error nobody would ever find.
    """
    if segment.steps <= MAX_SEGMENT_STEPS:
        return [segment]
    pieces = []
    remaining = segment.steps
    while remaining > 0:
        take = min(remaining, MAX_SEGMENT_STEPS)
        pieces.append(
            Segment(
                direction=segment.direction,
                steps=take,
                period_us=segment.period_us,
                counts=segment.counts,
            )
        )
        remaining -= take
    return pieces


# --------------------------------------------------------------------------
# Backend
# --------------------------------------------------------------------------

class SerialBackend(Backend):
    """Owns the port, the handshake, and the two axis drivers."""

    name = "serial"

    #: How long to wait for ABORTED before giving up and calling the result
    #: inexact. Generous against a 1-2 ms link, because the alternative to
    #: waiting is claiming a step count we cannot back up.
    ABORT_TIMEOUT_S = 0.25
    #: Keep the firmware's watchdog fed while the antenna is parked. Comfortably
    #: inside FW_WATCHDOG_S so a single dropped frame is not an outage.
    PING_INTERVAL_S = 1.0
    #: No frame at all for this long means the board or the cable is gone.
    LINK_TIMEOUT_S = 3.0

    def __init__(
        self,
        config: RotatorConfig,
        *,
        transport: Transport | None = None,
        clock=None,
        connect: bool = True,
    ):
        from .clock import RealClock

        self.config = config
        self.clock = clock or RealClock()
        self.lock = threading.RLock()
        self.ident: sp.Ident | None = None
        self.fault: str = ""
        self.resets = 0
        self._estop_level = 0
        self._estop_cb: Callable[[], None] | None = None
        self._fault_cb: Callable[[str], None] | None = None
        self._drivers: dict[int, SerialAxisDriver] = {}
        self._ping_nonce = 0
        self._servicing = False
        #: An IDENT means "reset" only once the link is up and we did not just
        #: ask for one. Before that it is either the boot announcement or the
        #: answer to our HELLO, and both are expected.
        self._handshake_done = False
        self._hello_pending = False

        if transport is None:
            transport = SerialTransport(self._resolve_port(), config.serial.baud)
        self.transport = transport
        self.link = SerialLink(transport, clock=self.clock)
        self.link.handler = self._dispatch
        if connect:
            self.handshake()

    # -- setup --------------------------------------------------------------

    def _resolve_port(self) -> str:
        """Pick the port to open, using the same rule detection used.

        A configured port is taken at face value — naming a device is how you
        override every guess here. "auto" only ever picks a port that looks like
        an Arduino, because opening a port resets the board behind it and writes
        a HELLO at whatever is listening; doing that to the rig's CAT interface
        to find out what it is would be unacceptable.
        """
        configured = self.config.serial.port
        if configured and configured != "auto":
            return configured
        from .detect import FIRMWARE_PORT_MARKERS

        ports = probe_serial_ports()
        likely = [p for p in ports if any(m in p.lower() for m in FIRMWARE_PORT_MARKERS)]
        if likely:
            return likely[0]
        if ports:
            raise TransportError(
                "no port looks like an Arduino, but these serial devices are present: "
                + ", ".join(ports)
                + ". Set rotator.serial.port explicitly — auto-detection will not "
                "open a device it cannot identify."
            )
        raise TransportError(
            "no serial device found — looked in /dev/serial/by-id, /dev/ttyACM*, "
            "/dev/ttyUSB*. Is the Arduino plugged in?"
        )

    def handshake(self) -> sp.Ident:
        """Say hello, and refuse to run against firmware we do not understand.

        The board may already be mid-boot (opening the port reset it), so a boot
        IDENT is accepted just as readily as an answer to HELLO. A version
        mismatch fails here, in ``orbitaly doctor``, rather than halfway through
        a pass.
        """
        deadline = self.clock.monotonic() + self.config.serial.connect_timeout_s
        while True:
            self._hello_pending = True
            self.send(sp.Hello(sp.PROTOCOL_VERSION))
            if self.wait_for(lambda: self.ident is not None, 0.5):
                break
            if self.clock.monotonic() >= deadline:
                self._hello_pending = False
                raise TransportError(
                    "no IDENT from the firmware — wrong port, board not flashed, "
                    "or the sketch is not running"
                )
        # Drain the rest of what is already buffered before arming reset
        # detection, so the boot IDENT and the answer to HELLO — the firmware
        # legitimately sends both — are not mistaken for a reboot.
        self.service()
        self._hello_pending = False
        ident = self.ident
        assert ident is not None
        if ident.protocol != sp.PROTOCOL_VERSION:
            raise TransportError(
                f"firmware speaks protocol {ident.protocol}, this build speaks "
                f"{sp.PROTOCOL_VERSION} — reflash firmware/orbitaly_rotator"
            )
        self._handshake_done = True
        log.info(
            "Firmware %s, protocol %d, queue depth %d, %d axes",
            ident.version_str,
            ident.protocol,
            ident.queue_depth,
            ident.axis_count,
        )
        return ident

    @property
    def queue_depth(self) -> int:
        return self.ident.queue_depth if self.ident else DEFAULT_QUEUE_DEPTH

    # -- Backend ------------------------------------------------------------

    def axis_driver(self, config: AxisConfig, name: str) -> AxisDriver:
        index = _AXIS_INDEX.get(name, len(self._drivers))
        driver = SerialAxisDriver(self, index, config, name)
        if self.ident is not None:
            cap = sp.Caps.AZ_ENDSTOP if index == sp.AXIS_AZ else sp.Caps.EL_ENDSTOP
            driver._has_endstop = self.ident.has(cap)
        self._drivers[index] = driver
        return driver

    def estop_triggered(self) -> bool:
        with self.lock:
            return bool(self._estop_level)

    def arm_estop(self, callback: Callable[[], None] | None) -> None:
        self._estop_cb = callback

    def arm_fault(self, callback: Callable[[str], None] | None) -> None:
        self._fault_cb = callback

    def close(self) -> None:
        try:
            for index in self._drivers:
                self.link.send(sp.Enable(index, 0))
        except (TransportError, OSError):
            pass
        self.transport.close()

    def describe(self) -> str:
        port = getattr(self.transport, "port", "in-memory")
        if self.ident is None:
            return f"serial on {port} (no handshake)"
        return f"serial on {port}, firmware {self.ident.version_str} protocol {self.ident.protocol}"

    # -- link ---------------------------------------------------------------

    def send(self, message) -> None:
        try:
            self.link.send(message)
        except TransportError as exc:
            self.fail(str(exc))
            raise

    def send_safe(self, message) -> None:
        """Send, swallowing a dead link — for teardown and best-effort commands."""
        try:
            self.send(message)
        except (TransportError, OSError):
            pass

    def service(self) -> int:
        """Read pending frames, dispatch them, and keep the watchdogs fed."""
        with self.lock:
            if self._servicing:
                return 0  # re-entered from a handler; the outer call will finish
            self._servicing = True
        try:
            handled = self.link.service()
        except TransportError as exc:
            self.fail(str(exc))
            return 0
        finally:
            with self.lock:
                self._servicing = False

        now = self.clock.monotonic()
        if self.ident is not None and now - self.link.last_tx >= self.PING_INTERVAL_S:
            # The firmware's watchdog counts *any* valid frame, so a parked
            # antenna still has to say something or the enables drop.
            self._ping_nonce = (self._ping_nonce + 1) & 0xFF
            self.send_safe(sp.Ping(self._ping_nonce))
        if (
            self.ident is not None
            and not self.fault
            and now - self.link.last_rx > self.LINK_TIMEOUT_S
        ):
            self.fail("no response from the firmware — check the USB cable")
        return handled

    def wait_for(self, predicate, timeout_s: float) -> bool:
        """Pump the link until ``predicate`` holds. Bounded, and never blocks
        on a clock that is not moving."""
        deadline = self.clock.monotonic() + timeout_s
        while True:
            self.service()
            if predicate():
                return True
            if self.clock.monotonic() >= deadline:
                return False
            # 1 ms is inside VirtualClock's "this is a real wait" threshold, so
            # this terminates under the test harness as well as on the Pi.
            self.clock.sleep(0.001)

    def fail(self, reason: str) -> None:
        """Latch a backend fault and tell whoever is steering."""
        with self.lock:
            if self.fault:
                return
            self.fault = reason
        log.error("Serial backend fault: %s", reason)
        for driver in self._drivers.values():
            driver.on_reset()
        if self._fault_cb is not None:
            self._fault_cb(reason)

    # -- dispatch -----------------------------------------------------------

    def _dispatch(self, message) -> None:
        if isinstance(message, sp.Ident):
            self._on_ident(message)
        elif isinstance(message, sp.Ack):
            self._axis(message.axis, message.__class__.__name__, lambda d: d.on_ack(message))
        elif isinstance(message, sp.Nack):
            self._axis(message.axis, "NACK", lambda d: d.on_nack(message))
        elif isinstance(message, sp.Done):
            self._axis(message.axis, "DONE", lambda d: d.on_done(message))
        elif isinstance(message, sp.Aborted):
            self._axis(message.axis, "ABORTED", lambda d: d.on_aborted(message))
        elif isinstance(message, sp.Event):
            self._on_event(message)
        elif isinstance(message, sp.State):
            self._on_state(message)
        elif isinstance(message, sp.Pong):
            pass  # liveness only; last_rx already moved
        else:
            log.debug("ignoring %s from firmware", type(message).__name__)

    def _axis(self, index: int, what: str, action) -> None:
        driver = self._drivers.get(index)
        if driver is None:
            log.warning("%s for unknown axis %d", what, index)
            return
        action(driver)

    def _on_ident(self, ident: sp.Ident) -> None:
        self.ident = ident
        for index, driver in self._drivers.items():
            cap = sp.Caps.AZ_ENDSTOP if index == sp.AXIS_AZ else sp.Caps.EL_ENDSTOP
            driver._has_endstop = ident.has(cap)
        if not self._handshake_done or self._hello_pending:
            # Boot announcement, or the answer to our own HELLO. The firmware
            # repeats its boot IDENT until the host says something, so several
            # in a row before the handshake completes are normal.
            self._hello_pending = False
            return
        # An IDENT nobody asked for means the board reset — brownout, watchdog,
        # a replugged cable. Its queue is gone, so every segment accepted but not
        # reported is unaccounted for, and both axes must re-home before their
        # soft limits mean anything again.
        self.resets += 1
        log.error("Firmware rebooted mid-session (IDENT %s)", ident.version_str)
        for driver in self._drivers.values():
            driver.on_reset()
        # Re-handshake so the link itself comes back — a rebooted board refuses
        # every command until it has seen a version-matched HELLO, and a station
        # that needs a human to restart the process after a brownout is not one
        # you can leave running unattended. Recovering the *link* is not the same
        # as trusting the *position*, which is why the fault below still stands.
        self._hello_pending = True
        self.send_safe(sp.Hello(sp.PROTOCOL_VERSION))
        if self._fault_cb is not None:
            self._fault_cb("firmware rebooted — position reference lost, re-home before moving")

    def _on_event(self, event: sp.Event) -> None:
        if event.kind == sp.EventKind.ESTOP:
            with self.lock:
                was = self._estop_level
                self._estop_level = event.state
            if event.state and not was and self._estop_cb is not None:
                self._estop_cb()
            return
        index = sp.AXIS_AZ if event.kind == sp.EventKind.ENDSTOP_AZ else sp.AXIS_EL
        self._axis(index, "EVENT", lambda d: d.on_endstop(event.state))

    def _on_state(self, state: sp.State) -> None:
        with self.lock:
            self._estop_level = state.estop
        for index, axis_state in enumerate(state.axes):
            driver = self._drivers.get(index)
            if driver is not None:
                driver.on_endstop(axis_state.endstop)
                driver.on_state(axis_state)

    # -- diagnostics --------------------------------------------------------

    def request_state(self) -> sp.State | None:
        """Ask for a STATE snapshot and wait briefly for it. Used by doctor."""
        received: list[sp.State] = []
        previous = self.link.handler

        def capture(message):
            if isinstance(message, sp.State):
                received.append(message)
            if previous is not None:
                previous(message)

        self.link.handler = capture
        try:
            self.send_safe(sp.Status())
            self.wait_for(lambda: bool(received), 0.5)
        finally:
            self.link.handler = previous
        return received[0] if received else None

    def ping(self) -> bool:
        """One round trip. Returns False if the firmware did not answer."""
        self._ping_nonce = (self._ping_nonce + 1) & 0xFF
        nonce = self._ping_nonce
        seen: list[int] = []
        previous = self.link.handler

        def capture(message):
            if isinstance(message, sp.Pong) and message.nonce == nonce:
                seen.append(message.nonce)
            if previous is not None:
                previous(message)

        self.link.handler = capture
        try:
            self.send_safe(sp.Ping(nonce))
            return self.wait_for(lambda: bool(seen), 1.0)
        finally:
            self.link.handler = previous

    @property
    def stats(self) -> dict:
        stats = dict(self.link.stats)
        stats["resets"] = self.resets
        stats["fault"] = self.fault
        return stats
