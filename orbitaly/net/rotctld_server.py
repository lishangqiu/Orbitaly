"""Speak hamlib's rotator protocol, so existing tools can drive Orbitaly.

gpredict, SatPC32 bridges, satnogs-client and anything else built around
``rotctld`` all know how to point an antenna by opening TCP 4533 and writing
``P <az> <el>``. Implementing that here means Orbitaly's stepper control,
soft limits and homing interlocks are available to those tools without them
knowing anything about Orbitaly.

Two response formats are supported, because clients differ:

- **Default** — what hamlib's own NET rotctl client (rotator model 2) uses.
  Set commands answer ``RPRT 0``; get commands answer bare values, one per
  line.
- **Extended** — any command prefixed with ``+``, ``;``, ``|`` or ``,``. The
  command name is echoed, values are labelled, and every reply ends with
  ``RPRT``. This is what a human or a script wants.

Incoming azimuths are plain 0..360 sky bearings. They are mapped into the
rotator's real travel by :func:`~orbitaly.core.tracker.map_into_travel`, so a
client that knows nothing about a -90..450 mount still gets sane behaviour
near north.
"""
from __future__ import annotations

import asyncio
import logging

from ..core.tracker import map_into_travel
from ..motion.errors import MotionBlocked

log = logging.getLogger(__name__)

RPRT_OK = 0
RPRT_EINVAL = -1
RPRT_ENIMPL = -4
RPRT_EPROTO = -8
#: -RIG_ERJCTED, "rejected by the rig" — the closest hamlib code to "an
#: interlock refused this", and one existing clients already understand.
RPRT_REJECTED = -11

#: (long name, argument count) for each single-letter command
COMMANDS = {
    "P": ("set_pos", 2),
    "p": ("get_pos", 0),
    "S": ("stop", 0),
    "K": ("park", 0),
    "M": ("move", 2),
    "R": ("reset", 1),
    "_": ("get_info", 0),
    "1": ("dump_caps", 0),
}
LONG_NAMES = {long: short for short, (long, _) in COMMANDS.items()}

MOVE_UP, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT = 2, 4, 8, 16
JOG_DEG = 1.0


class RotctldError(Exception):
    def __init__(self, code: int, message: str = ""):
        super().__init__(message or f"RPRT {code}")
        self.code = code


class RotctldServer:
    def __init__(self, services, config):
        self.services = services
        self.config = config
        self._server: asyncio.AbstractServer | None = None

    @property
    def port(self) -> int:
        if self._server is None:
            return self.config.port
        return self._server.sockets[0].getsockname()[1]

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._handle, self.config.host, self.config.port
        )
        log.info("rotctld-compatible server listening on %s:%d", self.config.host, self.port)

    async def stop(self) -> None:
        if self._server is None:
            return
        self._server.close()
        await self._server.wait_closed()
        self._server = None

    # -- session ------------------------------------------------------------

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        log.info("rotctld client connected from %s", peer)
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line in ("q", "Q", "\\quit"):
                    break
                response = self.dispatch(line)
                if response:
                    writer.write(response.encode("ascii"))
                    await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        finally:
            log.info("rotctld client %s disconnected", peer)
            writer.close()

    # -- protocol -----------------------------------------------------------

    def dispatch(self, line: str) -> str:
        """Run one command line and render its response."""
        separator = "\n"
        extended = False
        if line[0] in "+;|,":
            extended = True
            separator = "\n" if line[0] == "+" else line[0]
            line = line[1:].strip()

        parts = line.split()
        if not parts:
            return ""
        token, args = parts[0], parts[1:]
        if token.startswith("\\"):
            name = token[1:]
            short = LONG_NAMES.get(name)
            if name not in ("dump_state",) and short is None:
                return self._render(name, [], RPRT_ENIMPL, extended, separator)
        else:
            short = token[0]
            entry = COMMANDS.get(short)
            if entry is None:
                return self._render(short, [], RPRT_ENIMPL, extended, separator)
            name = entry[0]
            # "P 10 20" may also arrive as "P10 20" from terse clients.
            if len(token) > 1:
                args = [token[1:], *args]

        try:
            values = self._execute(name, args)
            return self._render(name, values, RPRT_OK, extended, separator, echo=args)
        except RotctldError as exc:
            log.debug("rotctld %s -> %s", name, exc)
            return self._render(name, [], exc.code, extended, separator, echo=args)

    def _render(
        self,
        name: str,
        values: list[tuple[str, str]],
        code: int,
        extended: bool,
        separator: str,
        echo: list[str] | None = None,
    ) -> str:
        if extended:
            head = f"{name}: {' '.join(echo or [])}{separator}"
            body = "".join(f"{label}: {value}{separator}" for label, value in values)
            return f"{head}{body}RPRT {code}{separator}"
        if code != RPRT_OK:
            return f"RPRT {code}\n"
        if not values:
            return "RPRT 0\n"
        return "".join(f"{value}\n" for _, value in values)

    # -- commands -----------------------------------------------------------

    def _execute(self, name: str, args: list[str]) -> list[tuple[str, str]]:
        handler = getattr(self, f"_cmd_{name}", None)
        if handler is None:
            raise RotctldError(RPRT_ENIMPL)
        return handler(args)

    def _cmd_set_pos(self, args: list[str]) -> list[tuple[str, str]]:
        if len(args) < 2:
            raise RotctldError(RPRT_EINVAL)
        try:
            azimuth, elevation = float(args[0]), float(args[1])
        except ValueError as exc:
            raise RotctldError(RPRT_EINVAL) from exc
        rotator = self.services.rotator
        commanded = map_into_travel(
            azimuth, rotator.state().azimuth, *rotator.azimuth_travel
        )
        try:
            self.services.tracker.manual_goto(commanded, elevation)
        except MotionBlocked as exc:
            raise RotctldError(RPRT_REJECTED, str(exc)) from exc
        return []

    def _cmd_get_pos(self, args: list[str]) -> list[tuple[str, str]]:
        state = self.services.rotator.state()
        # Report a plain compass bearing: a client that sent 350 should not be
        # told the antenna is at -10.
        azimuth = state.azimuth % 360.0
        return [("Azimuth", f"{azimuth:.6f}"), ("Elevation", f"{state.elevation:.6f}")]

    def _cmd_stop(self, args: list[str]) -> list[tuple[str, str]]:
        self.services.tracker.manual_stop()
        return []

    def _cmd_park(self, args: list[str]) -> list[tuple[str, str]]:
        try:
            self.services.tracker.park()
        except MotionBlocked as exc:
            raise RotctldError(RPRT_REJECTED, str(exc)) from exc
        return []

    def _cmd_move(self, args: list[str]) -> list[tuple[str, str]]:
        if len(args) < 2:
            raise RotctldError(RPRT_EINVAL)
        try:
            direction, speed = int(args[0]), int(args[1])
        except ValueError as exc:
            raise RotctldError(RPRT_EINVAL) from exc
        # Hamlib's move is "go until told to stop"; Orbitaly is position-based,
        # so treat it as a jog whose size scales with the requested speed.
        scale = JOG_DEG * (max(speed, 1) / 100.0 if speed > 0 else 1.0)
        deltas = {
            MOVE_UP: (0.0, scale),
            MOVE_DOWN: (0.0, -scale),
            MOVE_LEFT: (-scale, 0.0),
            MOVE_RIGHT: (scale, 0.0),
        }
        if direction not in deltas:
            raise RotctldError(RPRT_EINVAL)
        try:
            self.services.tracker.manual_jog(*deltas[direction])
        except MotionBlocked as exc:
            raise RotctldError(RPRT_REJECTED, str(exc)) from exc
        return []

    def _cmd_reset(self, args: list[str]) -> list[tuple[str, str]]:
        self.services.rotator.home()
        return []

    def _cmd_get_info(self, args: list[str]) -> list[tuple[str, str]]:
        return [("Info", "Orbitaly stepper rotator")]

    def _cmd_dump_caps(self, args: list[str]) -> list[tuple[str, str]]:
        az_min, az_max = self.services.rotator.azimuth_travel
        el_min, el_max = self.services.rotator.elevation_travel
        return [
            ("Model name", "Orbitaly"),
            ("Mfg name", "Orbitaly"),
            ("Backend version", "0.1.0"),
            ("Rotator type", "AzEl"),
            ("Min Azimuth", f"{az_min:.2f}"),
            ("Max Azimuth", f"{az_max:.2f}"),
            ("Min Elevation", f"{el_min:.2f}"),
            ("Max Elevation", f"{el_max:.2f}"),
        ]

    def _cmd_dump_state(self, args: list[str]) -> list[tuple[str, str]]:
        """State block read by hamlib's own NET rotctl client on connect.

        Field order here follows what netrotctl reads: protocol version, model,
        then the azimuth and elevation limits. It is not specified in the
        rotctld man page, so treat it as validated only against the version of
        ``rotctl -m 2`` you test with — see tests/test_rotctld.py, which runs
        the real client when hamlib is installed.
        """
        az_min, az_max = self.services.rotator.azimuth_travel
        el_min, el_max = self.services.rotator.elevation_travel
        return [
            ("protocol", "0"),
            ("model", "2"),
            ("min_az", f"{az_min:.6f}"),
            ("max_az", f"{az_max:.6f}"),
            ("min_el", f"{el_min:.6f}"),
            ("max_el", f"{el_max:.6f}"),
        ]
