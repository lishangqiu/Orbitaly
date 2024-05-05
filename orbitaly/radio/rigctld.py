"""Hamlib ``rigctld`` client.

rigctld listens on TCP 4532 and speaks a line protocol. Orbitaly uses the
Extended Response Protocol — every command prefixed with ``+`` — because it
echoes the command name, labels each returned value, and always terminates
with ``RPRT <code>``. The default protocol returns bare values with no frame,
which is fine for a human at a telnet prompt and miserable to parse reliably::

    +\\get_freq          ->  get_freq:
                             Frequency: 145800000
                             RPRT 0

    +\\set_freq 145800000 -> set_freq: 145800000
                             RPRT 0

A rig that goes away must never take tracking down with it, so every failure
here degrades to a status message and a reconnect attempt.
"""
from __future__ import annotations

import logging
import socket
import threading

from ..config import RigConfig
from .base import Rig, RigError, RigStatus

log = logging.getLogger(__name__)

RECONNECT_BACKOFF_S = (1.0, 2.0, 5.0, 10.0, 30.0)


class RigctldRig(Rig):
    def __init__(self, config: RigConfig):
        self.config = config
        self._lock = threading.RLock()
        self._sock: socket.socket | None = None
        self._file = None
        self._error = ""
        self._failures = 0
        self._next_attempt = 0.0

    # -- connection ---------------------------------------------------------

    def open(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            try:
                self._sock = socket.create_connection(
                    (self.config.host, self.config.port), timeout=self.config.timeout_s
                )
                self._sock.settimeout(self.config.timeout_s)
                self._file = self._sock.makefile("rw", encoding="ascii", newline="\n")
                self._error = ""
                self._failures = 0
                log.info("Connected to rigctld at %s:%d", self.config.host, self.config.port)
            except OSError as exc:
                self._teardown()
                self._failures += 1
                self._error = f"cannot reach rigctld at {self.config.host}:{self.config.port}: {exc}"
                raise RigError(self._error) from exc

    def close(self) -> None:
        with self._lock:
            self._teardown()

    def _teardown(self) -> None:
        for closeable in (self._file, self._sock):
            try:
                if closeable is not None:
                    closeable.close()
            except OSError:
                pass
        self._file = None
        self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def backoff_s(self) -> float:
        """How long to wait before the next reconnect attempt."""
        index = min(self._failures, len(RECONNECT_BACKOFF_S)) - 1
        return RECONNECT_BACKOFF_S[max(index, 0)]

    # -- protocol -----------------------------------------------------------

    def command(self, text: str) -> dict[str, str]:
        """Run one extended-protocol command and return its labelled values."""
        with self._lock:
            if self._sock is None:
                self.open()
            try:
                self._file.write(f"+\\{text}\n")
                self._file.flush()
                return self._read_response(text)
            except (OSError, RigError) as exc:
                self._teardown()
                self._failures += 1
                self._error = str(exc)
                raise RigError(f"{text}: {exc}") from exc

    def _read_response(self, text: str) -> dict[str, str]:
        values: dict[str, str] = {}
        while True:
            line = self._file.readline()
            if not line:
                raise RigError("rigctld closed the connection")
            line = line.strip()
            if line.startswith("RPRT"):
                code = int(line.split()[1])
                if code != 0:
                    raise RigError(f"rigctld returned {code} ({_rprt_name(code)})")
                return values
            if ":" in line:
                label, _, value = line.partition(":")
                values[label.strip()] = value.strip()

    # -- Rig ----------------------------------------------------------------

    def set_rx_freq(self, hz: float) -> None:
        self.command(f"set_freq {int(round(hz))}")

    def set_tx_freq(self, hz: float) -> None:
        self.command(f"set_split_freq {int(round(hz))}")

    def get_rx_freq(self) -> float:
        return float(self.command("get_freq").get("Frequency", 0.0))

    def get_tx_freq(self) -> float:
        values = self.command("get_split_freq")
        return float(values.get("TX Frequency", values.get("Frequency", 0.0)))

    def set_split(self, enabled: bool, tx_vfo: str = "VFOB") -> None:
        self.command(f"set_split_vfo {1 if enabled else 0} {tx_vfo}")

    def get_ptt(self) -> bool:
        return self.command("get_ptt").get("PTT", "0") != "0"

    def get_mode(self) -> str:
        return self.command("get_mode").get("Mode", "")

    def status(self) -> RigStatus:
        return RigStatus(connected=self.connected, error=self._error)


def _rprt_name(code: int) -> str:
    return {
        -1: "RIG_EINVAL, invalid parameter",
        -4: "RIG_ENIMPL, not implemented by this rig",
        -5: "RIG_ETIMEOUT",
        -6: "RIG_EIO, rig I/O error",
        -9: "RIG_EPROTO, protocol error",
        -11: "RIG_ERJCTED, rejected by the rig",
    }.get(code, "see hamlib rig_errcode_e")
