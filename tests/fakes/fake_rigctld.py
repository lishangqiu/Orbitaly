"""A socket server that speaks hamlib's rigctld extended-response protocol.

Real TCP, real line framing, real ``RPRT`` codes — so the client under test is
exercised end to end rather than against a mocked-out method.
"""
from __future__ import annotations

import socket
import threading


class FakeRigctld:
    def __init__(self, *, fail_after: int | None = None):
        self.rx_hz = 145_800_000
        self.tx_hz = 145_990_000
        self.split = False
        self.ptt = 0
        self.mode = "USB"
        self.width = 2400
        self.commands: list[str] = []
        #: drop the connection after this many commands, to test recovery
        self.fail_after = fail_after

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(4)
        self.port = self._server.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True, name="fake-rigctld")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._server.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)

    # -- server -------------------------------------------------------------

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._server.accept()
            except OSError:
                return
            threading.Thread(target=self._session, args=(conn,), daemon=True).start()

    def _session(self, conn: socket.socket) -> None:
        served = 0
        with conn, conn.makefile("rw", encoding="ascii", newline="\n") as stream:
            while not self._stop.is_set():
                line = stream.readline()
                if not line:
                    return
                line = line.strip()
                if not line:
                    continue
                self.commands.append(line)
                served += 1
                if self.fail_after is not None and served > self.fail_after:
                    return  # yank the connection mid-conversation
                try:
                    stream.write(self._respond(line))
                    stream.flush()
                except OSError:
                    return

    def _respond(self, line: str) -> str:
        extended = line.startswith("+")
        body = line.lstrip("+").lstrip("\\")
        parts = body.split()
        name, args = parts[0], parts[1:]

        values: list[tuple[str, str]] = []
        code = 0
        if name == "set_freq":
            self.rx_hz = int(float(args[0]))
        elif name == "get_freq":
            values = [("Frequency", str(self.rx_hz))]
        elif name == "set_split_freq":
            self.tx_hz = int(float(args[0]))
        elif name == "get_split_freq":
            values = [("TX Frequency", str(self.tx_hz))]
        elif name == "set_split_vfo":
            self.split = args[0] == "1"
        elif name == "get_ptt":
            values = [("PTT", str(self.ptt))]
        elif name == "get_mode":
            values = [("Mode", self.mode), ("Passband", str(self.width))]
        elif name == "set_mode":
            self.mode = args[0]
        else:
            code = -4  # RIG_ENIMPL

        if not extended:
            payload = "".join(f"{value}\n" for _, value in values)
            return payload if code == 0 else f"RPRT {code}\n"

        head = f"{name}: {' '.join(args)}\n"
        payload = "".join(f"{label}: {value}\n" for label, value in values)
        return f"{head}{payload}RPRT {code}\n"
