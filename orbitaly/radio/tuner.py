"""The doppler tuning loop.

Signs are the thing to get right, and they are not symmetric:

- **Downlink (RX).** The satellite transmits on ``downlink_hz``; approaching,
  you hear it high. Tune to ``downlink + shift``.
- **Uplink (TX).** The satellite must *hear* ``uplink_hz``; approaching, it
  hears you high, so transmit low. Tune to ``uplink - shift``.

Getting the uplink sign backwards is the classic satellite-operator bug: it
looks fine at AOS, drifts twice as fast as it should, and puts you a couple of
kilohertz off the transponder passband at TCA.

The loop is deliberately lazy — deadband per mode, a cap on writes per second
— because a rig retuned at full rate is a rig clicking its audio a hundred
times a minute for no audible benefit.
"""
from __future__ import annotations

import logging
import threading
from typing import Callable

from ..config import RigConfig
from .base import Rig, RigError

log = logging.getLogger(__name__)


class DopplerTuner:
    TICK_S = 0.25

    def __init__(
        self,
        config: RigConfig,
        rig: Rig | None,
        readout_fn: Callable[[], dict],
        *,
        clock=None,
    ):
        from ..motion.clock import RealClock

        self.config = config
        self.rig = rig
        self.readout_fn = readout_fn
        self.clock = clock or RealClock()
        self.enabled = config.enabled

        self._lock = threading.RLock()
        self._last_rx: float | None = None
        self._last_tx: float | None = None
        self._target_rx: float | None = None
        self._target_tx: float | None = None
        self._last_tune_t = -1e9
        self._next_attempt_t = 0.0
        self._mode = ""
        self._mode_polled_t = -1e9
        self._error = ""
        self._tunes = 0
        self._stop_flag = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        if self.rig is None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="doppler-tuner")
        self._thread.start()

    def close(self) -> None:
        self._stop_flag.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.rig is not None:
            self.rig.close()

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                self.tick()
            except Exception:  # noqa: BLE001 - the radio must never take tracking down
                log.exception("Doppler tuner tick failed")
            self.clock.sleep(self.TICK_S)

    # -- control ------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.enabled = enabled
            if not enabled:
                self._target_rx = self._target_tx = None

    def status(self) -> dict:
        with self._lock:
            rig_status = self.rig.status() if self.rig is not None else None
            return {
                "backend": self.config.backend,
                "present": self.rig is not None,
                "enabled": self.enabled,
                "connected": bool(rig_status and rig_status.connected),
                "error": self._error or (rig_status.error if rig_status else ""),
                "mode": self._mode,
                "deadband_hz": self._deadband(),
                "target_rx_hz": self._target_rx,
                "target_tx_hz": self._target_tx,
                "rig_rx_hz": rig_status.rx_hz if rig_status else None,
                "rig_tx_hz": rig_status.tx_hz if rig_status else None,
                "tunes": self._tunes,
                "uplink_hz": self.config.uplink_hz or None,
                "downlink_hz": self.config.downlink_hz or None,
            }

    # -- the loop -----------------------------------------------------------

    def tick(self) -> None:
        with self._lock:
            if self.rig is None or not self.enabled:
                return
            now = self.clock.monotonic()
            if not self._ensure_connected(now):
                return
            if now - self._last_tune_t < 1.0 / max(self.config.max_tune_rate_hz, 0.01):
                return

            readout = self.readout_fn()
            if not readout.get("active"):
                self._target_rx = self._target_tx = None
                return
            if readout.get("elevation", 90.0) < self.config.min_elevation_deg:
                self._target_rx = self._target_tx = None
                return

            self._poll_mode(now)
            self._target_rx = readout.get("downlink_corrected_hz")
            self._target_tx = readout.get("uplink_corrected_hz")
            deadband = self._deadband()

            try:
                tuned = False
                if self.config.tune_rx and self._should_tune(self._target_rx, self._last_rx, deadband):
                    self.rig.set_rx_freq(self._target_rx)
                    self._last_rx = self._target_rx
                    tuned = True
                if (
                    self.config.tune_tx
                    and self._should_tune(self._target_tx, self._last_tx, deadband)
                    and not self._tx_blocked()
                ):
                    self.rig.set_tx_freq(self._target_tx)
                    self._last_tx = self._target_tx
                    tuned = True
            except RigError as exc:
                self._on_rig_error(exc, now)
                return

            self._error = ""
            if tuned:
                self._tunes += 1
                self._last_tune_t = now

    # -- helpers ------------------------------------------------------------

    def _ensure_connected(self, now: float) -> bool:
        if self.rig.connected:
            return True
        if now < self._next_attempt_t:
            return False
        try:
            self.rig.open()
            # Satellite operation is split by definition: RX here, TX there.
            self.rig.set_split(True, self.config.tx_vfo)
        except RigError as exc:
            self._on_rig_error(exc, now)
            return False
        self._last_rx = self._last_tx = None
        self._mode_polled_t = -1e9
        return True

    def _on_rig_error(self, exc: Exception, now: float) -> None:
        backoff = getattr(self.rig, "backoff_s", lambda: 5.0)()
        self._error = str(exc)
        self._next_attempt_t = now + backoff
        log.warning("Rig error (%s); retrying in %.0fs", exc, backoff)

    def _poll_mode(self, now: float) -> None:
        if now - self._mode_polled_t < self.config.mode_poll_s:
            return
        self._mode_polled_t = now
        try:
            self._mode = self.rig.get_mode() or self._mode
        except RigError:
            pass  # a rig that will not report its mode still gets the default deadband

    def _deadband(self) -> float:
        table = self.config.deadbands_hz
        return float(table.get(self._mode.upper(), table.get("default", 20.0)))

    @staticmethod
    def _should_tune(target: float | None, last: float | None, deadband: float) -> bool:
        if target is None:
            return False
        return last is None or abs(target - last) >= deadband

    def _tx_blocked(self) -> bool:
        if self.config.tune_while_tx:
            return False
        try:
            return self.rig.get_ptt()
        except RigError:
            return True
