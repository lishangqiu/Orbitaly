"""Deprecated shim.

v0.1 drove the pins from here with ``RPi.GPIO``, one Python-timed pulse at a
time. That library does not work on a Raspberry Pi 5 at all — the RP1
southbridge moved the pins behind the kernel's gpiochip interface — and the
per-step timing loop was never going to hold up under load anyway.

The real implementation now lives in :mod:`orbitaly.motion`. This module stays
so that ``from orbitaly.hardware.gpio import make_rotator`` keeps working, and
so ``backend: gpio`` in an old config still starts (it resolves to ``lgpio``).
"""
from __future__ import annotations

from ..motion.detect import make_rotator

__all__ = ["make_rotator"]
