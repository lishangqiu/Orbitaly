"""Motion-layer exceptions."""
from __future__ import annotations


class MotionBlocked(RuntimeError):
    """A motion command was refused by an interlock.

    Raised rather than silently ignored: a ground station that quietly declines
    to move is worse than one that says why.
    """
