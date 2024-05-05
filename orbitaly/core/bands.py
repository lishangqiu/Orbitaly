"""What this station can actually work — in frequency and above the horizon.

Geometry is not workability. The map draws where a satellite *is*; these two
gates decide what the map is allowed to *claim*. Keeping them here, pure and
testable, is what stops "in view" from quietly becoming "workable" somewhere
in the rendering code.
"""
from __future__ import annotations

from ..config import BandConfig, Config

#: Every transponder has a downlink and an uplink this station can use.
TWO_WAY = "two_way"
#: A downlink lands in a receive-capable band; the uplink does not, or there
#: is none. This is a real and common way to use a station — it is how a
#: SatNOGS-style receive-only site works — so it counts as workable.
RX_ONLY = "rx_only"
#: The satellite has transponder data and none of it is reachable from here.
OUT_OF_BAND = "out_of_band"
#: No transponder data, so no claim can be made either way.
UNKNOWN = "unknown"

#: Why a satellite is not workable, for the map's hover readout. Phrased as
#: facts about this station, because that is what they are.
REASONS = {
    TWO_WAY: None,
    RX_ONLY: "receive only — no uplink in station bands",
    OUT_OF_BAND: "no downlink in station bands",
    UNKNOWN: "no transponder data",
}


def band_status(transponders: list[dict], bands: list[BandConfig]) -> str:
    """Classify one satellite against the station's band capability.

    Evaluated per transponder, not per satellite: a bird with one transponder
    whose downlink is reachable and another whose uplink is would otherwise be
    reported as two-way workable when in fact neither transponder can be
    worked both ways.
    """
    if not transponders:
        return UNKNOWN
    rx_bands = [b for b in bands if b.rx]
    tx_bands = [b for b in bands if b.tx]
    best = OUT_OF_BAND
    for t in transponders:
        if not any(b.contains(t.get("downlink_hz")) for b in rx_bands):
            continue
        if any(b.contains(t.get("uplink_hz")) for b in tx_bands):
            return TWO_WAY
        best = RX_ONLY
    return best


def is_workable(status: str) -> bool:
    """Whether the station can hear it at all. RX-only counts."""
    return status in (TWO_WAY, RX_ONLY)


def effective_mask_deg(config: Config) -> float:
    """The elevation the map may call "in view".

    Never below what the tracker will actually schedule, so the map cannot
    promise a pass the rest of the software would refuse to take.
    """
    return max(
        config.station.min_workable_elevation_deg,
        config.tracker.min_elevation_deg,
    )
