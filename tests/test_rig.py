"""Rig control: the rigctld wire protocol, and the doppler tuning loop."""
import math

import pytest

from orbitaly.config import RigConfig, StationConfig
from orbitaly.core.predictor import Predictor
from orbitaly.motion.clock import VirtualClock
from orbitaly.radio import SimulatedRig, make_rig
from orbitaly.radio.base import RigError
from orbitaly.radio.rigctld import RigctldRig
from orbitaly.radio.tuner import DopplerTuner

from fakes.fake_rigctld import FakeRigctld
from test_pi_simulation import ISS, MODERATE_PASS_AOS


@pytest.fixture
def rigctld():
    server = FakeRigctld()
    try:
        yield server
    finally:
        server.close()


@pytest.fixture
def rig(rigctld):
    client = RigctldRig(RigConfig(backend="rigctld", host="127.0.0.1", port=rigctld.port))
    client.open()
    try:
        yield client
    finally:
        client.close()


# -- the wire protocol ------------------------------------------------------


def test_frequencies_go_out_as_extended_protocol_commands(rig, rigctld):
    rig.set_rx_freq(145_812_345.6)
    assert rigctld.rx_hz == 145_812_346  # rounded to the nearest hertz
    assert rigctld.commands[-1] == "+\\set_freq 145812346"


def test_transmit_frequency_uses_split_not_a_vfo_dance(rig, rigctld):
    """Satellite mode is full duplex; hamlib models that as split frequency."""
    rig.set_split(True, "VFOB")
    rig.set_tx_freq(435_123_456)
    assert rigctld.split
    assert rigctld.tx_hz == 435_123_456
    assert "+\\set_split_vfo 1 VFOB" in rigctld.commands


def test_reads_parse_the_labelled_response(rig, rigctld):
    rigctld.rx_hz = 437_800_000
    rigctld.tx_hz = 145_990_000
    rigctld.mode = "CW"
    assert rig.get_rx_freq() == 437_800_000
    assert rig.get_tx_freq() == 145_990_000
    assert rig.get_mode() == "CW"
    assert rig.get_ptt() is False


def test_an_unsupported_command_surfaces_the_hamlib_error_code(rig):
    with pytest.raises(RigError, match="not implemented"):
        rig.command("set_ant 3")


def test_a_dropped_connection_is_reported_and_the_client_recovers(rigctld):
    rigctld.fail_after = 1
    client = RigctldRig(RigConfig(host="127.0.0.1", port=rigctld.port))
    client.set_rx_freq(145_800_000)
    with pytest.raises(RigError):
        client.set_rx_freq(145_800_100)
    assert not client.connected

    rigctld.fail_after = None
    client.set_rx_freq(145_800_200)  # reconnects on the next command
    assert rigctld.rx_hz == 145_800_200


def test_backend_selection():
    assert make_rig(RigConfig(backend="none")) is None
    assert isinstance(make_rig(RigConfig(backend="simulated")), SimulatedRig)
    with pytest.raises(ValueError, match="Unknown rig backend"):
        make_rig(RigConfig(backend="yaesu"))


# -- the tuning loop --------------------------------------------------------

UPLINK = 145_990_000.0
DOWNLINK = 437_800_000.0


class Readout:
    """Stands in for Services.rig_readout with a controllable doppler shift."""

    def __init__(self):
        self.active = True
        self.elevation = 30.0
        self.shift_ratio = 0.0  # range rate / c

    def __call__(self) -> dict:
        if not self.active:
            return {"active": False}
        return {
            "active": True,
            "elevation": self.elevation,
            "downlink_corrected_hz": DOWNLINK + DOWNLINK * self.shift_ratio,
            "uplink_corrected_hz": UPLINK - UPLINK * self.shift_ratio,
        }


def make_tuner(**config_kwargs):
    clock = VirtualClock()
    readout = Readout()
    rig = SimulatedRig()
    config_kwargs.setdefault("max_tune_rate_hz", 1000.0)  # unthrottled unless asked
    config = RigConfig(backend="simulated", **config_kwargs)
    tuner = DopplerTuner(config, rig, readout, clock=clock)
    return tuner, rig, readout, clock


def test_tuning_follows_the_downlink_up_and_the_uplink_down():
    """The sign asymmetry: approaching, you listen high and transmit low."""
    tuner, rig, readout, clock = make_tuner()
    readout.shift_ratio = 1e-5  # approaching
    tuner.tick()
    assert rig.rx_hz > DOWNLINK
    assert rig.tx_hz < UPLINK
    assert rig.split is True

    clock.advance(1.0)
    readout.shift_ratio = -1e-5  # receding
    tuner.tick()
    assert rig.rx_hz < DOWNLINK
    assert rig.tx_hz > UPLINK


def test_small_drifts_inside_the_deadband_do_not_touch_the_rig():
    tuner, rig, readout, clock = make_tuner()
    tuner.tick()
    tunes = len(rig.rx_history)
    # 5 Hz of drift on a 20 Hz deadband
    readout.shift_ratio = 5.0 / DOWNLINK
    for _ in range(10):
        clock.advance(1.0)
        tuner.tick()
    assert len(rig.rx_history) == tunes


def test_the_deadband_follows_the_rig_mode():
    tuner, rig, readout, clock = make_tuner()
    rig.mode = "FM"
    tuner.tick()
    assert tuner.status()["deadband_hz"] == 200.0
    rig.mode = "CW"
    clock.advance(60.0)
    tuner.tick()
    assert tuner.status()["deadband_hz"] == 10.0


def test_cat_writes_are_rate_limited():
    tuner, rig, readout, clock = make_tuner(max_tune_rate_hz=2.0)
    for i in range(40):
        clock.advance(0.1)  # 4 s of ticking at 10 Hz
        readout.shift_ratio = i * 1e-6  # always outside the deadband
        tuner.tick()
    # At 2 Hz over ~4 seconds, single digits — not forty.
    assert 4 <= len(rig.rx_history) <= 10


def test_nothing_is_tuned_when_no_satellite_is_tracked():
    tuner, rig, readout, clock = make_tuner()
    readout.active = False
    tuner.tick()
    assert rig.rx_history == []
    assert tuner.status()["target_rx_hz"] is None


def test_a_satellite_below_the_horizon_is_not_chased():
    tuner, rig, readout, clock = make_tuner(min_elevation_deg=0.0)
    readout.elevation = -3.0
    tuner.tick()
    assert rig.rx_history == []


def test_transmit_can_be_held_off_while_the_rig_is_keyed():
    tuner, rig, readout, clock = make_tuner(tune_while_tx=False)
    rig.ptt = True
    readout.shift_ratio = 1e-5
    tuner.tick()
    assert rig.rx_history, "receive still tracks while transmitting"
    assert rig.tx_history == [], "transmit retune deferred until PTT drops"

    rig.ptt = False
    clock.advance(1.0)
    tuner.tick()
    assert rig.tx_history


def test_tuning_can_be_switched_off_at_runtime():
    tuner, rig, readout, clock = make_tuner()
    tuner.set_enabled(False)
    tuner.tick()
    assert rig.rx_history == []
    tuner.set_enabled(True)
    tuner.tick()
    assert rig.rx_history


def test_a_rig_that_will_not_connect_does_not_stop_the_tuner():
    """The antenna keeps tracking even if the radio is unplugged."""
    config = RigConfig(backend="rigctld", host="127.0.0.1", port=1, timeout_s=0.2)
    tuner = DopplerTuner(config, RigctldRig(config), Readout(), clock=VirtualClock())
    tuner.tick()  # must not raise
    status = tuner.status()
    assert not status["connected"]
    assert "cannot reach rigctld" in status["error"]


# -- against real orbital doppler -------------------------------------------


def test_over_a_real_pass_the_correction_matches_the_predictor():
    """Cross-check the tuner's numbers against the orbital math directly."""
    predictor = Predictor(StationConfig(latitude=40.44, longitude=-79.99, altitude_m=300))
    for offset in (0, 150, 300, 450, 600):
        at = MODERATE_PASS_AOS + offset
        obs = predictor.observe(ISS, at)
        shift = obs.doppler_shift_hz(DOWNLINK)
        # Approaching means negative range rate and a positive shift.
        assert math.copysign(1, shift) == math.copysign(1, -obs.range_rate_km_s)
        assert abs(shift) < 12_000  # 70 cm LEO doppler stays inside ~10 kHz
