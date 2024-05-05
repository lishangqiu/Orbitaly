"""The rotctld-compatible server: wire protocol and rotator integration."""
import asyncio
import shutil
import subprocess

import pytest

from orbitaly.config import RotatorConfig, RotctldConfig
from orbitaly.core.tracker import map_into_travel
from orbitaly.motion.backend import SimulatedBackend
from orbitaly.motion.errors import MotionBlocked
from orbitaly.motion.rotator import StepperRotator
from orbitaly.net.rotctld_server import RotctldServer


class FakeTracker:
    """Records what the server asked the tracker to do."""

    def __init__(self, rotator, blocked: str = ""):
        self.rotator = rotator
        self.blocked = blocked
        self.calls = []

    def _guard(self):
        if self.blocked:
            raise MotionBlocked(self.blocked)

    def manual_goto(self, az, el):
        self._guard()
        self.calls.append(("goto", az, el))
        self.rotator.goto(az, el)

    def manual_jog(self, d_az, d_el):
        self._guard()
        self.calls.append(("jog", d_az, d_el))

    def manual_stop(self):
        self.calls.append(("stop",))

    def park(self):
        self._guard()
        self.calls.append(("park",))


class FakeServices:
    def __init__(self, blocked: str = ""):
        config = RotatorConfig(backend="simulated", watchdog_s=0.0)
        self.rotator = StepperRotator(config, SimulatedBackend(), require_homing=False)
        self.tracker = FakeTracker(self.rotator, blocked)


@pytest.fixture
def server():
    return RotctldServer(FakeServices(), RotctldConfig(port=0))


# -- default protocol (what hamlib's own client speaks) ---------------------


def test_set_position_acknowledges_with_rprt_zero(server):
    assert server.dispatch("P 163.0 41.0") == "RPRT 0\n"
    assert server.services.tracker.calls == [("goto", 163.0, 41.0)]


def test_get_position_returns_two_bare_lines(server):
    server.services.rotator.az._origin_steps = int(round(120.0 * server.services.rotator.az._spd))
    server.services.rotator.el._origin_steps = int(round(30.0 * server.services.rotator.el._spd))
    lines = server.dispatch("p").splitlines()
    assert len(lines) == 2
    # Positions are whole steps, so they land within a step of the request.
    assert float(lines[0]) == pytest.approx(120.0, abs=0.01)
    assert float(lines[1]) == pytest.approx(30.0, abs=0.01)


def test_stop_park_and_reset_are_wired_through(server):
    assert server.dispatch("S") == "RPRT 0\n"
    assert server.dispatch("K") == "RPRT 0\n"
    assert server.dispatch("R 1") == "RPRT 0\n"
    assert ("stop",) in server.services.tracker.calls
    assert ("park",) in server.services.tracker.calls


def test_move_becomes_a_jog(server):
    assert server.dispatch("M 16 50") == "RPRT 0\n"  # right, half speed
    kind, d_az, d_el = server.services.tracker.calls[-1]
    assert kind == "jog" and d_az > 0 and d_el == 0


def test_unknown_commands_report_not_implemented(server):
    assert server.dispatch("Z") == "RPRT -4\n"
    assert server.dispatch("\\set_level 3") == "RPRT -4\n"


def test_malformed_arguments_report_invalid(server):
    assert server.dispatch("P north up") == "RPRT -1\n"
    assert server.dispatch("P 10") == "RPRT -1\n"


def test_long_command_names_work(server):
    assert server.dispatch("\\set_pos 90 45") == "RPRT 0\n"
    assert server.dispatch("\\get_pos").count("\n") == 2


def test_terse_clients_may_omit_the_space(server):
    assert server.dispatch("P90 45") == "RPRT 0\n"
    assert server.services.tracker.calls[-1] == ("goto", 90.0, 45.0)


# -- extended protocol ------------------------------------------------------


def test_extended_protocol_echoes_and_labels(server):
    assert server.dispatch("+P 90 45") == "set_pos: 90 45\nRPRT 0\n"


def test_extended_get_position_is_labelled(server):
    response = server.dispatch("+p")
    assert response.startswith("get_pos: \n")
    assert "Azimuth: " in response and "Elevation: " in response
    assert response.endswith("RPRT 0\n")


def test_alternate_separators_put_it_on_one_line(server):
    response = server.dispatch(";p")
    assert "\n" not in response
    assert response.endswith("RPRT 0;")


# -- integration with the real rotator --------------------------------------


def test_azimuth_is_mapped_into_extended_travel_the_short_way():
    """A client sending 350 must not send a -90..450 mount the long way round."""
    assert map_into_travel(350.0, 0.0, -90.0, 450.0) == pytest.approx(-10.0)
    assert map_into_travel(350.0, 340.0, -90.0, 450.0) == pytest.approx(350.0)
    assert map_into_travel(10.0, 400.0, -90.0, 450.0) == pytest.approx(370.0)
    # A plain 0..360 mount has only one option.
    assert map_into_travel(350.0, 0.0, 0.0, 360.0) == pytest.approx(350.0)


def test_positions_are_reported_as_compass_bearings(server):
    rotator = server.services.rotator
    rotator.az._origin_steps = int(round(-30.0 * rotator.az._spd))
    assert server.dispatch("p").startswith("330.000000")


def test_an_interlock_refusal_is_reported_not_swallowed():
    server = RotctldServer(FakeServices(blocked="not homed"), RotctldConfig(port=0))
    assert server.dispatch("P 90 45").startswith("RPRT -")
    assert server.services.tracker.calls == []


def test_dump_state_reports_the_travel_limits(server):
    response = server.dispatch("\\dump_state")
    assert response.splitlines() == ["0", "2", "-90.000000", "450.000000", "0.000000", "180.000000"]


def test_dump_caps_describes_an_azel_rotator(server):
    response = server.dispatch("+1")
    assert "Rotator type: AzEl" in response
    assert "Max Azimuth: 450.00" in response


# -- over a real socket -----------------------------------------------------


def test_a_real_client_can_drive_it_over_tcp():
    async def scenario():
        services = FakeServices()
        server = RotctldServer(services, RotctldConfig(host="127.0.0.1", port=0))
        await server.start()
        reader, writer = await asyncio.open_connection("127.0.0.1", server.port)
        try:
            writer.write(b"P 200.0 33.0\n")
            await writer.drain()
            assert await reader.readline() == b"RPRT 0\n"

            writer.write(b"p\n")
            await writer.drain()
            azimuth = float(await reader.readline())
            elevation = float(await reader.readline())
            assert 0.0 <= azimuth < 360.0 and elevation >= 0.0

            writer.write(b"q\n")
            await writer.drain()
        finally:
            writer.close()
            await server.stop()
            services.rotator.close()

    asyncio.run(scenario())


@pytest.mark.skipif(shutil.which("rotctl") is None, reason="hamlib's rotctl is not installed")
def test_hamlibs_own_client_can_talk_to_us():
    """The one check that validates dump_state's field order for real."""

    async def scenario():
        services = FakeServices()
        server = RotctldServer(services, RotctldConfig(host="127.0.0.1", port=0))
        await server.start()
        try:
            result = await asyncio.to_thread(
                subprocess.run,
                ["rotctl", "-m", "2", "-r", f"127.0.0.1:{server.port}", "get_pos"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            assert result.returncode == 0, result.stderr
            assert result.stdout.strip(), "rotctl returned no position"
        finally:
            await server.stop()
            services.rotator.close()

    asyncio.run(scenario())
