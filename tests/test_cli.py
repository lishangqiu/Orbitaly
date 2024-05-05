"""CLI surface: subcommand parsing and the doctor report."""
import pytest

from orbitaly.__main__ import build_parser, normalise_argv
from orbitaly.cli import doctor


def parse(argv):
    return build_parser().parse_args(normalise_argv(argv))


def test_bare_invocation_still_serves():
    """`orbitaly` and `orbitaly -c file` predate the subcommands."""
    assert parse([]).command == "serve"
    args = parse(["-c", "config.yaml"])
    assert args.command == "serve" and args.config == "config.yaml"


def test_subcommand_may_come_before_or_after_the_options():
    for argv in (["doctor", "-c", "x.yaml"], ["-c", "x.yaml", "doctor"]):
        args = parse(argv)
        assert args.command == "doctor" and args.config == "x.yaml"


def test_a_config_file_named_like_a_subcommand_is_not_mistaken_for_one():
    args = parse(["-c", "doctor"])
    assert args.command == "serve" and args.config == "doctor"


def test_serve_options_survive():
    args = parse(["serve", "--port", "9000", "--host", "127.0.0.1"])
    assert args.port == 9000 and args.host == "127.0.0.1"


def test_selftest_requires_an_explicit_loopback_confirmation():
    with pytest.raises(SystemExit):
        parse(["selftest", "--pin", "6"])  # no --loopback
    args = parse(["selftest", "--loopback", "--pin", "6", "--axis", "el"])
    assert args.pin == 6 and args.axis == "el"


def test_doctor_reports_the_backend_it_would_choose(capsys, tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("rotator:\n  backend: simulated\n")
    assert doctor.run(str(config)) == 0
    output = capsys.readouterr().out
    assert "Rotator backend: simulated -> simulated" in output
    assert "steps/deg" in output


def test_doctor_flags_pins_that_clash_with_each_other(capsys, tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text(
        "rotator:\n"
        "  azimuth:\n    pins: {step: 17, dir: 27}\n"
        "  elevation:\n    pins: {step: 17, dir: 24}\n"
    )
    doctor.run(str(config))
    assert "assigned to both" in capsys.readouterr().out


def test_doctor_warns_about_pins_the_spi_overlay_owns(capsys, tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("rotator:\n  azimuth:\n    pins: {step: 10, dir: 9}\n")
    doctor.run(str(config))
    output = capsys.readouterr().out
    assert "spi0" in output


def test_doctor_warns_when_azimuth_travel_cannot_clear_north(capsys, tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("rotator:\n  azimuth: {min_deg: 0, max_deg: 360}\n")
    doctor.run(str(config))
    assert "wrap slew" in capsys.readouterr().out
