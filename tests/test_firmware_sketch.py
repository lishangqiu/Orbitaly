"""Rules the sketch has to keep, and a compile gate when the toolchain exists.

The sketch is the one part of this system that cannot be exercised by the test
suite: it runs on an ATmega, its correctness depends on timer registers, and no
amount of Python proves it toggles a pin. What *can* be checked here is that it
stays the kind of program it is supposed to be — small, allocation-free, and
free of the trajectory intelligence that belongs on the Pi.

These are cheap, and they guard against the realistic failure: not somebody
writing bad AVR code, but somebody moving a decision onto the microcontroller
because it seemed convenient there.
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

SKETCH_DIR = Path(__file__).parent.parent / "firmware" / "orbitaly_rotator"
SKETCH = SKETCH_DIR / "orbitaly_rotator.ino"
SOURCES = sorted(SKETCH_DIR.glob("*.cpp")) + sorted(SKETCH_DIR.glob("*.h")) + [SKETCH]

#: Arduino-isms that mean unbounded RAM use on a 2 KB part. `String` in
#: particular fragments the heap until the board resets mid-pass, which looks
#: exactly like a brownout and is a genuinely miserable thing to debug.
BANNED = {
    "String": r"\bString\b",
    "malloc": r"\bmalloc\s*\(",
    "calloc": r"\bcalloc\s*\(",
    "realloc": r"\brealloc\s*\(",
    "new": r"\bnew\s+[A-Za-z_]",
    "std::": r"\bstd::",
}


def code_only(text: str) -> str:
    """Strip comments, so these rules police the code and not the prose.

    These files explain themselves at length — including, necessarily, what the
    firmware must *not* do — so scanning raw text would flag the very sentences
    that document the rule.
    """
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return "\n".join(re.sub(r"//.*$", "", line) for line in text.splitlines())


def sources_text() -> dict[Path, str]:
    return {path: code_only(path.read_text()) for path in SOURCES}


@pytest.mark.parametrize("name,pattern", sorted(BANNED.items()))
def test_the_firmware_allocates_nothing_dynamically(name, pattern):
    offenders = [
        path.name for path, text in sources_text().items() if re.search(pattern, text)
    ]
    assert offenders == [], f"{name} appears in {offenders}: the AVR has 2 KB of SRAM"


def test_the_firmware_holds_no_opinion_about_where_the_antenna_should_point():
    """The distinction from K3NG-style firmware, asserted rather than asserted-in-prose.

    Degrees, gear ratios, travel limits, acceleration and speed are host
    concepts. If any of them turn up here, the architecture has quietly become
    the position-target one this design exists to reject.
    """
    forbidden = (
        "gear_ratio",
        "gearRatio",
        "steps_per_deg",
        "stepsPerDeg",
        "accel",
        "max_speed",
        "maxSpeed",
        "backlash",
        "degrees",
    )
    for path, text in sources_text().items():
        for word in forbidden:
            assert word not in text, f"{path.name} mentions {word!r} outside a comment"


def test_the_sketch_uses_no_floating_point():
    """No FPU. A float multiply in a step ISR at 15 kHz is not affordable, and
    the arithmetic here is all integer by design."""
    for path, text in sources_text().items():
        assert not re.search(r"\b(float|double)\b", text), f"{path.name} uses floating point"


def test_the_protocol_core_stays_compilable_off_the_board():
    """framing.cpp and queue.cpp must not grow Arduino dependencies, or the host
    test in test_firmware_core.py silently stops testing anything."""
    for name in ("framing.cpp", "framing.h", "queue.cpp", "queue.h", "protocol.h"):
        text = (SKETCH_DIR / name).read_text()
        assert "Arduino.h" not in text, f"{name} grew an Arduino dependency"
        assert "Serial." not in text, f"{name} reaches for the UART"


def test_only_the_step_generator_touches_timers():
    """Timer registers in two places is how you get an axis that stops when the
    other one starts."""
    for path, text in sources_text().items():
        if path.name.startswith("stepgen"):
            continue
        assert not re.search(r"\bTCCR[12]|OCR[12]A|TIMSK[12]\b", text), (
            f"{path.name} pokes a timer register; that belongs in stepgen.cpp"
        )


def test_the_sketch_stays_small_enough_to_review_in_one_sitting():
    """PLAN-ARDUINO §3 puts a ~500-line target on the sketch, on the grounds
    that firmware nobody reads is firmware nobody trusts."""
    code_lines = [
        line
        for line in SKETCH.read_text().splitlines()
        if line.strip() and not line.strip().startswith("//")
    ]
    assert len(code_lines) < 500, f"the sketch is {len(code_lines)} lines of code"


def test_every_pin_the_handshake_reports_is_defined():
    pins_h = (SKETCH_DIR / "pins.h").read_text()
    for name in (
        "AZ_STEP_PIN",
        "AZ_DIR_PIN",
        "AZ_ENABLE_PIN",
        "AZ_ENDSTOP_PIN",
        "EL_STEP_PIN",
        "EL_DIR_PIN",
        "EL_ENABLE_PIN",
        "EL_ENDSTOP_PIN",
        "ESTOP_PIN",
    ):
        assert re.search(rf"^#define\s+{name}\s", pins_h, re.M), f"{name} is missing from pins.h"


def test_the_firmware_version_matches_what_the_fake_claims():
    """The fake firmware stands in for this sketch in every host test; a version
    skew between them would make those tests describe a board that does not
    exist."""
    pins_h = (SKETCH_DIR / "pins.h").read_text()
    version = tuple(
        int(re.search(rf"#define\s+FW_VERSION_{part}\s+(\d+)", pins_h).group(1))
        for part in ("MAJOR", "MINOR", "PATCH")
    )
    from fakes.fake_firmware import FW_VERSION

    assert version == FW_VERSION


def test_the_firmware_timing_constants_match_the_fake():
    pins_h = (SKETCH_DIR / "pins.h").read_text()

    def define(name: str) -> int:
        return int(re.search(rf"#define\s+{name}\s+(\d+)", pins_h).group(1))

    from fakes import fake_firmware as fake

    assert define("DIR_SETUP_US") == fake.DIR_SETUP_US
    assert define("PULSE_WIDTH_US") == fake.PULSE_WIDTH_US
    assert define("ENDSTOP_DEBOUNCE_MS") == fake.ENDSTOP_DEBOUNCE_MS
    assert define("FW_WATCHDOG_MS") / 1000.0 == fake.FW_WATCHDOG_S
    assert define("FW_IDLE_DISABLE_MS") / 1000.0 == fake.IDLE_DISABLE_S


# --------------------------------------------------------------------------
# The real compile gate
# --------------------------------------------------------------------------

needs_arduino_cli = pytest.mark.skipif(
    shutil.which("arduino-cli") is None,
    reason="arduino-cli is not installed — cannot compile the sketch here",
)


@needs_arduino_cli
def test_the_sketch_compiles_for_an_uno(tmp_path):
    """The only check that proves the AVR half is real code.

    Skips loudly rather than pretending, exactly like the gpiosim and node
    tests. Until this has run somewhere, the sketch is reviewed but unbuilt —
    and HANDOFF's "not verified" list says so.
    """
    result = subprocess.run(
        [
            "arduino-cli",
            "compile",
            "--fqbn",
            "arduino:avr:uno",
            "--build-path",
            str(tmp_path / "build"),
            str(SKETCH_DIR),
        ],
        capture_output=True,
        text=True,
        timeout=600,
    )
    if result.returncode != 0:
        pytest.fail("sketch did not compile:\n" + result.stdout + result.stderr)
