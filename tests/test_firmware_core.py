"""Compile the firmware's protocol core on this machine and torture it.

`framing.cpp` and `queue.cpp` are written without Arduino headers on purpose
(PLAN-ARDUINO §5), so the parts of the sketch that are easy to get subtly wrong
— CRCs, byte order, resynchronisation, ring wraparound — can be tested on a
host with a real compiler rather than on a board with two LEDs.

Skips when no C++ compiler is installed, the same way `test_gpiosim.py` skips
without the kernel's gpio-sim and `test_geo_js.py` skips without node: a missing
tool is not a failure, but it must be *visible* rather than silent.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

FIRMWARE = Path(__file__).parent.parent / "firmware"
SOURCES = [
    FIRMWARE / "test" / "test_firmware_core.cpp",
    FIRMWARE / "orbitaly_rotator" / "framing.cpp",
    FIRMWARE / "orbitaly_rotator" / "queue.cpp",
]
GOLDEN = FIRMWARE / "test" / "golden_frames.txt"


def find_compiler() -> str | None:
    for candidate in ("c++", "g++", "clang++"):
        if shutil.which(candidate):
            return candidate
    return None


needs_cxx = pytest.mark.skipif(
    find_compiler() is None,
    reason="no host C++ compiler (c++/g++/clang++) — install one to check the firmware core",
)


@needs_cxx
def test_the_firmware_protocol_core_passes_its_own_checks(tmp_path):
    compiler = find_compiler()
    binary = tmp_path / "test_firmware_core"
    build = subprocess.run(
        [
            compiler,
            # The Arduino AVR core is gcc 7-era and compiles at gnu++11. Building
            # to the same standard here is what stops a C++14/17 construct
            # passing on the host and failing on the board.
            "-std=c++11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-O1",
            f"-I{FIRMWARE / 'orbitaly_rotator'}",
            *[str(path) for path in SOURCES],
            "-o",
            str(binary),
        ],
        capture_output=True,
        text=True,
        timeout=180,
    )
    if build.returncode != 0:
        pytest.fail("firmware core did not compile:\n" + build.stdout + build.stderr)

    result = subprocess.run(
        [str(binary), str(GOLDEN)], capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        pytest.fail("firmware core checks failed:\n" + result.stdout + result.stderr)

    # Guard the guard: a binary that ran nothing would otherwise pass forever.
    assert "checks passed" in result.stdout
    count = int(result.stdout.strip().rsplit("\n", 1)[-1].split()[0])
    assert count >= 100, f"expected the full check set to run, saw {count}"


@needs_cxx
def test_the_firmware_core_compiles_without_arduino_headers(tmp_path):
    """The property the host test depends on, asserted directly.

    If somebody adds `#include <Arduino.h>` to framing.cpp or queue.cpp for
    convenience, the tests above stop being runnable anywhere except a board —
    and the failure would look like "compiler missing", not like a mistake.
    """
    for source in (FIRMWARE / "orbitaly_rotator" / "framing.cpp", FIRMWARE / "orbitaly_rotator" / "queue.cpp"):
        assert "Arduino.h" not in source.read_text(), f"{source.name} grew an Arduino dependency"
