#!/usr/bin/env python3
"""Run the amplifier supply switch's decisions against a made-up charger.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/pvdd_switch/run.py
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / "components/pvdd_switch"

# pvdd_switch_logic.c has no ESP-IDF dependencies, so it is compiled as it is.
with tempfile.TemporaryDirectory(prefix="pvdd-switch-tests-") as temporary:
    build = Path(temporary)
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(SOURCE),
        str(HERE / "test.c"), str(SOURCE / "pvdd_switch_logic.c"),
        "-o", str(build / "test"),
    ]
    print("Amplifier supply switch host tests", flush=True)
    subprocess.run(command, check=True)
    subprocess.run([str(build / "test")], check=True)
