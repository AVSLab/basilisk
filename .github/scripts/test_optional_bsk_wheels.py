#!/usr/bin/env python3
"""Test optional Basilisk wheels in isolated virtual environments.

 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import venv
from pathlib import Path


CORE_IMPORTS = [
    "Basilisk.architecture.messaging",
    "Basilisk.utilities.SimulationBaseClass",
    "Basilisk.simulation.spacecraft",
    "Basilisk.simulation.vizInterface",
    "Basilisk.fswAlgorithms.mrpFeedback",
    "Basilisk.simulation.mujoco",
    "Basilisk.simulation.thrOnTimeToForce",
    "Basilisk.moduleTemplates.rustModuleTemplate",
]
OPNAV_IMPORTS = [
    "Basilisk.simulation.camera",
    "Basilisk.fswAlgorithms.centerRadiusCNN",
    "Basilisk.fswAlgorithms.houghCircles",
    "Basilisk.fswAlgorithms.limbFinding",
]


def venv_python(venv_path: Path) -> Path:
    if os.name == "nt":
        return venv_path / "Scripts/python.exe"
    return venv_path / "bin/python"


def run(command: list[str | Path], *, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(str(part) for part in command), flush=True)
    subprocess.run([str(part) for part in command], check=True, env=env)


def find_one_wheel(wheelhouse: Path, pattern: str) -> Path:
    matches = sorted(wheelhouse.glob(pattern))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one wheel matching {pattern!r}, found {len(matches)}.")
    return matches[0]


def create_venv(parent: Path, name: str) -> Path:
    venv_path = parent / name
    shutil.rmtree(venv_path, ignore_errors=True)
    venv.EnvBuilder(with_pip=True).create(venv_path)
    return venv_path


def import_check_script(required: list[str], missing: list[str]) -> str:
    return f"""
import importlib
import importlib.util
import sys

required = {required!r}
missing = {missing!r}

import Basilisk
print("Basilisk:", Basilisk.__file__)

build_info = Basilisk.getBuildInfo()
build_options = build_info["diagnostics"]["build"]
if not build_options["rustModules"]:
    raise SystemExit("wheel was not built with Rust modules")
if not build_options["rustCorrosion"]:
    raise SystemExit("wheel was not built with Corrosion")

for name in required:
    module = importlib.import_module(name)
    print("OK import", name, "->", getattr(module, "__file__", "<builtin>"))

rust_module_api = sys.modules["Basilisk.moduleTemplates.rustModuleTemplate"]
rust_module = rust_module_api.rustModuleTemplate()
print("OK construct Rust module", type(rust_module).__name__)

for name in missing:
    if importlib.util.find_spec(name) is not None:
        raise SystemExit(f"unexpected optional module present: {{name}}")
    print("EXPECTED missing", name)
"""


def run_import_check(
    python: Path,
    *,
    required: list[str],
    missing: list[str],
    env: dict[str, str],
) -> None:
    run([python, "-c", import_check_script(required, missing)], env=env)


def install_extra(python: Path, wheelhouse: Path, extra: str) -> None:
    run([
        python,
        "-m",
        "pip",
        "install",
        "--no-index",
        "--find-links",
        wheelhouse,
        f"bsk[{extra}]",
    ])


def test_optional_wheels(wheelhouse: Path) -> None:
    base_wheel = find_one_wheel(wheelhouse, "bsk-*.whl")
    opnav_wheel = find_one_wheel(wheelhouse, "bsk_opnav-*.whl")
    print(f"Testing base wheel: {base_wheel}")
    print(f"Testing opNav wheel: {opnav_wheel}")

    with tempfile.TemporaryDirectory(prefix="bsk-optional-wheel-test-") as tmp_dir_name:
        tmp_dir = Path(tmp_dir_name)
        test_env = os.environ.copy()
        test_env["MPLBACKEND"] = "Agg"
        test_env["MPLCONFIGDIR"] = str(tmp_dir / "matplotlib")

        venv_path = create_venv(tmp_dir, "install")
        python = venv_python(venv_path)

        run([python, "-m", "pip", "install", base_wheel])
        run_import_check(
            python,
            required=CORE_IMPORTS,
            missing=OPNAV_IMPORTS,
            env=test_env,
        )

        install_extra(python, wheelhouse, "opnav")
        run([python, "-m", "pip", "show", "bsk", "bsk-opnav"])
        run_import_check(
            python,
            required=CORE_IMPORTS + OPNAV_IMPORTS,
            missing=[],
            env=test_env,
        )

        run([python, "-m", "pip", "uninstall", "-y", "bsk-opnav"])
        run_import_check(
            python,
            required=CORE_IMPORTS,
            missing=OPNAV_IMPORTS,
            env=test_env,
        )

        install_extra(python, wheelhouse, "all")
        run([python, "-m", "pip", "show", "bsk", "bsk-opnav"])
        run_import_check(
            python,
            required=CORE_IMPORTS + OPNAV_IMPORTS,
            missing=[],
            env=test_env,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test optional Basilisk wheels.")
    parser.add_argument("--wheelhouse", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        test_optional_wheels(args.wheelhouse)
    except Exception as err:
        print(f"error: {err}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
