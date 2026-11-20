#
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""Smoke tests for optional benchmark entry points."""

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SUBPROCESS_TIMEOUT_SEC = 180  # [s]


def _run(command, env=None):
    """Run ``command`` from the repository root and require success."""

    uses_shell = isinstance(command, str)
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        shell=uses_shell,
        timeout=SUBPROCESS_TIMEOUT_SEC,
    )
    command_text = command if uses_shell else " ".join(str(item) for item in command)
    assert completed.returncode == 0, (
        f"Command failed: {command_text}\n"
        f"stdout:\n{completed.stdout}\n"
        f"stderr:\n{completed.stderr}"
    )
    return completed


def _python_environment():
    """Return a subprocess environment suitable for running benchmark scripts."""

    env = os.environ.copy()
    python_path_entries = [str(REPO_ROOT / "dist3")]
    if env.get("PYTHONPATH"):
        python_path_entries.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_path_entries)
    env.setdefault("MPLBACKEND", "Agg")
    return env


def _cmake_command():
    """Return the configured CMake command or skip the test."""

    candidate_names = ["cmake.exe"] if os.name == "nt" else ["cmake"]
    venv_bin_directory = REPO_ROOT / ".venv" / ("Scripts" if os.name == "nt" else "bin")

    for candidate_name in candidate_names:
        candidate_path = venv_bin_directory / candidate_name
        if candidate_path.exists():
            return str(candidate_path)

    cmake_path = shutil.which("cmake")
    if cmake_path is None:
        pytest.skip("CMake is not available")
    return cmake_path


def _visual_studio_dev_command():
    """Return ``VsDevCmd.bat`` if Visual Studio can be found on Windows."""

    if os.name != "nt":
        return None

    configured_path = os.environ.get("BASILISK_VSDEVCMD")
    if configured_path and Path(configured_path).exists():
        return configured_path

    program_files_x86 = os.environ.get("ProgramFiles(x86)")
    if program_files_x86 is None:
        return None

    vswhere_path = Path(program_files_x86) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    if not vswhere_path.exists():
        return None

    completed = subprocess.run(
        [
            str(vswhere_path),
            "-latest",
            "-products",
            "*",
            "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
            "-property",
            "installationPath",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=SUBPROCESS_TIMEOUT_SEC,
    )
    if completed.returncode != 0:
        return None

    install_paths = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not install_paths:
        return None

    vsdevcmd_path = Path(install_paths[0]) / "Common7" / "Tools" / "VsDevCmd.bat"
    if not vsdevcmd_path.exists():
        return None

    return str(vsdevcmd_path)


def _build_environment_command(command):
    """Return ``command`` wrapped in a compiler environment when needed."""

    if os.name != "nt":
        return command
    if os.environ.get("VCINSTALLDIR") and os.environ.get("INCLUDE"):
        return command

    vsdevcmd_path = _visual_studio_dev_command()
    if vsdevcmd_path is None:
        return command

    command_text = subprocess.list2cmdline(command)
    return f'call "{vsdevcmd_path}" -arch=amd64 -host_arch=amd64 >nul && {command_text}'


def _cmake_target_is_available(cmake_command, build_directory, target_name):
    """Return ``True`` if ``target_name`` is present in the configured build tree."""

    target_directories = build_directory / "CMakeFiles" / "TargetDirectories.txt"
    if target_directories.exists():
        target_text = target_directories.read_text(encoding="utf-8", errors="ignore")
        if target_name in target_text:
            return True

    completed = subprocess.run(
        [
            cmake_command,
            "--build",
            str(build_directory),
            "--target",
            "help",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=SUBPROCESS_TIMEOUT_SEC,
    )
    if completed.returncode != 0:
        pytest.skip("CMake target list is not available from the configured 'dist3' build directory")

    return target_name in completed.stdout or target_name in completed.stderr


def test_dynamics_benchmark_smoke():
    """Run each Python dynamics benchmark case with the smallest timing load."""

    benchmark_script = REPO_ROOT / "benchmarks" / "dynamics" / "benchmark_state_effectors.py"
    completed = _run(
        [
            sys.executable,
            str(benchmark_script),
            "--case",
            "all",
            "--steps",
            "1",
            "--trials",
            "1",
            "--warmup-steps",
            "0",
            "--components",
            "1",
            "--segments",
            "1",
        ],
        env=_python_environment(),
    )

    assert "Dynamics effector benchmark" in completed.stdout
    assert "spacecraft" in completed.stdout


def test_eigen_linear_algebra_benchmark_smoke():
    """Build and run the C++ Eigen versus linearAlgebra benchmark smoke target."""

    build_directory = REPO_ROOT / "dist3"
    if not (build_directory / "CMakeCache.txt").exists():
        pytest.skip("Basilisk CMake build directory 'dist3' is not configured")

    cmake_command = _cmake_command()
    target_name = "benchmark_smoke_tests"
    if not _cmake_target_is_available(cmake_command, build_directory, target_name):
        pytest.skip(
            "CMake build directory 'dist3' does not include benchmark targets; "
            "re-run CMake configure before running this smoke test"
        )

    completed = _run(
        _build_environment_command(
            [
                cmake_command,
                "--build",
                str(build_directory),
                "--target",
                target_name,
                "--config",
                "Release",
            ]
        )
    )

    assert "Smoke mode:" in completed.stdout
