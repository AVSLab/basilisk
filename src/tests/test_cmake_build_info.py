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

"""Exercise build metadata regeneration through native incremental builds."""

import os
import runpy
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENT_CMAKE = Path(sys.executable).with_name("cmake")
CMAKE = shutil.which("cmake") or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)
ENVIRONMENT_NINJA = Path(sys.executable).with_name("ninja")
NINJA = shutil.which("ninja") or (
    str(ENVIRONMENT_NINJA) if ENVIRONMENT_NINJA.is_file() else None
)
GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and os.name != "nt":
    GENERATORS.append("Ninja")


def _run(command):
    """Run a fixture command and expose its diagnostics on failure.

    :param command: Executable and command-line arguments.
    """
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("generator", GENERATORS)
@pytest.mark.parametrize("cached_version", [None, "2.12.0"])
def test_incremental_build_refreshes_source_version(tmp_path, generator, cached_version):
    """Refresh metadata after version edits without a manual configure step.

    :param tmp_path: Temporary directory supplied by pytest.
    :param generator: Native build generator to exercise.
    :param cached_version: Old Conan version retained in the CMake cache, if any.
    """
    project = tmp_path / "src"
    build = tmp_path / "build"
    # Preserve the production layout while changing only the fixture version.
    for relative_path in (
        "src/cmake/bskBuildInfo.cmake",
        "src/cmake/bskBuildInfoData.py.in",
        "src/cmake/bskBuildInfo.py",
        "src/cmake/bskBuildInfoProbe.c",
        "src/cmake/bskBuildInfoProbe.cpp",
        "src/architecture/utilities/bskAbiDescriptor.h",
    ):
        destination = tmp_path / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(REPOSITORY_ROOT / relative_path, destination)
    version_file = tmp_path / "docs/source/bskVersion.txt"
    version_file.parent.mkdir(parents=True)
    version_file.write_text("2.12.0\n", encoding="utf-8")
    (project / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.26)
project(buildInfoVersion C CXX)
include(cmake/bskBuildInfo.cmake)
# Only configure the ABI probe; this test exercises the generated Python metadata.
add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
bsk_generate_build_info("${CMAKE_BINARY_DIR}/Basilisk")
add_custom_target(buildInfoMetadata DEPENDS "${CMAKE_BINARY_DIR}/Basilisk/_buildInfoData.py")
file(APPEND "${CMAKE_BINARY_DIR}/configure-runs.txt" "configured\\n")
""",
        encoding="utf-8",
    )
    command = [CMAKE, "-S", str(project), "-B", str(build)]
    if generator:
        command.extend(["-G", generator])
    if generator == "Ninja":
        command.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    if cached_version:
        command.append(f"-DBSK_VERSION={cached_version}")
    if sys.platform == "darwin" and not os.environ.get("SDKROOT"):
        command.append("-DCMAKE_OSX_SYSROOT=macosx")
    _run(command)

    metadata_file = build / "Basilisk/_buildInfoData.py"
    configure_runs = build / "configure-runs.txt"
    build_command = [CMAKE, "--build", str(build), "--target", "buildInfoMetadata"]
    _run(build_command)
    metadata = runpy.run_path(str(metadata_file))["buildInfoData"]
    assert metadata["artifact"]["basiliskVersion"] == "2.12.0"

    for version_text, expected_version in (
        ("2.13.0b0\n", "2.13.0b0"),
        ("2.13.0\n", "2.13.0"),
        # Preserve the surrounding-whitespace normalization formerly done by Conan.
        (" \t2.13.1b0\t \n", "2.13.1b0"),
        ("\n2.13.2b0\n", "2.13.2b0"),
        (" \t\n2.13.3b0\n", "2.13.3b0"),
    ):
        previous_runs = configure_runs.read_text(encoding="utf-8")
        version_file.write_text(version_text, encoding="utf-8")
        _run(build_command)
        metadata = runpy.run_path(str(metadata_file))["buildInfoData"]
        assert metadata["artifact"]["basiliskVersion"] == expected_version
        assert configure_runs.read_text(encoding="utf-8") == previous_runs + "configured\n"

    # An unchanged build should neither reconfigure nor rewrite the metadata.
    unchanged_runs = configure_runs.read_text(encoding="utf-8")
    metadata_time = metadata_file.stat().st_mtime_ns
    _run(build_command)
    assert configure_runs.read_text(encoding="utf-8") == unchanged_runs
    assert metadata_file.stat().st_mtime_ns == metadata_time
