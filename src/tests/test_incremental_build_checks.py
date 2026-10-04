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

"""Exercise the incremental-build checks with native CMake generators."""

import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CHECKS = runpy.run_path(str(REPOSITORY_ROOT / ".github/scripts/test_incremental_build.py"))
SUBPROCESS_TIMEOUT_SECONDS = 180  # [s]


def find_build_tool(name):
    """Find a build tool on PATH or beside the active Python executable."""
    executable = name + (".exe" if os.name == "nt" else "")
    local = Path(sys.executable).with_name(executable)
    return shutil.which(executable) or (str(local) if local.is_file() else None)


CMAKE = find_build_tool("cmake")
NINJA = find_build_tool("ninja")
BUILD_GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and (os.name != "nt" or os.environ.get("VCINSTALLDIR")):
    BUILD_GENERATORS.append("Ninja")
if sys.platform == "darwin":
    BUILD_GENERATORS.append("Xcode")


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("generator", BUILD_GENERATORS)
def test_package_configuration_checks_native_targets(tmp_path, monkeypatch, generator):
    """Accept header-only packages and detect accidental binary targets.

    :param tmp_path: Temporary directory supplied by pytest.
    :param monkeypatch: Pytest helper for selecting the CMake executable.
    :param generator: Native CMake generator to exercise.
    """
    if generator == "Xcode":
        xcodebuild = shutil.which("xcodebuild")
        if xcodebuild is None:
            pytest.skip("Xcode is not installed")
        version = subprocess.run(
            [xcodebuild, "-version"], capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        if version.returncode:
            pytest.skip("A full Xcode installation must be selected to test its generator")

    project = tmp_path / "source"
    build = tmp_path / "build"
    (project / "cmake").mkdir(parents=True)
    for helper in ("bskCollectWrapperCustomFiles.cmake", "bskSourceInventory.cmake"):
        shutil.copyfile(REPOSITORY_ROOT / "src/cmake" / helper, project / "cmake" / helper)
    (project / "fixture.c").write_text("int fixture(void) { return 0; }\n", encoding="utf-8")
    declarations = [
        "cmake_minimum_required(VERSION 3.26)",
        "project(packageCheckFixture LANGUAGES C)",
        # A similar name must not be mistaken for a header-only package.
        "add_library(communicationLibHelper STATIC fixture.c)",
    ]
    declarations.append('''foreach(package communicationLib vizardLib)
  if(package STREQUAL BINARY_PACKAGE)
    add_library(${package} STATIC fixture.c)
  else()
    add_library(${package} INTERFACE)
  endif()
endforeach()''')
    (project / "CMakeLists.txt").write_text("\n".join(declarations) + "\n", encoding="utf-8")
    command = [CMAKE, "-S", str(project), "-B", str(build)]
    if generator:
        command.extend(["-G", generator])
    if generator == "Ninja":
        command.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    check = CHECKS["assert_package_configuration"]
    monkeypatch.setitem(check.__globals__, "cmake_executable", lambda: CMAKE)
    # Reuse compiler detection while exercising each actual File API target set.
    for binary_package in ("", "communicationLib", "vizardLib", ""):
        result = subprocess.run(
            command + [f"-DBINARY_PACKAGE={binary_package}"], capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        if binary_package:
            with pytest.raises(CHECKS["IncrementalBuildError"], match=f"binary library targets: {binary_package}"):
                check(project, build)
        else:
            check(project, build)
