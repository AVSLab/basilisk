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

"""Verify concise CMake summaries and support-file links without a full build."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CMAKE_NAME = "cmake.exe" if os.name == "nt" else "cmake"
ENVIRONMENT_CMAKE = Path(sys.executable).with_name(CMAKE_NAME)
CMAKE = shutil.which(CMAKE_NAME) or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)
pytestmark = pytest.mark.skipif(CMAKE is None, reason="CMake is required")


def run_script(tmp_path, content, log_level="NOTICE"):
    """Execute the production CMake helpers in an isolated script."""
    script = tmp_path / "check.cmake"
    script.write_text("cmake_minimum_required(VERSION 3.26)\n" + content, encoding="utf-8")
    result = subprocess.run(
        [CMAKE, f"--log-level={log_level}", "-P", str(script)], cwd=tmp_path,
        capture_output=True, text=True, check=False, timeout=30,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout + result.stderr


@pytest.mark.parametrize("multi_config", [False, True])
@pytest.mark.parametrize("features_enabled", [False, True])
def test_build_summary_reports_selected_configuration(tmp_path, multi_config, features_enabled):
    """Show the actual tools and feature state in both normal and verbose modes."""
    module = (REPOSITORY_ROOT / "src/cmake/bskBuildInfo.cmake").as_posix()
    state = "ON" if features_enabled else "OFF"
    content = f'''include("{module}")
set(CMAKE_C_COMPILER_ID CCompiler)
set(CMAKE_C_COMPILER_VERSION 1.2)
set(CMAKE_CXX_COMPILER_ID CxxCompiler)
set(CMAKE_CXX_COMPILER_VERSION 3.4)
set(CMAKE_C_COMPILER "/tools/cc")
set(CMAKE_CXX_COMPILER "/tools/cxx")
set(CMAKE_GENERATOR "Fixture Generator")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES "{'Debug;Release' if multi_config else ''}")
set(BSK_CONAN_BUILD_TYPE Release)
set(Python3_VERSION 3.14.0)
set(SWIG_VERSION 4.5.0)
foreach(option BUILD_VIZINTERFACE BUILD_OPNAV BUILD_MUJOCO BUILD_RUST_MODULES BSK_STRICT_WARNINGS)
  set(${{option}} {state})
endforeach()
bsk_print_build_summary("1.97.0")
'''
    for verbose in (False, True):
        output = run_script(tmp_path, content, "VERBOSE" if verbose else "NOTICE")
        configuration = "multi-config (Conan profile: Release)" if multi_config else "Release"
        assert f"Basilisk configuration: {configuration} / Fixture Generator" in output
        assert "C: CCompiler 1.2 | C++: CxxCompiler 3.4" in output
        assert "Python: 3.14.0 | SWIG: 4.5.0" in output
        assert ("Rust: 1.97.0" in output) == features_enabled
        assert f"Features: Vizard={state}, OpNav={state}, MuJoCo={state}, Rust={state}" in output
        assert f"Strict compiler warnings: {state}" in output
        assert f"Build directory: {tmp_path.as_posix()}" in output
        assert ("C compiler: /tools/cc" in output) == verbose
        assert ("C++ compiler: /tools/cxx" in output) == verbose


@pytest.mark.parametrize("force_link_failure", [False, True])
def test_support_links_ignore_metadata_and_create_destination(tmp_path, force_link_failure):
    """Link real files on clean and cached runs, retaining warnings for copy fallbacks."""
    cmake_source = (REPOSITORY_ROOT / "src/CMakeLists.txt").read_text(encoding="utf-8")
    helper = re.search(
        r"^function\(create_symlinks destinationPath\).*?^endfunction\(create_symlinks\)",
        cmake_source, re.MULTILINE | re.DOTALL,
    ).group()
    source = tmp_path / "support files"
    destination = tmp_path / "new build" / "utilities"
    source.mkdir()
    (source / ".DS_Store").write_text("Finder metadata", encoding="utf-8")
    (source / "sample.py").write_text("value = 1\n", encoding="utf-8")
    (source / "package").mkdir()
    (source / "package/__init__.py").write_text("", encoding="utf-8")
    content = helper + f'''
set(CMAKE_CURRENT_SOURCE_DIR "{source.as_posix()}")
set(CMAKE_CURRENT_BINARY_DIR "{destination.as_posix()}")
'''
    if force_link_failure:
        # Simulate a platform that cannot create symlinks, without changing permissions.
        content += '''function(execute_process)
  cmake_parse_arguments(process "" "RESULT_VARIABLE;ERROR_VARIABLE" "COMMAND" ${ARGN})
  set(${process_RESULT_VARIABLE} 1 PARENT_SCOPE)
  set(${process_ERROR_VARIABLE} "Fixture link failure" PARENT_SCOPE)
endfunction()
'''
    content += f'''file(GLOB support_files "{source.as_posix()}/*")
create_symlinks("{destination.as_posix()}" ${{support_files}})
'''
    for cached in (False, True):
        if cached:
            # A previous build may have copied metadata into the output directory.
            (destination / ".DS_Store").write_text("stale metadata", encoding="utf-8")
        output = run_script(tmp_path, content)
        assert (source / ".DS_Store").read_text(encoding="utf-8") == "Finder metadata"
        assert not (destination / ".DS_Store").exists()
        assert not (destination / ".DS_Store").is_symlink()
        assert (destination / "sample.py").read_text(encoding="utf-8") == "value = 1\n"
        assert (destination / "package/__init__.py").is_file()
        assert ".DS_Store" not in output
        if force_link_failure:
            assert "CMake Warning" in output
            assert "Fixture link failure" in output
            assert "Copied instead." in output
        elif os.name != "nt":
            assert (destination / "sample.py").is_symlink()
            assert (destination / "package").is_symlink()
            assert "CMake Warning" not in output
