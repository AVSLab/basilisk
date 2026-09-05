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

"""Check Rust binding dependencies without downloading or compiling Rust crates."""

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).parents[2]
ENVIRONMENT_CMAKE = Path(sys.executable).with_name("cmake.exe" if os.name == "nt" else "cmake")
ENVIRONMENT_NINJA = Path(sys.executable).with_name("ninja.exe" if os.name == "nt" else "ninja")
CMAKE = shutil.which("cmake") or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)
NINJA = shutil.which("ninja") or (
    str(ENVIRONMENT_NINJA) if ENVIRONMENT_NINJA.is_file() else None
)
GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA:
    GENERATORS.extend(["Ninja", "Ninja Multi-Config"])

# Keep the real Basilisk integration but substitute Corrosion's public target
# graph and Cargo's content-preserving generator. No Rust toolchain, Python
# extension import, C++ compiler, or network access is needed by these tests.
PROJECT = r"""cmake_minimum_required(VERSION 3.26)
project(rustBindingsIncremental NONE)
include("@HELPER@")
file(APPEND "${CMAKE_BINARY_DIR}/configure-runs.txt" "configure\n")

function(_bsk_load_corrosion)
endfunction()
function(_bsk_rust_package_name_from_metadata manifest output)
  set(${output} fixture PARENT_SCOPE)
endfunction()
function(corrosion_import_crate)
  cmake_parse_arguments(IMPORT "LOCKED" "IMPORTED_CRATES;MANIFEST_PATH" "CRATES;CRATE_TYPES" ${ARGN})
  add_library(fixture INTERFACE)
  add_library(fixture-static STATIC IMPORTED)
  add_custom_target(_cargo-build_fixture
    COMMAND "${CMAKE_COMMAND}" -E env "$<TARGET_PROPERTY:fixture,FIXTURE_ENV>"
      "${CMAKE_COMMAND}" "-DSOURCE=${CMAKE_SOURCE_DIR}" "-DBINARY=${CMAKE_BINARY_DIR}"
      -P "${CMAKE_SOURCE_DIR}/cargo.cmake"
    COMMAND_EXPAND_LISTS VERBATIM)
  add_custom_target(cargo-prebuild_fixture)
  add_dependencies(_cargo-build_fixture cargo-prebuild_fixture)
  add_custom_target(cargo-build_fixture)
  add_dependencies(cargo-build_fixture _cargo-build_fixture)
  set(${IMPORT_IMPORTED_CRATES} fixture PARENT_SCOPE)
endfunction()
function(corrosion_set_env_vars target)
  set_property(TARGET ${target} PROPERTY FIXTURE_ENV "${ARGN}")
endfunction()

bsk_add_rust_module_sources(
  TARGET fixture MANIFEST "${CMAKE_SOURCE_DIR}/Cargo.toml"
  OUT_HEADER_VAR header OUT_INTERFACE_VAR interface OUT_BUILD_TARGET_VAR bindings)

# Model SWIG followed by its compiled consumer. Both use ordinary file-level
# dependencies as well as the ordering target supplied by the real helper.
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/wrapper.txt"
  COMMAND "${CMAKE_COMMAND}" "-DHEADER=${header}" "-DINTERFACE=${interface}"
    "-DBINARY=${CMAKE_BINARY_DIR}" -P "${CMAKE_SOURCE_DIR}/wrapper.cmake"
  DEPENDS "${header}" "${interface}" VERBATIM)
add_custom_target(wrapper DEPENDS "${CMAKE_BINARY_DIR}/wrapper.txt")
add_dependencies(wrapper ${bindings})
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/consumer.txt"
  COMMAND "${CMAKE_COMMAND}" -E copy
    "${CMAKE_BINARY_DIR}/wrapper.txt" "${CMAKE_BINARY_DIR}/consumer.txt"
  DEPENDS "${CMAKE_BINARY_DIR}/wrapper.txt" "${header}" VERBATIM)
add_custom_target(consumer ALL DEPENDS "${CMAKE_BINARY_DIR}/consumer.txt")
add_dependencies(consumer wrapper)
"""

CARGO_SCRIPT = r"""file(APPEND "${BINARY}/cargo-runs.txt" "cargo\n")
# Simulate Cargo's watched-input cache, including the production missing-file
# trigger. Merely deleting a binding must not bypass that trigger in this test.
file(SHA256 "${SOURCE}/module.rs" module_hash)
file(SHA256 "${SOURCE}/support.rs" support_hash)
file(SHA256 "${SOURCE}/implementation.rs" implementation_hash)
file(SHA256 "$ENV{BSK_BINDINGS_TRIGGER_PATH}" trigger_hash)
set(fingerprint "${module_hash};${support_hash};${implementation_hash};${trigger_hash}")
set(previous "")
if(EXISTS "${BINARY}/cargo-cache.txt")
  file(READ "${BINARY}/cargo-cache.txt" previous)
endif()
if(NOT fingerprint STREQUAL previous)
  configure_file("${SOURCE}/module.rs" "$ENV{BSK_HEADER_PATH}" COPYONLY)
  configure_file("${SOURCE}/support.rs" "$ENV{BSK_INTERFACE_PATH}" COPYONLY)
  file(WRITE "${BINARY}/cargo-cache.txt" "${fingerprint}")
endif()
"""

WRAPPER_SCRIPT = r"""file(READ "${HEADER}" header)
file(READ "${INTERFACE}" interface)
file(WRITE "${BINARY}/wrapper.txt" "${header}${interface}")
file(APPEND "${BINARY}/wrapper-runs.txt" "wrapper\n")
"""


def _run(command):
    """Run a build command and include its diagnostics in any assertion failure.

    :param command: Executable and command-line arguments.
    """
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("generator", GENERATORS)
def test_rust_binding_changes_reach_consumers_in_one_build(tmp_path, generator):
    """Propagate changed or missing bindings in one build, without no-op work.

    :param tmp_path: Temporary directory supplied by pytest.
    :param generator: Native CMake generator to exercise.
    """
    project = tmp_path / "project with spaces"
    build = tmp_path / "build with spaces"
    project.mkdir()
    helper = REPOSITORY_ROOT / "src/cmake/bskAddRustModuleSources.cmake"
    files = {
        "CMakeLists.txt": PROJECT.replace("@HELPER@", helper.as_posix()),
        "Cargo.toml": "# Metadata discovery is stubbed for this dependency-graph test.\n",
        "cargo.cmake": CARGO_SCRIPT,
        "wrapper.cmake": WRAPPER_SCRIPT,
        "module.rs": "original header\n",
        "support.rs": "original interface\n",
        "implementation.rs": "original implementation\n",
    }
    for name, content in files.items():
        (project / name).write_text(content, encoding="utf-8")
    configure = [CMAKE, "-S", str(project), "-B", str(build)]
    if generator:
        configure.extend(["-G", generator])
    if generator and generator.startswith("Ninja"):
        configure.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    _run(configure)

    def nativeBuild():
        """Perform exactly one invocation of the native incremental build."""
        _run([CMAKE, "--build", str(build), "--config", "Release", "--parallel", "4"])

    def separateMakeTimestamps():
        """Allow Apple's bundled Make to distinguish rapid fixture edits."""
        # Make 3.81 can compare whole-second timestamps. Test dependency
        # ordering independently of that coarse clock resolution.
        if generator == "Unix Makefiles":
            time.sleep(1.05)  # [s]

    header = build / "rust/include/fixture.h"
    interface = build / "rust/swig/fixture_rust_wrap.i"
    outputs = [header, interface, build / "wrapper.txt", build / "consumer.txt"]

    def assertCurrent():
        """Check the consumer against both current generated binding inputs."""
        expected = (project / "module.rs").read_text(encoding="utf-8")
        expected += (project / "support.rs").read_text(encoding="utf-8")
        assert outputs[-1].read_text(encoding="utf-8") == expected

    def assertNoWrapperRebuild():
        """Confirm Cargo still checks inputs without rebuilding unchanged consumers."""
        times = [path.stat().st_mtime_ns for path in outputs]
        wrapperRuns = (build / "wrapper-runs.txt").read_text(encoding="utf-8")
        cargoRuns = (build / "cargo-runs.txt").read_text(encoding="utf-8")
        nativeBuild()
        assert [path.stat().st_mtime_ns for path in outputs] == times
        assert (build / "wrapper-runs.txt").read_text(encoding="utf-8") == wrapperRuns
        assert (build / "cargo-runs.txt").read_text(encoding="utf-8") != cargoRuns

    nativeBuild()
    assertCurrent()
    assertNoWrapperRebuild()

    # Either a module edit or a support-crate edit may change just one binding.
    for name in ("module.rs", "support.rs"):
        separateMakeTimestamps()
        (project / name).write_text(f"changed {name}\n", encoding="utf-8")
        nativeBuild()
        assertCurrent()
        assertNoWrapperRebuild()

    # A Rust-only implementation change must not force another SWIG invocation.
    (project / "implementation.rs").write_text("changed implementation\n", encoding="utf-8")
    assertNoWrapperRebuild()

    # Restore either missing binding, or both, with Cargo's input cache intact.
    for missing in ([header], [interface], [header, interface]):
        separateMakeTimestamps()
        for path in missing:
            path.unlink()
        nativeBuild()
        assert all(path.is_file() for path in outputs)
        assertCurrent()
        assertNoWrapperRebuild()

    _run([CMAKE, "--build", str(build), "--config", "Release", "--target", "clean"])
    assert not header.exists()
    assert not interface.exists()
    nativeBuild()
    assertCurrent()
    assertNoWrapperRebuild()
    assert (build / "configure-runs.txt").read_text(encoding="utf-8") == "configure\n"
