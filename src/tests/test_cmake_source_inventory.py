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

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).parents[2]
ENVIRONMENT_CMAKE = Path(sys.executable).with_name("cmake")
CMAKE = shutil.which("cmake") or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)
ENVIRONMENT_NINJA = Path(sys.executable).with_name("ninja")
NINJA = shutil.which("ninja") or (
    str(ENVIRONMENT_NINJA) if ENVIRONMENT_NINJA.is_file() else None
)
GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA:
    GENERATORS.append("Ninja")


def _write_file(path: Path) -> None:
    """Create an empty source-discovery fixture.

    :param path: Fixture path to create.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
def test_source_inventory_watches_build_file_types(tmp_path):
    """Collect and index the relevant source files under each source root.

    :param tmp_path: Temporary directory supplied by pytest.
    """
    first_root = tmp_path / "first"
    second_root = tmp_path / "second"
    first_source = first_root / "module/module.cpp"
    second_header = second_root / "include/module.hpp"
    _write_file(first_source)
    _write_file(first_root / "module/module.rst")
    _write_file(first_root / "module/ignored.txt")
    _write_file(first_root / "target/generated.cpp")
    _write_file(second_header)
    _write_file(second_root / "__pycache__/generated.py")
    external_modules = first_root / "ExternalModules"
    external_wrapper = external_modules / "category/module"
    _write_file(external_wrapper / "Custom.cmake")
    _write_file(external_modules / "category/_GeneralModuleFiles/Custom.cmake")

    module_directory = (REPOSITORY_ROOT / "src/cmake").as_posix()
    project_directory = tmp_path / "project"
    project_directory.mkdir()
    (project_directory / "CMakeLists.txt").write_text(
        f"""cmake_minimum_required(VERSION 3.26)
project(testSourceInventory NONE)
list(APPEND CMAKE_MODULE_PATH "{module_directory}")
include(bskSourceInventory)
include(bskCollectWrapperCustomFiles)
bsk_collect_source_inventory(
  source_inventory
  "{first_root.as_posix()}"
  "{second_root.as_posix()}")
list(LENGTH source_inventory source_count)
if(NOT source_count EQUAL 5)
  message(FATAL_ERROR "Expected five inventoried files: ${{source_inventory}}")
endif()
bsk_index_source_inventory(${{source_inventory}})
bsk_get_directory_source_files(
  module_sources "{(first_root / 'module').as_posix()}")
list(LENGTH module_sources module_source_count)
if(NOT module_source_count EQUAL 2)
  message(FATAL_ERROR "Expected two indexed module files: ${{module_sources}}")
endif()
set(custom_file_inventory ${{source_inventory}})
list(FILTER custom_file_inventory INCLUDE REGEX "(^|/)Custom\\.cmake$")
bsk_collect_wrapper_custom_files(
  wrapper_custom_files
  "{external_wrapper.as_posix()}"
  ExternalModules
  "{project_directory.as_posix()}"
  "{first_root.as_posix()}"
  "${{custom_file_inventory}}")
list(LENGTH wrapper_custom_files custom_file_count)
if(NOT custom_file_count EQUAL 2)
  message(FATAL_ERROR "Expected local and shared custom files: ${{wrapper_custom_files}}")
endif()
""",
        encoding="utf-8",
    )

    build_directory = project_directory / "build"
    subprocess.run(
        [CMAKE, "-S", str(project_directory), "-B", str(build_directory)],
        check=True,
        capture_output=True,
        text=True,
    )

    verify_globs = build_directory / "CMakeFiles/VerifyGlobs.cmake"
    glob_checks = re.findall(
        r"^file\(GLOB", verify_globs.read_text(encoding="utf-8"), re.MULTILINE
    )
    # Both roots watch the eight supported extensions, never a catch-all '*'.
    assert len(glob_checks) == 16


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("generator", GENERATORS)
def test_inventory_ignores_artifacts_and_discovers_new_sources(tmp_path, generator):
    """Ignore output files while discovering source additions and removals.

    :param tmp_path: Temporary directory supplied by pytest.
    :param generator: Native build generator to exercise.
    """
    project = tmp_path / "project"
    source_root = tmp_path / "modules"
    build = tmp_path / "build"
    project.mkdir()
    _write_file(source_root / "existing/module.cpp")
    (project / "CMakeLists.txt").write_text(
        f"""cmake_minimum_required(VERSION 3.26)
project(inventoryChanges NONE)
include("{REPOSITORY_ROOT.as_posix()}/src/cmake/bskSourceInventory.cmake")
bsk_collect_source_inventory(sources "{source_root.as_posix()}")
file(WRITE "${{CMAKE_BINARY_DIR}}/discovered.txt" "${{sources}}")
file(APPEND "${{CMAKE_BINARY_DIR}}/configure-runs.txt" "configured\\n")
""",
        encoding="utf-8",
    )
    command = [CMAKE, "-S", str(project), "-B", str(build)]
    if generator:
        command.extend(["-G", generator])
    if generator == "Ninja":
        command.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    subprocess.run(command, check=True, capture_output=True, text=True)

    def native_build():
        """Check the inventory through a normal native build."""
        subprocess.run(
            [CMAKE, "--build", str(build)],
            check=True, capture_output=True, text=True,
        )

    native_build()
    configure_runs = build / "configure-runs.txt"
    original_runs = configure_runs.read_text(encoding="utf-8")
    artifacts = [
        source_root / "existing/__pycache__/module.cpython-314.pyc",
        source_root / "existing/_UnitTest/plots/result.png",
        source_root / "existing/_UnitTest/run.log",
        source_root / "target/release/deps/module.rlib",
        source_root / "target/release/deps/module.rmeta",
    ]
    for artifact in artifacts:
        _write_file(artifact)
    native_build()
    assert configure_runs.read_text(encoding="utf-8") == original_runs
    for artifact in artifacts:
        artifact.unlink()
    native_build()
    assert configure_runs.read_text(encoding="utf-8") == original_runs

    # New directories, ownership manifests, and messages must still be found
    # on the first build, without requiring a manual configure command.
    new_sources = [
        source_root / "new/module/module.i",
        source_root / "new/module/ModuleSources.cmake",
        source_root / "new/msgPayloadDefC/AddedMsgPayload.h",
    ]
    for new_source in new_sources:
        _write_file(new_source)
    native_build()
    discovered = (build / "discovered.txt").read_text(encoding="utf-8").split(";")
    assert all(path.as_posix() in discovered for path in new_sources)
    assert configure_runs.read_text(encoding="utf-8") != original_runs
    for new_source in new_sources:
        new_source.unlink()
    native_build()
    discovered = (build / "discovered.txt").read_text(encoding="utf-8").split(";")
    assert all(path.as_posix() not in discovered for path in new_sources)
    unchanged_runs = configure_runs.read_text(encoding="utf-8")
    native_build()
    assert configure_runs.read_text(encoding="utf-8") == unchanged_runs


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
def test_production_source_rejects_multiple_owners(tmp_path):
    """Reject a production implementation assigned to different targets.

    :param tmp_path: Temporary directory supplied by pytest.
    """
    source_file = tmp_path / "module.cpp"
    _write_file(source_file)
    module_path = (REPOSITORY_ROOT / "src/cmake/bskSourceInventory.cmake").as_posix()
    script_path = tmp_path / "check-ownership.cmake"
    script_path.write_text(
        f"""include("{module_path}")
bsk_claim_production_source("{source_file.as_posix()}" "first target")
bsk_claim_production_source("{source_file.as_posix()}" "first target")
bsk_claim_production_source("{source_file.as_posix()}" "second target")
""",
        encoding="utf-8",
    )

    result = subprocess.run(
        [CMAKE, "-P", str(script_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "has multiple owners: 'first target' and 'second target'" in result.stderr
