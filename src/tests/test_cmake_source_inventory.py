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


def _write_file(path: Path) -> None:
    """Create an empty source-discovery fixture.

    :param path: Fixture path to create.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
def test_source_inventory_uses_one_glob_per_root(tmp_path):
    """Inventory each source root once and index its relevant source files.

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
    assert len(glob_checks) == 2


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
