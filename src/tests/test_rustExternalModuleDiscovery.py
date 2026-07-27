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

import shutil
import subprocess
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).parents[2]
CMAKE = shutil.which("cmake")
CARGO = shutil.which("cargo")


def _write_package(package_directory: Path, package_name: str, *, module: bool) -> None:
    """Create a minimal Cargo package used for CMake discovery testing.

    :param package_directory: Directory in which to create the package.
    :param package_name: Cargo package and library name.
    :param module: Whether to mark the package as a Basilisk module.
    """
    package_directory.mkdir(parents=True)
    metadata = "\n[package.metadata.basilisk]\nmodule = true\n" if module else ""
    (package_directory / "Cargo.toml").write_text(
        f"""[package]
name = "{package_name}"
version = "0.1.0"
edition = "2021"
{metadata}
[lib]
path = "{package_name}.rs"
""",
        encoding="utf-8",
    )
    (package_directory / f"{package_name}.rs").write_text("", encoding="utf-8")


def _configure_discovery_test(project_directory: Path, external_root: Path) -> None:
    """Configure a minimal project that exercises external Rust discovery.

    :param project_directory: CMake source and build parent directory.
    :param external_root: External Basilisk project root under test.
    """
    project_directory.mkdir()
    cmake_module_directory = (REPOSITORY_ROOT / "src/cmake").as_posix()
    expected_manifest = (
        external_root / "ExternalModules/externalRust/Cargo.toml"
    ).as_posix()
    (project_directory / "CMakeLists.txt").write_text(
        f"""cmake_minimum_required(VERSION 3.26)
project(testExternalRustDiscovery NONE)
set(BUILD_RUST_MODULES ON)
list(APPEND CMAKE_MODULE_PATH "{cmake_module_directory}")
include(bskFindRustModules)
find_external_rust_package_targets("{external_root.as_posix()}" rust_targets)
list(LENGTH rust_targets rust_target_count)
if(NOT rust_target_count EQUAL 1)
  message(FATAL_ERROR "Expected one external Rust module, got: ${{rust_targets}}")
endif()
list(GET rust_targets 0 rust_target)
get_filename_component(
  rust_target_absolute "${{rust_target}}" ABSOLUTE BASE_DIR "${{CMAKE_SOURCE_DIR}}")
file(REAL_PATH "${{rust_target_absolute}}" rust_target_absolute)
file(REAL_PATH "{expected_manifest}" expected_manifest)
if(NOT rust_target_absolute STREQUAL expected_manifest)
  message(FATAL_ERROR
    "Expected ${{expected_manifest}}, discovered ${{rust_target_absolute}}")
endif()
""",
        encoding="utf-8",
    )
    subprocess.run(
        [CMAKE, "-S", str(project_directory), "-B", str(project_directory / "build")],
        check=True,
        capture_output=True,
        text=True,
    )


@pytest.mark.skipif(CMAKE is None or CARGO is None, reason="CMake and Cargo are required")
@pytest.mark.parametrize("use_workspace", [True, False], ids=["workspace", "packages"])
def test_external_rust_module_discovery(tmp_path, use_workspace):
    """Discover marked external modules in workspace and package layouts.

    :param tmp_path: Temporary directory supplied by pytest.
    :param use_workspace: Create one external workspace when true; otherwise
        create independent packages with their own lockfiles.
    """
    external_root = tmp_path / "External"
    external_modules = external_root / "ExternalModules"
    _write_package(external_modules / "externalRust", "externalRust", module=True)
    _write_package(external_modules / "rustSupport", "rustSupport", module=False)

    if use_workspace:
        (external_root / "Cargo.toml").write_text(
            """[workspace]
resolver = "2"
members = [
    "ExternalModules/externalRust",
    "ExternalModules/rustSupport",
]
""",
            encoding="utf-8",
        )
        manifests = [external_root / "Cargo.toml"]
    else:
        manifests = [
            external_modules / "externalRust/Cargo.toml",
            external_modules / "rustSupport/Cargo.toml",
        ]

    for manifest in manifests:
        subprocess.run(
            [CARGO, "generate-lockfile", "--manifest-path", str(manifest)],
            check=True,
            capture_output=True,
            text=True,
        )

    _configure_discovery_test(tmp_path / "cmake-project", external_root)
