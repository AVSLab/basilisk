#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
"""Tests for Basilisk's CMake generator selection."""

import importlib
from pathlib import Path

import pytest


pytest.importorskip("conan")


@pytest.fixture(scope="module")
def conanfile_module(monkeypatch_module):
    """Import the repository's Conan recipe for generator tests."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch_module.chdir(repo_root)
    monkeypatch_module.syspath_prepend(str(repo_root))
    return importlib.import_module("conanfile")


@pytest.fixture(scope="module")
def monkeypatch_module(request):
    """Provide module-scoped monkeypatching for the shared recipe import."""
    patch = pytest.MonkeyPatch()
    request.addfinalizer(patch.undo)
    return patch


def test_explicit_generator_rejects_existing_build_mismatch(
        conanfile_module,
        tmp_path,
):
    """Require a clean build before changing an existing CMake generator."""
    (tmp_path / "CMakeCache.txt").write_text(
        "CMAKE_GENERATOR:INTERNAL=Unix Makefiles\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=r"use --clean with conanfile.py"):
        conanfile_module.select_cmake_generator(
            "Ninja", tmp_path, "Linux", True,
        )


def test_explicit_generator_matches_existing_build(conanfile_module, tmp_path):
    """Honor an explicit generator when it matches the existing CMake cache."""
    (tmp_path / "CMakeCache.txt").write_text(
        "CMAKE_GENERATOR:INTERNAL=Ninja\n",
        encoding="utf-8",
    )

    generator, reason = conanfile_module.select_cmake_generator(
        "Ninja", tmp_path, "Linux", True,
    )

    assert generator == "Ninja"
    assert reason == "explicitly requested"


def test_existing_build_generator_is_reused(conanfile_module, tmp_path):
    """Preserve the cached generator so incremental configuration remains valid."""
    (tmp_path / "CMakeCache.txt").write_text(
        "CMAKE_GENERATOR:INTERNAL=Unix Makefiles\n",
        encoding="utf-8",
    )

    generator, reason = conanfile_module.select_cmake_generator(
        "", tmp_path, "Linux", True, lambda _: "/usr/bin/ninja",
    )

    assert generator == "Unix Makefiles"
    assert reason == "reused from the existing build directory"


@pytest.mark.parametrize("operating_system", ["Linux", "Macos", "Windows"])
def test_new_command_line_build_prefers_ninja(
        conanfile_module,
        tmp_path,
        operating_system,
):
    """Use Ninja for a new automatic build when its executable is available."""
    generator, reason = conanfile_module.select_cmake_generator(
        "", tmp_path, operating_system, True, lambda _: "/usr/local/bin/ninja",
    )

    assert generator == "Ninja"
    assert reason == "ninja executable found"


def test_new_command_line_build_falls_back_to_make(conanfile_module, tmp_path):
    """Use Unix Makefiles when Ninja is unavailable."""
    generator, reason = conanfile_module.select_cmake_generator(
        "", tmp_path, "Linux", False, lambda _: None,
    )

    assert generator == "Unix Makefiles"
    assert reason == "ninja executable not found"


def test_new_windows_build_falls_back_to_visual_studio(conanfile_module, tmp_path):
    """Use Visual Studio for an automatic Windows build when Ninja is unavailable."""
    generator, reason = conanfile_module.select_cmake_generator(
        "", tmp_path, "Windows", True, lambda _: None,
    )

    assert generator == "Visual Studio 17 2022"
    assert reason == "ninja executable not found"


@pytest.mark.parametrize(
    ("operating_system", "build_project", "expected_generator"),
    [
        ("Windows", False, "Visual Studio 17 2022"),
        ("Macos", False, "Xcode"),
    ],
)
def test_platform_ide_defaults(
        conanfile_module,
        tmp_path,
        operating_system,
        build_project,
        expected_generator,
):
    """Retain the established Windows and macOS IDE defaults."""
    generator, _ = conanfile_module.select_cmake_generator(
        "", tmp_path, operating_system, build_project, lambda _: "/usr/bin/ninja",
    )

    assert generator == expected_generator
