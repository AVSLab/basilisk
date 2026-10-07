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
"""Tests for Basilisk's CMake generation and generator selection."""

import importlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

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


@pytest.fixture
def generation_context(conanfile_module, tmp_path, monkeypatch):
    """Run recipe generation without resolving or generating dependency toolchains."""
    recipe_root = tmp_path / "repository"
    source_root = recipe_root / "src"
    source_root.mkdir(parents=True)
    settings = Mock(os="Linux", build_type="Release")
    settings.get_safe.return_value = None
    conf = Mock()
    conf.get.return_value = False
    recipe = SimpleNamespace(
        recipe_folder=str(recipe_root),
        source_folder=str(source_root),
        options=conanfile_module.BasiliskConan().options,
        settings=settings,
        conf=conf,
    )
    recipe.options.generator = "Ninja"
    toolchain = Mock(cache_variables={})
    monkeypatch.setattr(conanfile_module, "CMakeDeps", Mock())
    monkeypatch.setattr(conanfile_module, "CMakeToolchain", Mock(return_value=toolchain))
    return recipe, toolchain


@pytest.mark.parametrize("export_option", [None, False], ids=["default", "disabled"])
@pytest.mark.parametrize("database_exists", [False, True], ids=["missing", "existing"])
@pytest.mark.parametrize("build_folder", ["dist3", "custom-build"])
def test_compile_commands_export_cleans_only_the_selected_database(
        conanfile_module,
        generation_context,
        tmp_path,
        monkeypatch,
        export_option,
        database_exists,
        build_folder,
):
    """Remove stale databases only when disabled, honoring Conan's resolved output folder."""
    recipe, toolchain = generation_context
    recipe.options.buildFolder = build_folder
    if export_option is not None:
        recipe.options.exportCompileCommands = export_option
    selected_build = tmp_path / "conan-output" / build_folder
    recipe.build_folder = str(selected_build)
    recipe.generators_folder = str(selected_build / "Release" / "generators")
    conanfile_module.write_basilisk_build_marker(Path(recipe.source_folder), selected_build)

    database = selected_build / "compile_commands.json"
    database_contents = '[{"file": "previous.cpp"}]\n'
    if database_exists:
        database.write_text(database_contents, encoding="utf-8")
    retained_artifact = selected_build / "existing-library.a"
    retained_artifact.write_text("retain", encoding="utf-8")
    unselected_database = Path(recipe.recipe_folder) / "dist3" / "compile_commands.json"
    unselected_database.parent.mkdir(parents=True)
    unselected_database.write_text(database_contents, encoding="utf-8")
    monkeypatch.chdir(recipe.recipe_folder)

    conanfile_module.BasiliskConan.generate(recipe)

    export_enabled = export_option is not False
    assert toolchain.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] is export_enabled
    assert database.exists() == (database_exists and export_enabled)
    if database.exists():
        assert database.read_text(encoding="utf-8") == database_contents
    assert retained_artifact.read_text(encoding="utf-8") == "retain"
    assert unselected_database.read_text(encoding="utf-8") == database_contents

    # Repeating the disabled configuration must tolerate the already removed file.
    conanfile_module.BasiliskConan.generate(recipe)
    assert database.exists() == (database_exists and export_enabled)
