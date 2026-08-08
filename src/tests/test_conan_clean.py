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
"""Tests for Basilisk build-clean helpers."""

import importlib
from pathlib import Path
from types import SimpleNamespace

import pytest


pytest.importorskip("conan")


def test_clean_numba_cache_artifacts(tmp_path, monkeypatch):
    """Ensure clean builds remove repo and user Numba cache artifacts."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    cache_files = [
        tmp_path / "src" / "module" / "__pycache__" / "state.nbc",
        tmp_path / "src" / "module" / "__pycache__" / "state.nbi",
        tmp_path / "docs" / "source" / "codeSamples" / "__pycache__" / "sample.nbc",
        tmp_path / "examples" / "__pycache__" / "scenario.nbi",
    ]
    retained_files = [
        tmp_path / "src" / "module" / "__pycache__" / "state.pyc",
        tmp_path / "src" / "module" / "reference.nbc",
        tmp_path / "docs" / "source" / "codeSamples" / "sample.py",
        tmp_path / "docs" / "source" / "codeSamples" / "reference.nbi",
        tmp_path / "examples" / "scenario_data.nbc",
    ]
    for path in cache_files + retained_files:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("cache", encoding="utf-8")

    user_cache_dir = tmp_path / "user-cache" / "basilisk" / "numba_model"
    user_cache_file = user_cache_dir / "nbm_test.py"
    user_cache_dir.mkdir(parents=True)
    user_cache_file.write_text("cache", encoding="utf-8")

    removed_files, removed_user_cache = conanfile.clean_numba_cache_artifacts(
        tmp_path,
        user_cache_dir,
        print_fn=None,
    )

    assert removed_files == len(cache_files)
    assert removed_user_cache is True
    assert not user_cache_dir.exists()
    for path in cache_files:
        assert not path.exists()
    for path in retained_files:
        assert path.exists()


def test_clean_rust_target_artifacts(tmp_path, monkeypatch):
    """Ensure clean builds remove workspace and stale crate Cargo outputs."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    workspace_manifest = tmp_path / "src" / "Cargo.toml"
    workspace_manifest.parent.mkdir(parents=True)
    workspace_manifest.write_text("[workspace]\n", encoding="utf-8")
    workspace_target_file = tmp_path / "src" / "target" / "debug" / "artifact"
    workspace_target_file.parent.mkdir(parents=True)
    workspace_target_file.write_text("workspace output", encoding="utf-8")

    crate_directories = [
        tmp_path / "src" / "architecture" / "rust" / "bsk_build",
        tmp_path / "src" / "moduleTemplates" / "rustModuleTemplate",
    ]
    for crate_directory in crate_directories:
        (crate_directory / "Cargo.toml").parent.mkdir(parents=True, exist_ok=True)
        (crate_directory / "Cargo.toml").write_text("[package]\n", encoding="utf-8")
        target_file = crate_directory / "target" / "debug" / "artifact"
        target_file.parent.mkdir(parents=True)
        target_file.write_text("build output", encoding="utf-8")

    unrelated_target = tmp_path / "src" / "not_a_crate" / "target"
    unrelated_target.mkdir(parents=True)

    removed_directories = conanfile.clean_rust_target_artifacts(tmp_path, print_fn=None)

    assert removed_directories == len(crate_directories) + 1
    assert not workspace_target_file.parent.parent.exists()
    for crate_directory in crate_directories:
        assert not (crate_directory / "target").exists()
    assert unrelated_target.exists()


@pytest.mark.parametrize("use_absolute_path", [False, True])
def test_clean_configured_build_folder(
        tmp_path,
        monkeypatch,
        use_absolute_path,
):
    """Clean the selected relative or absolute build folder instead of ``dist3``."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    source_root.mkdir()
    default_build = source_root / "dist3"
    default_build.mkdir()
    (default_build / "CMakeCache.txt").write_text("default", encoding="utf-8")

    if use_absolute_path:
        selected_build = tmp_path / "external-build"
        build_option = selected_build
    else:
        selected_build = source_root / "custom-build"
        build_option = Path("custom-build")
    selected_build.mkdir()
    (selected_build / "CMakeCache.txt").write_text("selected", encoding="utf-8")

    cleaned_path = conanfile.clean_configured_build_folder(source_root, build_option)

    assert cleaned_path == selected_build.resolve()
    assert not selected_build.exists()
    assert default_build.exists()


@pytest.mark.parametrize("build_option", [Path("."), Path("..")])
def test_clean_configured_build_folder_rejects_repository_scope(
        tmp_path,
        monkeypatch,
        build_option,
):
    """Refuse to clean the repository root or one of its ancestors."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    source_root.mkdir()

    with pytest.raises(ValueError, match="unsafe build directory"):
        conanfile.clean_configured_build_folder(source_root, build_option)

    assert source_root.exists()


def test_clean_configured_build_folder_rejects_unrecognized_directory(
        tmp_path,
        monkeypatch,
):
    """Avoid deleting a nonempty directory without recognizable build output."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    unrelated_directory = source_root / "documentation"
    unrelated_directory.mkdir(parents=True)
    retained_file = unrelated_directory / "notes.txt"
    retained_file.write_text("retain", encoding="utf-8")

    with pytest.raises(ValueError, match="does not look like"):
        conanfile.clean_configured_build_folder(
            source_root,
            Path("documentation"),
        )

    assert retained_file.exists()


def test_clean_configured_build_folder_accepts_partial_conan_generation(
        tmp_path,
        monkeypatch,
):
    """Clean a build folder where Conan generated files before CMake failed."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    build_directory = source_root / "partial-build"
    generator_directory = build_directory / "Release" / "generators"
    generator_directory.mkdir(parents=True)
    (generator_directory / "conan_toolchain.cmake").write_text(
        "# Partially generated Conan toolchain.\n",
        encoding="utf-8",
    )

    cleaned_path = conanfile.clean_configured_build_folder(
        source_root,
        Path("partial-build"),
    )

    assert cleaned_path == build_directory.resolve()
    assert not build_directory.exists()


def test_recipe_clean_uses_configured_build_folder(tmp_path, monkeypatch):
    """Have the Conan ``configure()`` method clean its ``buildFolder`` option."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    source_root.mkdir()
    default_build = source_root / "dist3"
    selected_build = source_root / "custom-build"
    for build_directory in (default_build, selected_build):
        build_directory.mkdir()
        (build_directory / "CMakeCache.txt").write_text(
            "configured",
            encoding="utf-8",
        )

    option_values = {
        "buildFolder": "custom-build",
        "clean": True,
    }
    recipe = SimpleNamespace(
        options=SimpleNamespace(
            get_safe=lambda name: option_values.get(name, False),
        ),
        recipe_folder=str(source_root),
        settings=SimpleNamespace(get_safe=lambda name: "Release"),
    )
    monkeypatch.setattr(conanfile, "clean_numba_cache_artifacts", lambda root: None)
    monkeypatch.setattr(conanfile, "clean_rust_target_artifacts", lambda root: None)

    conanfile.BasiliskConan.configure(recipe)

    assert not selected_build.exists()
    assert default_build.exists()


def test_windows_dll_scan_excludes_cargo_build_artifacts(tmp_path, monkeypatch):
    """The Windows wheel scan ignores Cargo compiler plugins and its output."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")
    build_folder = tmp_path / "dist3"

    assert conanfile.should_scan_windows_dll_directory(
        str(build_folder / "bin"),
        str(build_folder),
    )
    assert not conanfile.should_scan_windows_dll_directory(
        str(build_folder / "Basilisk"),
        str(build_folder),
    )
    assert not conanfile.should_scan_windows_dll_directory(
        str(build_folder / "cargo" / "workspace" / "release" / "deps"),
        str(build_folder),
    )
