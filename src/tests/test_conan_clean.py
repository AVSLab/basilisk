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

import pytest


pytest.importorskip("conan")


def write_cmake_cache(build_directory, source_root):
    """Write the CMake source ownership entry used by cleanup tests."""
    build_directory.mkdir(parents=True, exist_ok=True)
    (build_directory / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={Path(source_root).resolve()}\n",
        encoding="utf-8",
    )


def test_is_symlink_or_junction_supports_legacy_windows_python(monkeypatch):
    """Detect junction reparse points when ``Path.is_junction`` is unavailable."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    class JunctionStatus:
        """Provide the Windows file attribute exposed by ``Path.lstat``."""

        st_file_attributes = 0x400

    class JunctionPath:
        """Model a pre-Python 3.12 path pointing at a Windows junction."""

        @staticmethod
        def is_symlink():
            """Report that the junction is not a symbolic link."""
            return False

        @staticmethod
        def lstat():
            """Return a Windows reparse-point file status."""
            return JunctionStatus()

    monkeypatch.setattr(conanfile.os, "name", "nt")

    assert conanfile._is_symlink_or_junction(JunctionPath())


def test_recipe_resource_paths_do_not_depend_on_working_directory(
        tmp_path,
        monkeypatch,
):
    """Resolve recipe imports and version files outside the repository root."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(tmp_path)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    expected_version = (repo_root / "docs/source/bskVersion.txt").read_text(
        encoding="utf-8"
    )
    expected_mujoco_version = (repo_root / "libs/mujoco/version.txt").read_text(
        encoding="utf-8"
    ).strip()

    assert conanfile.BasiliskConan.version == expected_version
    assert conanfile.get_mujoco_version() == expected_mujoco_version


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
    write_cmake_cache(selected_build, source_root)

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

    with pytest.raises(ValueError, match="not owned by this Basilisk source tree"):
        conanfile.clean_configured_build_folder(
            source_root,
            Path("documentation"),
        )

    assert retained_file.exists()


def test_clean_configured_build_folder_accepts_partial_conan_generation(
        tmp_path,
        monkeypatch,
):
    """Clean an owned build folder where Conan generation stopped before CMake."""
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
    (build_directory / conanfile.BASILISK_BUILD_MARKER).write_text(
        f"{source_root.resolve()}\n",
        encoding="utf-8",
    )

    cleaned_path = conanfile.clean_configured_build_folder(
        source_root,
        Path("partial-build"),
    )

    assert cleaned_path == build_directory.resolve()
    assert not build_directory.exists()


def test_clean_configured_build_folder_rejects_unowned_conan_generation(
        tmp_path,
        monkeypatch,
):
    """Do not treat another Conan project's generator file as Basilisk ownership."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    source_root.mkdir()
    build_directory = source_root / "partial-build"
    generator_directory = build_directory / "Release" / "generators"
    generator_directory.mkdir(parents=True)
    retained_file = generator_directory / "conan_toolchain.cmake"
    retained_file.write_text("# Unowned Conan output.\n", encoding="utf-8")

    with pytest.raises(ValueError, match="not owned by this Basilisk source tree"):
        conanfile.clean_configured_build_folder(source_root, build_directory)

    assert retained_file.exists()


def test_clean_configured_build_folder_rejects_unrelated_cmake_build(
        tmp_path,
        monkeypatch,
):
    """Preserve a CMake build configured from an unrelated source tree."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "basilisk" / "src"
    source_root.mkdir(parents=True)
    unrelated_source = tmp_path / "unrelated" / "src"
    unrelated_source.mkdir(parents=True)
    build_directory = tmp_path / "unrelated-build"
    write_cmake_cache(build_directory, unrelated_source)
    retained_file = build_directory / "important-artifact"
    retained_file.write_text("retain", encoding="utf-8")

    with pytest.raises(ValueError, match="configured from"):
        conanfile.clean_configured_build_folder(source_root, build_directory)

    assert retained_file.exists()


@pytest.mark.parametrize("link_parent", [False, True])
def test_clean_configured_build_folder_rejects_symlink_traversal(
        tmp_path,
        monkeypatch,
        link_parent,
):
    """Preserve build output reached through a final or parent symbolic link."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    source_root = tmp_path / "source"
    source_root.mkdir()
    victim_parent = tmp_path / "victim"
    victim_build = victim_parent / "build"
    write_cmake_cache(victim_build, source_root)
    retained_file = victim_build / "important-artifact"
    retained_file.write_text("retain", encoding="utf-8")

    try:
        if link_parent:
            linked_parent = source_root / "linked-parent"
            linked_parent.symlink_to(victim_parent, target_is_directory=True)
            build_option = linked_parent / "build"
        else:
            linked_build = source_root / "linked-build"
            linked_build.symlink_to(victim_build, target_is_directory=True)
            build_option = linked_build
    except OSError as error:
        pytest.skip(f"Symbolic links are unavailable: {error}")

    with pytest.raises(ValueError, match="symbolic link or junction"):
        conanfile.clean_configured_build_folder(source_root, build_option)

    assert retained_file.exists()


def test_prepare_conan_build_folder_uses_resolved_output_folder(
        tmp_path,
        monkeypatch,
):
    """Clean Conan's post-layout output folder instead of repository ``dist3``."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch.chdir(repo_root)
    monkeypatch.syspath_prepend(str(repo_root))
    conanfile = importlib.import_module("conanfile")

    recipe_root = tmp_path / "repository"
    source_root = recipe_root / "src"
    source_root.mkdir(parents=True)
    repository_build = recipe_root / "dist3"
    write_cmake_cache(repository_build, source_root)
    repository_artifact = repository_build / "retain"
    repository_artifact.write_text("repository build", encoding="utf-8")

    output_build = tmp_path / "output" / "dist3"
    write_cmake_cache(output_build, source_root)
    removed_artifact = output_build / "remove"
    removed_artifact.write_text("selected output build", encoding="utf-8")
    generators_folder = output_build / "Release" / "generators"
    generators_folder.mkdir(parents=True)

    monkeypatch.setattr(conanfile, "clean_numba_cache_artifacts", lambda root: None)
    monkeypatch.setattr(conanfile, "clean_rust_target_artifacts", lambda root: None)
    monkeypatch.chdir(generators_folder)

    marker_path = conanfile.prepare_conan_build_folder(
        recipe_root=recipe_root,
        source_root=source_root,
        build_folder=output_build,
        generators_folder=generators_folder,
        clean=True,
    )

    assert repository_artifact.exists()
    assert not removed_artifact.exists()
    assert marker_path == output_build / conanfile.BASILISK_BUILD_MARKER
    assert marker_path.read_text(encoding="utf-8").strip() == str(source_root.resolve())
    assert Path.cwd() == generators_folder.resolve()


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
