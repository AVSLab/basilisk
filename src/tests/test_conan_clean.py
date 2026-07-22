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
