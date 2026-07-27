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

"""Test fast Rust feature detection in the repository test runner."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "bsk_run_all_test", REPOSITORY_ROOT / "run_all_test.py"
)
assert RUNNER_SPEC is not None
assert RUNNER_SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(RUNNER_SPEC)
RUNNER_SPEC.loader.exec_module(RUNNER)


def _writeBuildInfo(package_directory: Path, rust_modules: object) -> None:
    """Write a minimal generated build-information module for one test."""
    package_directory.mkdir(parents=True)
    (package_directory / "_buildInfoData.py").write_text(
        f"buildInfoData = {{'features': {{'rustModules': {rust_modules!r}}}}}\n",
        encoding="utf-8",
    )


@pytest.mark.parametrize("rust_modules", [False, True])
def test_rustModulesEnabledReadsSourceBuild(tmp_path, monkeypatch, rust_modules):
    """Read the local ``dist3`` metadata without importing Basilisk."""
    _writeBuildInfo(tmp_path / "dist3" / "Basilisk", rust_modules)
    monkeypatch.setattr(RUNNER, "find_spec", lambda _name: None)

    assert RUNNER._rust_modules_enabled(tmp_path) is rust_modules


def test_rustModulesEnabledSupportsRelocatedEditablePackage(tmp_path, monkeypatch):
    """Fall back to generated metadata exposed by an editable installation."""
    package_directory = tmp_path / "editable" / "Basilisk"
    _writeBuildInfo(package_directory, True)
    package_spec = SimpleNamespace(submodule_search_locations=[str(package_directory)])
    monkeypatch.setattr(RUNNER, "find_spec", lambda _name: package_spec)

    assert RUNNER._rust_modules_enabled(tmp_path / "checkout") is True


def test_rustModulesEnabledRejectsMalformedMetadata(tmp_path, monkeypatch):
    """Reject a generated Rust feature that is not Boolean."""
    _writeBuildInfo(tmp_path / "dist3" / "Basilisk", "enabled")
    monkeypatch.setattr(RUNNER, "find_spec", lambda _name: None)

    with pytest.raises(RuntimeError, match="must be a Boolean"):
        RUNNER._rust_modules_enabled(tmp_path)
