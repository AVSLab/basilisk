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

import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
METADATA_TIMEOUT = 60  # [s]
GIT_TEST_TIMEOUT = 10  # [s]

PACKAGING_SPEC = importlib.util.spec_from_file_location(
    "bsk_packaging", REPO_ROOT / "src/buildSupport/_bsk_packaging.py",
)
PACKAGING = importlib.util.module_from_spec(PACKAGING_SPEC)
PACKAGING_SPEC.loader.exec_module(PACKAGING)

SETUP_METADATA_SCRIPT = """
import sys
from pathlib import Path

from setuptools import build_meta


metadataRoot = Path(sys.argv[1])
metadata_hook = getattr(build_meta, sys.argv[2])
distInfoName = metadata_hook(str(metadataRoot))
metadataPath = metadataRoot / distInfoName / "METADATA"
if not metadataPath.is_file():
    raise RuntimeError(f"Package metadata was not generated at {metadataPath}")

source_files = set((metadataRoot / "bsk.egg-info" / "SOURCES.txt").read_text().splitlines())
required_sources = {
    "src/buildSupport/_bsk_packaging.py",
    "src/CMakeLists.txt",
    "src/Cargo.toml",
    "src/architecture/msgPayloadDefC/SCStatesMsgPayload.h",
}
missing_sources = required_sources - {Path(source).as_posix() for source in source_files}
if missing_sources:
    raise RuntimeError(f"Source discovery omitted required build inputs: {sorted(missing_sources)}")
"""


@pytest.mark.parametrize("metadata_hook", [
    "prepare_metadata_for_build_editable",
    "prepare_metadata_for_build_wheel",
])
def test_metadata_generation(tmp_path, metadata_hook):
    r"""Verify package metadata and source discovery with the installed build tools.

    This exercises ``setup.py`` in an isolated process so both the legacy
    ``Extension`` implementation and the Setuptools 84 dataclass-backed
    implementation use the same Basilisk compatibility path. Both editable
    installs and wheel builds must retain the native and Rust build inputs.
    """
    result = subprocess.run(
        [sys.executable, "-c", SETUP_METADATA_SCRIPT, str(tmp_path), metadata_hook],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=METADATA_TIMEOUT,
    )

    assert result.returncode == 0, (
        f"{metadata_hook} failed:\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )


def test_git_attribute_batches_preserve_paths_and_export_rules(tmp_path, monkeypatch):
    """Preserve Git attribute results for quoted paths across bounded batches."""
    subprocess.run(["git", "init", "--quiet", str(tmp_path)], check=True, timeout=GIT_TEST_TIMEOUT)
    paths = [
        "excluded/", "space dir/", " leading space/", "unicode-μ/", "-dash/",
        "line\nbreak/", "kept/",
    ]
    (tmp_path / ".gitattributes").write_text(
        "".join(f"{json.dumps(path, ensure_ascii=False)} export-ignore\n" for path in paths[:-1]),
        encoding="utf-8",
    )
    monkeypatch.setattr(PACKAGING, "GIT_PATH_ARGUMENT_BYTES", 24)
    commands = []

    def run_command(args, *, cwd, strip, **kwargs):
        commands.append(args)
        assert strip is False
        assert "input" not in kwargs
        assert "--stdin" not in args
        return subprocess.run(
            args, cwd=cwd, capture_output=True, text=True,
            encoding="utf-8", errors="surrogateescape", **kwargs,
        )

    result = PACKAGING._git_with_path_arguments(
        None, run_command, PACKAGING.GIT_ATTRIBUTE_STDIN_ARGS, tmp_path,
        input="\0".join(paths), check=True, timeout=GIT_TEST_TIMEOUT,
    )
    expected = "".join(
        f"{path}\0export-ignore\0{'set' if path != 'kept/' else 'unspecified'}\0"
        for path in paths
    )
    assert result.stdout == expected
    assert len(commands) > 1
    assert all(
        sum(len(path.encode("utf-8")) + 1 for path in command[7:]) <= 24
        for command in commands
    )


def test_git_attribute_failure_is_preserved(tmp_path):
    """Return failed Git results instead of accepting a partial attribute list."""
    failure = subprocess.CompletedProcess(["git"], 1, stdout="", stderr="attribute query failed")
    result = PACKAGING._git_with_path_arguments(
        None, lambda *args, **kwargs: failure,
        PACKAGING.GIT_ATTRIBUTE_STDIN_ARGS, tmp_path, input="directory/",
    )
    assert result is failure


def test_other_git_commands_are_unchanged(tmp_path):
    """Forward unrelated commands and their keyword arguments unchanged."""
    commands = []
    expected = object()

    def run_git(*args, **kwargs):
        commands.append((args, kwargs))
        return expected

    result = PACKAGING._git_with_path_arguments(
        run_git, None, ["ls-files", "-z"], tmp_path, timeout=GIT_TEST_TIMEOUT,
    )
    assert result is expected
    assert commands == [((["ls-files", "-z"], tmp_path), {"timeout": GIT_TEST_TIMEOUT})]


@pytest.mark.parametrize("platform", ["darwin", "linux", "win32"])
def test_git_workaround_scope_and_cleanup(monkeypatch, platform):
    """Adapt only macOS packaging and restore the helper when setup fails."""
    original = object()
    git_finder = SimpleNamespace(run_git=original, _run=object())
    imported = []

    def import_module(name):
        imported.append(name)
        return git_finder

    monkeypatch.setattr(PACKAGING, "sys", SimpleNamespace(platform=platform))
    monkeypatch.setattr(PACKAGING, "importlib", SimpleNamespace(import_module=import_module))
    with pytest.raises(RuntimeError, match="setup failed"):
        with PACKAGING.git_attribute_arguments():
            assert (git_finder.run_git is not original) == (platform == "darwin")
            raise RuntimeError("setup failed")
    assert git_finder.run_git is original
    assert bool(imported) == (platform == "darwin")


@pytest.mark.parametrize("missing_module", [True, False])
def test_git_workaround_with_older_file_finders(monkeypatch, missing_module):
    """Allow older setuptools-scm and vcs-versioning file finders."""
    def import_module(name):
        if missing_module:
            raise ModuleNotFoundError(name="vcs_versioning")
        return SimpleNamespace()

    monkeypatch.setattr(PACKAGING, "sys", SimpleNamespace(platform="darwin"))
    monkeypatch.setattr(PACKAGING, "importlib", SimpleNamespace(import_module=import_module))
    result = PACKAGING.setup_with_git_file_discovery(
        lambda **kwargs: kwargs, name="bsk",
    )
    assert result == {"name": "bsk"}
