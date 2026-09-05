"""Tests for content-aware synchronization of generated documentation."""

import importlib.util
import os
from pathlib import Path


DOCUMENTATION_SYNC_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs"
    / "source"
    / "_ext"
    / "generated_documentation.py"
)
SPEC = importlib.util.spec_from_file_location(
    "generated_documentation", DOCUMENTATION_SYNC_PATH
)
GENERATED_DOCUMENTATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATED_DOCUMENTATION)


def _write_file(path, contents):
    """Write a test file, creating its parent directory first."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf8")


def test_sync_generated_tree_preserves_unchanged_files(tmp_path):
    """Verify synchronization updates content without touching equal files."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "destination"

    _write_file(staged_root / "unchanged.rst", "unchanged\n")
    _write_file(staged_root / "updated.rst", "new contents\n")
    _write_file(staged_root / "new" / "added.rst", "added\n")
    _write_file(staged_root / "was-a-file" / "child.rst", "child\n")

    _write_file(destination_root / "unchanged.rst", "unchanged\n")
    _write_file(destination_root / "updated.rst", "old contents\n")
    _write_file(destination_root / "stale" / "obsolete.rst", "obsolete\n")
    _write_file(destination_root / "was-a-file", "obsolete blocker\n")

    unchanged_file = destination_root / "unchanged.rst"
    original_timestamp = 1_600_000_000_000_000_000  # [ns]
    os.utime(unchanged_file, ns=(original_timestamp, original_timestamp))

    counts = GENERATED_DOCUMENTATION.sync_generated_tree(
        staged_root, destination_root
    )

    assert unchanged_file.stat().st_mtime_ns == original_timestamp
    assert (destination_root / "updated.rst").read_text(encoding="utf8") == (
        "new contents\n"
    )
    assert (destination_root / "new" / "added.rst").is_file()
    assert (destination_root / "was-a-file" / "child.rst").is_file()
    assert not (destination_root / "stale").exists()
    assert counts == {"added": 2, "updated": 1, "unchanged": 1, "removed": 2}


def test_sync_generated_tree_replaces_directory_with_file(tmp_path):
    """Verify a generated file can replace an obsolete directory tree."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "destination"

    _write_file(staged_root / "replacement.rst", "replacement\n")
    _write_file(destination_root / "replacement.rst" / "obsolete.rst", "old\n")

    GENERATED_DOCUMENTATION.sync_generated_tree(staged_root, destination_root)

    replacement = destination_root / "replacement.rst"
    assert replacement.is_file()
    assert replacement.read_text(encoding="utf8") == "replacement\n"


def test_sync_generated_tree_creates_missing_destination(tmp_path):
    """Verify synchronization supports the first build after ``make clean``."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "missing-destination"
    _write_file(staged_root / "index.rst", "Generated documentation\n")

    counts = GENERATED_DOCUMENTATION.sync_generated_tree(
        staged_root, destination_root
    )

    assert (destination_root / "index.rst").is_file()
    assert counts == {"added": 1, "updated": 0, "unchanged": 0, "removed": 0}
