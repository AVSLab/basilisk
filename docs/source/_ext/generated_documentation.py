"""Utilities for synchronizing generated documentation sources."""

import filecmp
import shutil
from pathlib import Path


def _tree_entries(root):
    """Return the regular files and directories beneath ``root``."""
    files = set()
    directories = set()

    for entry in root.rglob("*"):
        relative_path = entry.relative_to(root)
        if entry.is_symlink():
            raise ValueError(
                f"Generated documentation must not contain symlinks: {entry}"
            )
        if entry.is_dir():
            directories.add(relative_path)
        elif entry.is_file():
            files.add(relative_path)

    return files, directories


def sync_generated_tree(staged_directory, destination_directory):
    """Synchronize a staged generated tree into its live location.

    Files whose contents have not changed are left untouched so that Sphinx can
    use their modification times for incremental builds. Files and directories
    that are absent from the staged tree are removed from the destination.

    :param staged_directory: Complete newly generated documentation tree.
    :param destination_directory: Live documentation tree to update.
    :return: Counts of added, updated, unchanged, and removed files.
    """
    staged_path = Path(staged_directory)
    destination_path = Path(destination_directory)

    if staged_path.is_symlink():
        raise ValueError(
            f"Staged documentation directory must not be a symlink: {staged_path}"
        )
    if not staged_path.is_dir():
        raise ValueError(
            f"Staged documentation directory does not exist: {staged_path}"
        )
    if destination_path.is_symlink():
        raise ValueError(
            f"Documentation destination must not be a symlink: {destination_path}"
        )

    staged_root = staged_path.resolve()
    destination_root = destination_path.resolve()
    if (
        staged_root == destination_root
        or staged_root in destination_root.parents
        or destination_root in staged_root.parents
    ):
        raise ValueError("Staged and destination documentation trees must be separate")

    staged_files, staged_directories = _tree_entries(staged_root)
    destination_root.mkdir(parents=True, exist_ok=True)

    removed_files = 0
    destination_entries = sorted(
        destination_root.rglob("*"),
        key=lambda entry: len(entry.parts),
        reverse=True,
    )
    for destination_entry in destination_entries:
        relative_path = destination_entry.relative_to(destination_root)
        if destination_entry.is_symlink():
            destination_entry.unlink()
            removed_files += 1
        elif destination_entry.is_file() and relative_path not in staged_files:
            destination_entry.unlink()
            removed_files += 1
        elif destination_entry.is_dir() and relative_path not in staged_directories:
            destination_entry.rmdir()

    for relative_path in sorted(staged_directories):
        (destination_root / relative_path).mkdir(parents=True, exist_ok=True)

    added_files = 0
    updated_files = 0
    unchanged_files = 0
    for relative_path in sorted(staged_files):
        staged_file = staged_root / relative_path
        destination_file = destination_root / relative_path
        destination_file.parent.mkdir(parents=True, exist_ok=True)

        if destination_file.exists() and filecmp.cmp(
            staged_file, destination_file, shallow=False
        ):
            unchanged_files += 1
            continue

        if destination_file.exists():
            updated_files += 1
        else:
            added_files += 1
        shutil.copyfile(staged_file, destination_file)

    return {
        "added": added_files,
        "updated": updated_files,
        "unchanged": unchanged_files,
        "removed": removed_files,
    }
