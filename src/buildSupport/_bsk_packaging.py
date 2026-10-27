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

"""Compatibility helpers used only while packaging Basilisk."""

import importlib
import os
import sys
from contextlib import contextmanager
from functools import partial
from pathlib import Path


GIT_PATH_ARGUMENT_BYTES = 16 * 1024  # Maximum encoded path bytes per command.
GIT_ATTRIBUTE_STDIN_ARGS = ["check-attr", "-z", "--stdin", "export-ignore"]


def _path_batches(paths):
    """Keep Git command arguments bounded, including multibyte path names."""
    batch = []
    batch_bytes = 0
    for path in paths:
        path_bytes = len(os.fsencode(path)) + 1
        if batch and batch_bytes + path_bytes > GIT_PATH_ARGUMENT_BYTES:
            yield batch
            batch = []
            batch_bytes = 0
        batch.append(path)
        batch_bytes += path_bytes
    if batch:
        yield batch


def _git_with_path_arguments(run_git, run_command, args, repo, **kwargs):
    """Send export-ignore queries as arguments instead of a bidirectional pipe."""
    input_paths = kwargs.get("input")
    if (
        list(args) != GIT_ATTRIBUTE_STDIN_ARGS
        or not isinstance(input_paths, str)
        or not input_paths
    ):
        return run_git(args, repo, **kwargs)

    paths = input_paths.split("\0")
    if not paths[-1]:
        paths.pop()
    options = {key: value for key, value in kwargs.items() if key != "input"}
    stdout = []
    stderr = []
    for batch in _path_batches(paths):
        # Keep upstream environment handling and timeout/error behavior, but
        # preserve whitespace at the start of the first path in each batch.
        result = run_command(
            ["git", "--git-dir", Path(repo) / ".git", "check-attr", "-z",
             "export-ignore", "--", *batch],
            cwd=repo, strip=False, **options,
        )
        if result.returncode:
            return result
        stdout.append(result.stdout)
        stderr.append(result.stderr)
    result.stdout = "".join(stdout)
    result.stderr = "".join(stderr)
    return result


@contextmanager
def git_attribute_arguments():
    """Temporarily adapt the macOS Git file finder during a packaging operation.

    macOS can report a pipe writable even when its buffer cannot grow. Python
    then blocks writing directory names while Git blocks writing responses.
    Keep the upstream file selection and attribute parsing, but avoid sending
    input through a pipe. Older setuptools-scm releases without vcs-versioning
    do not use this path. The private helper is restored even if setup fails.
    """
    if sys.platform != "darwin":
        yield
        return
    try:
        git_finder = importlib.import_module("vcs_versioning._file_finders._git")
    except ModuleNotFoundError as error:
        if error.name not in {
            "vcs_versioning",
            "vcs_versioning._file_finders",
            "vcs_versioning._file_finders._git",
        }:
            raise
        yield
        return

    original_run_git = getattr(git_finder, "run_git", None)
    if original_run_git is None:
        # Early vcs-versioning releases used a different file finder.
        yield
        return
    git_finder.run_git = partial(
        _git_with_path_arguments, original_run_git, git_finder._run,
    )
    try:
        yield
    finally:
        git_finder.run_git = original_run_git


def setup_with_git_file_discovery(setup_function, **kwargs):
    """Run setuptools with the scoped Git attribute transport workaround."""
    with git_attribute_arguments():
        return setup_function(**kwargs)
