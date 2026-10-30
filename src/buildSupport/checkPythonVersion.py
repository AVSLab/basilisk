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

"""Check build interpreters against Basilisk's package metadata."""

import argparse
from pathlib import Path
import sys


def check_python_version(pyproject_path, version=None):
    """Reject Python versions outside ``project.requires-python``.

    :param pyproject_path: Path to Basilisk's ``pyproject.toml``.
    :param version: Version string to check; defaults to the running interpreter.
    :raises ValueError: If the metadata is invalid or the version is unsupported.
    """
    try:
        import tomllib
    except ModuleNotFoundError:
        import tomli as tomllib
    from packaging.specifiers import SpecifierSet

    with Path(pyproject_path).open("rb") as metadata_file:
        metadata = tomllib.load(metadata_file)
    project = metadata.get("project", {})
    requires_python = project.get("requires-python") if isinstance(project, dict) else None
    if not isinstance(requires_python, str) or not requires_python.strip():
        raise ValueError("pyproject.toml must define project.requires-python.")
    supported_versions = SpecifierSet(requires_python)
    if version is None:
        # Match the release tuple used by installers for Requires-Python checks.
        version = ".".join(str(component) for component in sys.version_info[:3])
    if not supported_versions.contains(version, prereleases=True):
        raise ValueError(
            f"Basilisk requires Python {requires_python}, but found Python {version} "
            f"({sys.executable}). Create or activate a virtual environment with a "
            "supported Python version and rerun the build."
        )


def main(argv=None):
    """Return a command-line status after checking the selected metadata file.

    :param argv: Command-line arguments; defaults to ``sys.argv[1:]``.
    :return: Zero on success, or one if validation fails.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pyproject_path", type=Path)
    args = parser.parse_args(argv)
    try:
        check_python_version(args.pyproject_path)
    except ImportError as error:
        print(
            f"Python version check failed: {error}. Install the build requirements "
            "from requirements_dev.txt.",
            file=sys.stderr,
        )
        return 1
    except (OSError, ValueError) as error:
        print(f"Python version check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
