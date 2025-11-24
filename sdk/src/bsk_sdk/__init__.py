"""
Utilities for locating the Basilisk SDK headers within a Python environment.

The package installs the header-only SDK under ``include/bsk``. Plugin build
systems can use :func:`include_dir` to configure their compiler include paths.
"""

from importlib import resources
from pathlib import Path


def include_dir() -> str:
    return str(resources.files(__package__) / "include")


def include_dirs() -> list[str]:
    root = Path(include_dir())
    dirs = [str(root), str(root / "Basilisk")]

    try:
        import pybind11
    except ImportError:
        return dirs

    dirs.append(pybind11.get_include())
    return dirs


__all__ = ["include_dir", "include_dirs"]
