from importlib import resources
from pathlib import Path


def package_root() -> Path:
    return Path(resources.files(__package__))


def cmake_config_dir() -> str:
    return str(package_root() / "lib" / "cmake" / "bsk-sdk")


def include_dir() -> str:
    return str(package_root() / "include")


def include_dirs() -> list[str]:
    root = package_root()
    return [
        str(root / "include"),
        str(root / "include" / "Basilisk"),
        str(root / "include" / "compat"),
    ]


def swig_dir() -> str:
    return str(package_root() / "swig")


def tools_dir() -> str:
    return str(package_root() / "tools")


def msg_autosource_dir() -> str:
    return str(package_root() / "tools" / "msgAutoSource")


__all__ = [
    "package_root",
    "cmake_config_dir",
    "include_dir",
    "include_dirs",
    "swig_dir",
    "tools_dir",
    "msg_autosource_dir",
]
