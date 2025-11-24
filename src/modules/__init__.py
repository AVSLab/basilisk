"""
Dynamic plugin namespace for Basilisk runtime modules.

This module defers plugin discovery until attributes are accessed. Plugins are
expected to register their SysModel subclasses or factories via the global
registry that lives in :mod:`bsk_core.plugins`.
"""

from __future__ import annotations

from typing import Any, Iterable

from bsk_core.plugins import GLOBAL_REGISTRY, load_all_plugins

__all__ = ["GLOBAL_REGISTRY", "load_all_plugins"]


def _known_attribute_names() -> Iterable[str]:
    load_all_plugins()
    return tuple(set(GLOBAL_REGISTRY.py_modules) | set(GLOBAL_REGISTRY.factories))


def __getattr__(name: str) -> Any:
    load_all_plugins()

    if name in GLOBAL_REGISTRY.py_modules:
        return GLOBAL_REGISTRY.py_modules[name]

    if name in GLOBAL_REGISTRY.factories:
        return GLOBAL_REGISTRY.factories[name]

    raise AttributeError(f"module 'Basilisk.modules' has no attribute '{name}'")


def __dir__() -> list[str]:
    return sorted(set(globals()) - {"__builtins__"} | set(_known_attribute_names()))
