"""
Runtime plugin registration support for Basilisk.

Only Python-based registration is implemented today. C++ factories are stubbed
out to support future extensions without breaking the public API.
"""

from __future__ import annotations

from importlib import metadata
from typing import Any, Callable, Iterable, Optional

from Basilisk.architecture import sysModel

ENTRY_POINT_GROUP = "basilisk.plugins"
"""Python entry point group used to discover Basilisk plugins."""


class PluginRegistry:
    """Container for Basilisk plugin registrations."""

    def __init__(self) -> None:
        self.py_modules: dict[str, type[sysModel.SysModel]] = {}
        self.factories: dict[str, Any] = {}

    def register_python_module(self, name: str, cls: type[sysModel.SysModel]) -> None:
        """Register a Python :class:`~Basilisk.architecture.sysModel.SysModel` subclass."""
        if not isinstance(name, str) or not name:
            raise TypeError("Module name must be a non-empty string")
        if not isinstance(cls, type):
            raise TypeError("Only classes can be registered as Python modules")

        try:
            is_sysmodel = issubclass(cls, sysModel.SysModel)
        except TypeError as exc:  # cls is not a class or similar edge cases
            raise TypeError("Only SysModel subclasses can be registered") from exc

        if not is_sysmodel:
            raise TypeError(
                f"Cannot register {cls!r} as '{name}': not a SysModel subclass"
            )

        existing = self.py_modules.get(name)
        if existing is not None and existing is not cls:
            raise ValueError(
                f"Module name '{name}' already registered with {existing!r}"
            )

        self.py_modules[name] = cls

    def register_factory(self, name: str, factory: Any) -> None:
        """
        Register a future C++ factory.

        No validation is performed yet; factories act as opaque callables or
        objects until C++ support is implemented.
        """
        if not isinstance(name, str) or not name:
            raise TypeError("Factory name must be a non-empty string")

        existing = self.factories.get(name)
        if existing is not None and existing is not factory:
            raise ValueError(f"Factory name '{name}' already registered")

        self.factories[name] = factory


GLOBAL_REGISTRY = PluginRegistry()
"""Shared registry instance used across the Basilisk runtime."""

_PLUGINS_LOADED = False


def _iter_plugin_entry_points() -> Iterable[metadata.EntryPoint]:
    """Return an iterable over all registered plugin entry points."""
    entry_points = metadata.entry_points()
    if hasattr(entry_points, "select"):
        return entry_points.select(group=ENTRY_POINT_GROUP)
    return entry_points.get(ENTRY_POINT_GROUP, [])


def _resolve_register_callable(obj: Any) -> Callable[[PluginRegistry], None]:
    """Normalize the value advertised by an entry point into a register callable."""
    if callable(obj):
        return obj  # The entry point points directly to register()

    register = getattr(obj, "register", None)
    if callable(register):
        return register

    raise TypeError(
        "Basilisk plugin entry points must reference a callable or an object with "
        "a callable 'register' attribute"
    )


def load_all_plugins(registry: Optional[PluginRegistry] = None) -> PluginRegistry:
    """
    Discover and register all Basilisk plugins using ``importlib.metadata``.

    The discovery process is idempotent; repeated calls do not re-register
    plugins.
    """
    global _PLUGINS_LOADED

    if registry is None:
        registry = GLOBAL_REGISTRY

    if _PLUGINS_LOADED:
        return registry

    for entry_point in _iter_plugin_entry_points():
        register = _resolve_register_callable(entry_point.load())
        register(registry)

    _PLUGINS_LOADED = True
    return registry


__all__ = ["GLOBAL_REGISTRY", "PluginRegistry", "load_all_plugins"]
