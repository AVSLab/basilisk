from __future__ import annotations

from importlib import metadata
from typing import Any, Callable, Iterable, Optional

from Basilisk.architecture import sysModel
from ._adapters import normalize_factory, PybindPluginAsSysModel

ENTRY_POINT_GROUP = "basilisk.plugins"


class PluginRegistry:
    def __init__(self) -> None:
        self.py_modules: dict[str, type[sysModel.SysModel]] = {}
        self.factories: dict[str, Any] = {}

    def register_python_module(self, name: str, cls: type[sysModel.SysModel]) -> None:
        if not isinstance(name, str) or not name:
            raise TypeError("Module name must be a non-empty string")
        if not isinstance(cls, type):
            raise TypeError("Only classes can be registered as Python modules")

        try:
            is_sysmodel = issubclass(cls, sysModel.SysModel)
        except TypeError as exc:
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
        if not isinstance(name, str) or not name:
            raise TypeError("Factory name must be a non-empty string")

        existing = self.factories.get(name)
        if existing is not None and existing is not factory:
            raise ValueError(f"Factory name '{name}' already registered")

        self.factories[name] = factory


GLOBAL_REGISTRY = PluginRegistry()
_PLUGINS_LOADED = False


def _iter_plugin_entry_points() -> Iterable[metadata.EntryPoint]:
    eps = metadata.entry_points()
    if hasattr(eps, "select"):
        return eps.select(group=ENTRY_POINT_GROUP)
    return eps.get(ENTRY_POINT_GROUP, [])


def _resolve_register_callable(obj: Any) -> Callable[[PluginRegistry], None]:
    if callable(obj):
        return obj
    register = getattr(obj, "register", None)
    if callable(register):
        return register
    raise TypeError(
        "Basilisk plugin entry points must reference a callable or an object with "
        "a callable 'register' attribute"
    )


def load_all_plugins(registry: Optional[PluginRegistry] = None) -> PluginRegistry:
    global _PLUGINS_LOADED
    if registry is None:
        registry = GLOBAL_REGISTRY
    if _PLUGINS_LOADED:
        return registry

    for ep in _iter_plugin_entry_points():
        register = _resolve_register_callable(ep.load())
        register(registry)

    _PLUGINS_LOADED = True
    return registry


def load(name: str, registry: Optional[PluginRegistry] = None) -> sysModel.SysModel:
    if registry is None:
        registry = load_all_plugins()

    cls = registry.py_modules.get(name)
    if cls is not None:
        return cls()

    fobj = registry.factories.get(name)
    if fobj is None:
        raise KeyError(f"No Basilisk plugin registered under '{name}'")

    factory = normalize_factory(fobj)
    impl = factory()
    return PybindPluginAsSysModel(impl)
