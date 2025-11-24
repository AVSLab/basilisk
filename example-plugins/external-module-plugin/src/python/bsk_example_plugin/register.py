"""
Entry point for the Basilisk Example plugin.

This module hooks the packaged custom modules into the Basilisk runtime through
``bsk_core.plugins``.
"""


from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any, Callable

if TYPE_CHECKING:  # pragma: no cover - typing only
    from bsk_core.plugins import PluginRegistry


def _load_cpp_extension() -> Any:
    try:
        return import_module("Basilisk.ExternalModules._custom_cpp")
    except ModuleNotFoundError as exc:  # pragma: no cover - build/runtime issue
        raise ImportError(
            "Unable to import the Basilisk External C++ extension. "
            "Ensure the package was built with scikit-build-core."
        ) from exc


def _register_cpp_factory(registry: "PluginRegistry") -> None:
    extension = _load_cpp_extension()
    if hasattr(extension, "register_payloads"):
        extension.register_payloads()

    factory = extension.create_factory()

    registry.register_factory("CustomCppModule", factory)
    registry.register_factory("customCppModule", factory)


def _register_python_module(registry: "PluginRegistry") -> None:
    module = import_module("Basilisk.ExternalModules.customPythonModule")

    def factory():
        return module.CustomPythonModule()

    registry.register_factory("CustomPythonModule", factory)
    registry.register_factory("customPythonModule", factory)


def register(registry: "PluginRegistry") -> None:
    """Register all external modules with the Basilisk runtime."""

    _register_cpp_factory(registry)
    _register_python_module(registry)


__all__ = ["register"]
