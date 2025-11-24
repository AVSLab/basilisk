"""
Lightweight core helpers that are shared across Basilisk's Python surface.

The plugin system implemented in :mod:`bsk_core.plugins` is responsible for
discovering entry points and exposing their registrations to the runtime.
"""

from .plugins import GLOBAL_REGISTRY, PluginRegistry, load_all_plugins

__all__ = ["GLOBAL_REGISTRY", "PluginRegistry", "load_all_plugins"]
