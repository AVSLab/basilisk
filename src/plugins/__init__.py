"""
Public plugin API for Basilisk.

- Discovers entry points in the "basilisk.plugins" group.
- Provides `load()` to construct a SysModel from a registered plugin name.
"""

from ._registry import GLOBAL_REGISTRY, PluginRegistry, load_all_plugins, load

__all__ = ["GLOBAL_REGISTRY", "PluginRegistry", "load_all_plugins", "load"]
