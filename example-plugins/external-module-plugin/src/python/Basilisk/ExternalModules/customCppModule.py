"""
Python helpers exposing the C++ CustomCppModule to Basilisk users.
"""

from __future__ import annotations

from importlib import import_module

_extension = import_module("Basilisk.ExternalModules._custom_cpp")

CustomCppModule = _extension.CustomCppModule
CustomPluginMsgPayload = _extension.CustomPluginMsgPayload


def customCppModule():
    """
    Backwards compatible factory returning a new :class:`CustomCppModule`.
    """

    return CustomCppModule()


__all__ = ["CustomCppModule", "CustomPluginMsgPayload", "customCppModule"]
