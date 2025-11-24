"""
Example of External Basilisk modules distributed as a standalone plugin.

The package mirrors Basilisk's legacy layout by exposing each module as a child
module so imports such as ``from Basilisk.ExternalModules import customCppModule``
continue to work.
"""

from __future__ import annotations

from . import customCppModule, customPythonModule

__all__ = ["customCppModule", "customPythonModule"]
