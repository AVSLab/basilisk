#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""Smoke tests for the runtime plugin registry."""

from __future__ import annotations

import importlib
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import pytest

from Basilisk.architecture import sysModel
from bsk_core import plugins


@pytest.fixture(autouse=True)
def reset_registry():
    """Ensure the global registry is clean before and after every test."""
    snapshot_py = dict(plugins.GLOBAL_REGISTRY.py_modules)
    snapshot_factories = dict(plugins.GLOBAL_REGISTRY.factories)
    snapshot_loaded = plugins._PLUGINS_LOADED

    plugins.GLOBAL_REGISTRY.py_modules.clear()
    plugins.GLOBAL_REGISTRY.factories.clear()
    plugins._PLUGINS_LOADED = False

    yield plugins.GLOBAL_REGISTRY

    plugins.GLOBAL_REGISTRY.py_modules.clear()
    plugins.GLOBAL_REGISTRY.py_modules.update(snapshot_py)

    plugins.GLOBAL_REGISTRY.factories.clear()
    plugins.GLOBAL_REGISTRY.factories.update(snapshot_factories)

    plugins._PLUGINS_LOADED = snapshot_loaded


class DummySysModel(sysModel.SysModel):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.reset_called = False
        self.update_called = False

    def Reset(self, current_sim_nanos):
        self.reset_called = True
        return super().Reset(current_sim_nanos)

    def UpdateState(self, current_sim_nanos, call_time=None):
        self.update_called = True
        # SWIG base only takes (current_sim_nanos)
        return super().UpdateState(current_sim_nanos)


def test_register_python_module_accepts_sysmodel(reset_registry):
    registry = reset_registry
    registry.register_python_module("Dummy", DummySysModel)
    assert registry.py_modules["Dummy"] is DummySysModel


def test_register_python_module_rejects_non_sysmodel(reset_registry):
    registry = reset_registry
    with pytest.raises(TypeError):
        registry.register_python_module("Bad", object)  # type: ignore[arg-type]


def test_register_factory_allows_simple_storage(reset_registry):
    registry = reset_registry
    factory = object()
    registry.register_factory("factory", factory)
    assert registry.factories["factory"] is factory


@dataclass
class _FakeEntryPoint:
    loader: Callable[[plugins.PluginRegistry], None]

    def load(self):
        return self.loader


class _FakeEntryPoints:
    def __init__(self, entries):
        self._entries = entries

    def select(self, *, group):
        if group == plugins.ENTRY_POINT_GROUP:
            return self._entries
        return []


def test_load_all_plugins_discovers_entry_points(monkeypatch, reset_registry):
    calls = []

    def register(registry):
        calls.append(registry)
        registry.register_python_module("PluginSysModel", DummySysModel)

    fake_entry_points = _FakeEntryPoints([_FakeEntryPoint(register)])
    monkeypatch.setattr(plugins.metadata, "entry_points", lambda: fake_entry_points)

    registry = plugins.load_all_plugins()
    assert "PluginSysModel" in registry.py_modules
    assert calls == [registry]

    second = plugins.load_all_plugins()
    assert second is registry
    assert calls == [registry]


def test_modules_namespace_exposes_registered_plugins(monkeypatch, reset_registry):
    modules_pkg = importlib.import_module("Basilisk.modules")

    def fake_loader():
        # Register a module and advertise that discovery has happened.
        plugins.GLOBAL_REGISTRY.register_python_module(
            "NamespaceSysModel", DummySysModel
        )
        plugins._PLUGINS_LOADED = True
        return plugins.GLOBAL_REGISTRY

    monkeypatch.setattr(modules_pkg, "load_all_plugins", fake_loader)

    resolved = modules_pkg.NamespaceSysModel
    assert resolved is DummySysModel

    exported = dir(modules_pkg)
    assert "NamespaceSysModel" in exported

    with pytest.raises(AttributeError):
        getattr(modules_pkg, "DoesNotExist")


def test_example_plugin_discovery(monkeypatch, reset_registry):
    pkg = types.ModuleType("bsk_example_plugin")
    simple_module = types.ModuleType("bsk_example_plugin.simple")

    class ExamplePluginModule(DummySysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = "Example"

    def register(registry: plugins.PluginRegistry):
        registry.register_python_module("ExamplePluginModule", ExamplePluginModule)

    simple_module.ExamplePluginModule = ExamplePluginModule
    simple_module.register = register
    pkg.simple = simple_module

    monkeypatch.setitem(sys.modules, "bsk_example_plugin", pkg)
    monkeypatch.setitem(sys.modules, "bsk_example_plugin.simple", simple_module)

    entry_point = plugins.metadata.EntryPoint(
        name="example",
        value="bsk_example_plugin.simple:register",
        group=plugins.ENTRY_POINT_GROUP,
    )
    monkeypatch.setattr(plugins, "_iter_plugin_entry_points", lambda: [entry_point])

    registry = plugins.load_all_plugins()
    assert "ExamplePluginModule" in registry.py_modules

    modules_pkg = importlib.import_module("Basilisk.modules")
    cls = modules_pkg.ExamplePluginModule
    instance = cls()
    instance.Reset(0)
    instance.UpdateState(0, 0)
    assert instance.reset_called is True
    assert instance.update_called is True


def test_example_cpp_plugin_discovery(monkeypatch, reset_registry):
    pkg = types.ModuleType("bsk_example_plugin_cpp")
    register_module = types.ModuleType("bsk_example_plugin_cpp.register")

    class StubExampleCppModule:
        def __init__(self):
            self.reset_called = False
            self.update_called = False

        def Reset(self, current_sim_nanos):
            self.reset_called = True

        def UpdateState(self, current_sim_nanos, call_time):
            self.update_called = True

    def stub_factory():
        return StubExampleCppModule()

    stub_extension = types.SimpleNamespace(create_factory=lambda: stub_factory)
    monkeypatch.setitem(sys.modules, "bsk_example_plugin_cpp", pkg)
    monkeypatch.setitem(sys.modules, "bsk_example_plugin_cpp.register", register_module)
    monkeypatch.setitem(
        sys.modules, "bsk_example_plugin_cpp._example_cpp", stub_extension
    )

    def register(registry: plugins.PluginRegistry):
        extension = importlib.import_module("bsk_example_plugin_cpp._example_cpp")
        factory = extension.create_factory()
        registry.register_factory("ExampleCppFactory", factory)

    register_module.register = register

    entry_point = plugins.metadata.EntryPoint(
        name="example_cpp",
        value="bsk_example_plugin_cpp.register:register",
        group=plugins.ENTRY_POINT_GROUP,
    )
    monkeypatch.setattr(plugins, "_iter_plugin_entry_points", lambda: [entry_point])

    registry = plugins.load_all_plugins()
    assert "ExampleCppFactory" in registry.factories

    modules_pkg = importlib.import_module("Basilisk.modules")
    factory = modules_pkg.ExampleCppFactory
    instance = factory()
    instance.Reset(0)
    instance.UpdateState(0, 0)
    assert instance.reset_called is True
    assert instance.update_called is True
