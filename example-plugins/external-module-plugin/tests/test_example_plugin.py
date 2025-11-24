from __future__ import annotations

import sys
from pathlib import Path
import importlib
import types

import pytest

# Ensure the Basilisk sources under development are importable when running the
# tests directly from the repository.
REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = REPO_ROOT / "src"
if SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

from bsk_core.plugins import PluginRegistry

from bsk_example_plugin.register import register


@pytest.fixture()
def registry():
    return PluginRegistry()


@pytest.fixture(autouse=True)
def plugin_sys_path(monkeypatch):
    test_root = Path(__file__).resolve().parents[3]
    sdk_python = test_root / "src"
    plugin_src = (
        test_root / "example-plugins" / "external-module-plugin" / "src" / "python"
    )
    monkeypatch.syspath_prepend(str(sdk_python))
    monkeypatch.syspath_prepend(str(plugin_src))


@pytest.fixture(autouse=True)
def stub_cpp_extension(monkeypatch):
    class StubPayload:
        def __init__(self, values=None):
            self.dataVector = list(values or [0.0, 0.0, 0.0])

    class StubCppModule:
        def __init__(self):
            self.reset_called = False
            self.update_called = False
            self._steps = 0
            self._input = StubPayload()
            self._output = StubPayload()
            self._last_update = 0

        def Reset(self, current_sim_nanos):
            del current_sim_nanos
            self.reset_called = True
            self.update_called = False
            self._steps = 0
            self._output = StubPayload()
            self._last_update = 0

        def UpdateState(self, current_sim_nanos):
            self.update_called = True
            self._steps += 1
            vec = self._input.dataVector
            self._output = StubPayload(
                [vec[0] + float(self._steps), vec[1], float(current_sim_nanos) * 1e-9]
            )
            self._last_update = current_sim_nanos

        def set_input_payload(self, payload):
            self._input = payload

        def last_input(self):
            return self._input

        def last_output(self):
            return self._output

        def last_update_nanos(self):
            return self._last_update

    class StubModule:
        CustomCppModule = StubCppModule
        CustomPluginMsgPayload = StubPayload

        @staticmethod
        def create_factory():
            def factory():
                return StubCppModule()

            return factory

    monkeypatch.setitem(sys.modules, "Basilisk.ExternalModules._custom_cpp", StubModule)


def test_registers_python_and_cpp_modules(registry):
    register(registry)

    assert "CustomCppModule" in registry.factories
    assert "customCppModule" in registry.factories
    assert "CustomPythonModule" in registry.factories
    assert "customPythonModule" in registry.factories


def test_cpp_factory_behaves_like_module(registry):
    register(registry)

    factory = registry.factories["CustomCppModule"]
    instance = factory()

    assert instance.reset_called is False
    assert instance.update_called is False

    instance.Reset(0)
    assert instance.reset_called is True

    from Basilisk.ExternalModules import customCppModule as cpp_mod

    payload = cpp_mod.CustomPluginMsgPayload([1.0, -0.5, 0.7])
    instance.set_input_payload(payload)
    instance.UpdateState(1_500_000_000)
    assert instance.update_called is True
    assert instance.last_input().dataVector == [1.0, -0.5, 0.7]
    assert instance.last_output().dataVector == pytest.approx([2.0, -0.5, 1.5])


def test_python_module_behaves_like_module(registry):
    register(registry)

    factory = registry.factories["CustomPythonModule"]
    instance = factory()

    instance.input_vector = [0.1, 0.2, 0.3]
    instance.Reset(0.0)
    assert instance.reset_called is True

    instance.UpdateState(0.0, 2.0)
    assert instance.update_called is True
    assert instance.dummy == pytest.approx(1.0)
    assert instance.data_vector == pytest.approx([1.1, 0.2, 2.0])


def test_legacy_wrappers_expose_modules():
    # Ensure the wrappers can be imported even before the plugin is registered.
    from Basilisk.ExternalModules import customCppModule, customPythonModule

    cpp_instance = customCppModule.customCppModule()
    assert isinstance(cpp_instance, customCppModule.CustomCppModule)
    payload = customCppModule.CustomPluginMsgPayload()
    assert payload.dataVector == [0.0, 0.0, 0.0]

    py_instance = customPythonModule.customPythonModule()
    assert isinstance(py_instance, customPythonModule.CustomPythonModule)


def test_documented_quickstart(monkeypatch, stub_cpp_extension):
    """Demonstrate the user-facing workflow for loading the plugin."""

    from bsk_core import plugins

    entry_point = types.SimpleNamespace(load=lambda: register)
    monkeypatch.setattr(plugins, "_iter_plugin_entry_points", lambda: [entry_point])
    monkeypatch.setattr(plugins, "GLOBAL_REGISTRY", plugins.PluginRegistry())
    monkeypatch.setattr(plugins, "_PLUGINS_LOADED", False)

    modules_pkg = importlib.import_module("Basilisk.modules")
    modules_pkg = importlib.reload(modules_pkg)

    from Basilisk.ExternalModules import customCppModule, customPythonModule

    cpp_factory = getattr(modules_pkg, "CustomCppModule")
    cpp_instance = cpp_factory()
    assert cpp_instance.reset_called is False

    payload = customCppModule.CustomPluginMsgPayload([1.0, -0.5, 0.0])
    cpp_instance.set_input_payload(payload)
    cpp_instance.Reset(0)
    cpp_instance.UpdateState(500_000_000)
    assert cpp_instance.last_input().dataVector == [1.0, -0.5, 0.0]
    assert cpp_instance.last_output().dataVector == pytest.approx([2.0, -0.5, 0.5])

    py_factory = getattr(modules_pkg, "CustomPythonModule")
    py_instance = py_factory()
    py_instance.input_vector = [0.25, 1.0, 0.0]
    py_instance.Reset(0.0)
    py_instance.UpdateState(0.0, 3.0)
    assert py_instance.data_vector == pytest.approx([1.25, 1.0, 3.0])
