# Basilisk Plugin Example

This package is a **template** demonstrating how to distribute custom Basilisk modules as a
standalone plugin powered by the :mod:`bsk-sdk`. It contains both Python and
C++ examples and registers them with the Basilisk runtime through the plugin
entry-point system.

Use this structure as a starting point for your own custom Basilisk plugins.

## Building

```bash
pip install -e ./sdk              # install the published SDK locally
pip install scikit-build-core
pip install -e ./example-plugins/external-module-plugin --no-build-isolation
```

Requirements (beyond the Basilisk runtime):

- Python 3.8+
- A C++17 compiler
- ``bsk-sdk`` (published via ``./sdk`` in this repo for local development)
- ``pybind11`` (installed automatically via the ``bsk-sdk`` dependency)
- ``scikit-build-core`` (build tooling)

## Usage

After installation the plugin is discovered automatically:

```python
from Basilisk import modules

cpp_factory = modules.CustomCppModule
instance = cpp_factory()
instance.Reset(0)
instance.UpdateState(0, 0)
```

The plugin also exposes a pure Python module:

```python
from Basilisk import modules

python_cls = modules.CustomPythonModule
module = python_cls()
module.Reset(0)
module.UpdateState(0, 0)
```

The C++ factory mirrors Basilisk's ``SysModel`` concept and exposes a plugin
specific message payload:

```python
from Basilisk import modules
from Basilisk.ExternalModules import customCppModule

factory = modules.CustomCppModule
instance = factory()
payload = customCppModule.CustomPluginMsgPayload([1.0, 0.0, 0.0])
instance.set_input_payload(payload)
instance.Reset(0)
instance.UpdateState(1_000_000_000)
print(instance.last_output().dataVector)
```

```
