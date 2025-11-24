Basilisk SDK Version 1
======================

.. contents:: Outline
   :local:

Purpose
-------

The Basilisk SDK (``bsk-sdk``) defines the public surface that external plugin
authors can rely on when integrating new simulation capabilities with the
core runtime. Version 1 focuses on establishing a stable contract for Python
and C++ plugin authors and capturing the minimal tooling that ships inside the
Basilisk source tree.

Scope and Deliverables
----------------------

Version 1 guarantees the following artifacts:

- ``bsk_core.plugins``: the runtime registry responsible for discovering
  entry-point advertised plugins and exposing them under ``Basilisk.modules``.
- ``bsk-sdk``: a small Python package that publishes the SDK headers, declares a
  dependency on the ``pybind11`` headers required by the helper macros, and
  provides :func:`bsk_sdk.include_dir` / :func:`bsk_sdk.include_dirs` helpers for
  build scripts.
- A companion ``sync_headers.py`` utility (``sdk/tools``) keeps the vendored
  Basilisk ``architecture`` headers in sync with the main source tree.
- ``sdk/include/bsk/sdk.hpp``: a single header that wraps the pybind11
  boilerplate required for C++ factories and enforces the default constructible
  + ``Reset``/``UpdateState`` interface contract. The same header is shipped by
  :mod:`bsk-sdk`.
- A consolidated ``plugins`` example package containing both Python and C++
  implementations that demonstrate the expected packaging and registration
  patterns.

Any other files in the repository are explicitly *not* part of the SDK
agreement for this release.

Plugin Registry API
-------------------

The ``bsk_core.plugins.PluginRegistry`` class is the primary integration
point for third-party plugins. The registry is responsible for staging plugin
definitions until the runtime exports them under ``Basilisk.modules``.

The public methods guaranteed in v1 are:

.. code-block:: python

   class PluginRegistry:
       def register_python_module(self, name: str, cls: type[sysModel.SysModel]) -> None: ...
       def register_factory(self, name: str, factory: Any) -> None: ...

``register_python_module`` accepts any subclass of
``Basilisk.architecture.sysModel.SysModel`` and exposes it as a class on
``Basilisk.modules`` using the provided name. ``register_factory`` stores an
opaque object under the supplied name. Factories are expected to be callables
returning Basilisk-compatible module instances, but v1 defers any runtime shape
validation to keep the surface area small.

Plugins must advertise a ``register(registry)`` callable through the
``basilisk.plugins`` entry-point group. During startup Basilisk resolves the
entry-point, imports the containing module, and invokes the callable with the
shared registry instance.

Python Plugin Pattern
---------------------

Pure-Python plugins should follow the pattern demonstrated in
``plugins/src/python/Basilisk/ExternalModules/customPythonModule.py``:

.. code-block:: python

   from Basilisk.architecture import sysModel

   class ExamplePluginModule(sysModel.SysModel):
       def Reset(self, current_sim_nanos):
           ...

       def UpdateState(self, current_sim_nanos, call_time):
           ...

   def register(registry):
       registry.register_python_module("ExamplePluginModule", ExamplePluginModule)

The distribution's ``pyproject.toml`` must expose the ``register`` function via

.. code-block:: toml

   [project.entry-points."basilisk.plugins"]
   example = "bsk_example_plugin.simple:register"

At runtime users import the module from ``Basilisk.modules``:

.. code-block:: python

   from Basilisk import modules

   plugin_cls = modules.ExamplePluginModule
   instance = plugin_cls()
   instance.Reset(0)
   instance.UpdateState(0, 0)

C++ Plugin Pattern
------------------

Native extensions should include ``sdk/include/bsk/sdk.hpp`` to inherit
the pybind11 binding helpers. When building outside the Basilisk source tree
the :mod:`bsk-sdk` package exposes the headers via
``import bsk_sdk; bsk_sdk.include_dir()`` (or ``include_dirs()`` to also capture
the ``Basilisk`` subdirectory and ``pybind11`` include path). Version 1
guarantees the availability of:

- ``bsk::plugin::register_basic_plugin``
- ``BSK_PLUGIN_PYBIND_MODULE``

The ``BSK_PLUGIN_PYBIND_MODULE`` macro defines both the pybind11 module and the
``create_factory`` callable consumed by the Basilisk runtime. The expected class
contract mirrors the Python case: default constructible with ``Reset`` and
``UpdateState`` methods.

.. code-block:: cpp

   #include <bsk/sdk.hpp>

   class ExampleCppModule {
    public:
     void Reset(double current_sim_nanos);
     void UpdateState(double current_sim_nanos, double call_time);
   };

   BSK_PLUGIN_PYBIND_MODULE(_example_cpp, ExampleCppModule, "ExampleCppModule");

The companion Python package should lazily import the extension, extract the
factory, and register it:

.. code-block:: python

   from importlib import import_module

   def register(registry):
       ext = import_module("bsk_example_plugin_cpp._example_cpp")
       factory = ext.create_factory()
       registry.register_factory("ExampleCppFactory", factory)

Limitations and Future Work
---------------------------

Version 1 intentionally leaves several items out of scope so they can be
designed with real-world feedback:

- The SDK header is distributed from the Basilisk source tree and is not
  published as a standalone artifact.
- Factories registered via ``register_factory`` are treated as opaque callables;
  Basilisk does not verify their type or interface beyond name collisions.
- The helper header requires C++17 and a compatible pybind11 toolchain.
- Plugin lifecycle hooks beyond ``Reset``/``UpdateState`` will be designed as
  future Basilisk modules adopt richer interfaces.

Feedback on these gaps is welcome and will inform the roadmap for subsequent
SDK revisions.
