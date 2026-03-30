.. toctree::
   :maxdepth: 1
   :hidden:

.. _bskPlugins:

About Plugins
================

.. sidebar:: What is a Plugin?

    A plugin is a Basilisk-compatible C++/SWIG module compiled outside of
    the Basilisk source tree.  It links against the same Basilisk runtime
    your simulation uses, so it behaves identically to any built-in module.

Basilisk ships a curated set of simulation modules covering common
astrodynamics tasks.  **Plugins** let you extend Basilisk with your own
C++ modules—new dynamics models, custom environment models, proprietary
algorithms, or research prototypes—without modifying or recompiling
Basilisk itself.

Plugins vs. External Modules
-----------------------------

Plugins are the recommended way to write custom Basilisk modules.  Basilisk
also supports an older :ref:`buildExtModules` mechanism that folds modules
into a from-source Basilisk build, but it requires cloning and recompiling
all of Basilisk every time you make a change.

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Feature
     - Plugin ✓ **recommended**
     - External module
   * - Requires cloning BSK source
     - **No**
     - Yes
   * - Requires recompiling all of Basilisk
     - **No** — builds in seconds
     - Yes — full rebuild
   * - Distributable via ``pip install``
     - **Yes**
     - No
   * - Works with ``pip install bsk``
     - **Yes**
     - No
   * - Portable across machines/environments
     - **Yes** — just install the wheel
     - No — tied to a local build
   * - Can be kept in a private repo
     - Yes
     - Yes
   * - Supports custom messages
     - Yes
     - Yes

Only use external modules if you specifically need your code compiled into
the same build as Basilisk (i.e. available as
``Basilisk.ExternalModules.myModule``).

Why Write a Plugin?
-------------------

.. tip::

    ``pip install bsk`` + ``pip install bsk-sdk`` is all you need.
    No cloning. No building Basilisk from source. Ever.

- **No Basilisk source checkout required.** Install Basilisk and the SDK
  from PyPI and start writing your module immediately.  Upgrading Basilisk
  is a single ``pip install --upgrade bsk``.

- **Compile only your code.** Plugin builds take seconds, not minutes.
  You never wait on a full Basilisk recompile to test a one-line change.

- **Fully portable.** A plugin is a standard Python wheel.  Check it into
  your own repo, build it in CI, and install it anywhere with
  ``pip install``.  No build environment setup required on the target machine.

- **Share with the community.** Publish to PyPI and anyone can install
  your module with ``pip install my-plugin`` alongside their existing
  Basilisk installation.

- **Keep proprietary code private.** Your module lives in its own
  repository.  Nothing about the Basilisk source is exposed or required.


How Plugins Work
----------------

Basilisk plugins are built with `bsk-sdk <https://pypi.org/project/bsk-sdk/>`_,
a companion Python package that ships the headers, SWIG interface files, and
CMake helpers needed to compile out-of-tree modules.

.. code-block:: text

    ┌─────────────────────────────────────┐
    │  Your plugin (separate repo/wheel)  │
    │                                     │
    │   myModule.cpp / myModule.h         │
    │   myModule.i                        │
    │   CMakeLists.txt                    │
    └──────────────┬──────────────────────┘
                   │ links against
    ┌──────────────▼──────────────────────┐
    │  bsk-sdk wheel                      │
    │  (vendored BSK headers + cmake)     │
    └──────────────┬──────────────────────┘
                   │ compatible with
    ┌──────────────▼──────────────────────┐
    │  Basilisk (pip install bsk)         │
    └─────────────────────────────────────┘

The plugin compiles against the same BSK headers and SWIG runtime that
Basilisk uses, so message types, base classes, and the module API are
all fully compatible.

Quick Start
-----------

**1. Install the SDK**

.. code-block:: bash

    pip install bsk-sdk

**2. Create your plugin layout** (following BSK module conventions)

.. code-block:: text

    my-plugin/
    ├── pyproject.toml
    ├── CMakeLists.txt
    └── myModule/
        ├── myModule.h
        ├── myModule.cpp
        ├── myModule.i
        └── _UnitTest/
            └── test_myModule.py

**3. Wire up CMakeLists.txt**

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.18)
    project(my_plugin LANGUAGES C CXX)

    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c
        "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
      OUTPUT_VARIABLE bsk_sdk_dir
    )
    set(bsk-sdk_DIR "${bsk_sdk_dir}")
    find_package(bsk-sdk CONFIG REQUIRED)

    bsk_add_swig_module(
      TARGET myModule
      INTERFACE myModule/myModule.i
      SOURCES   myModule/myModule.cpp
      LINK_LIBS bsk::plugin
      OUTPUT_DIR "${SKBUILD_PLATLIB_DIR}/my_plugin"
    )

**4. Build and install**

.. code-block:: bash

    pip install scikit-build-core build
    python -m build --wheel
    pip install dist/*.whl

**5. Use it in a simulation**

.. code-block:: python

    from my_plugin import myModule
    from Basilisk.utilities import SimulationBaseClass

    sim = SimulationBaseClass.SimBaseClass()
    mod = myModule.MyModule()
    sim.AddModelToTask("task", mod)

A complete working example is provided in the
`bsk-sdk repository <https://github.com/AVSLab/bsk_sdk/tree/develop/examples/custom-atm-plugin>`_.

Version Compatibility
---------------------

A plugin wheel is compiled against a specific version of the BSK headers.
``bsk-sdk`` version numbers track Basilisk:  ``bsk-sdk==2.9.1`` contains
headers from Basilisk ``v2.9.1``.  If the installed Basilisk version does
not match, CMake will error at configure time with a clear message.

Always install matching versions:

.. code-block:: bash

    pip install "bsk==2.9.1" "bsk-sdk==2.9.1"
