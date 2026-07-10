.. toctree::
   :maxdepth: 1
   :hidden:

.. _bskExtensions:

About Extensions
================

.. sidebar:: What is an Extension?

    An extension is a Basilisk-compatible C++/SWIG module compiled outside of
    the Basilisk source tree.  It links against the same Basilisk runtime
    your simulation uses, so it behaves identically to any built-in module.

Basilisk ships a curated set of simulation modules covering common
astrodynamics tasks.  **Extensions** let you extend Basilisk with your own
C++ modules—new dynamics models, custom environment models, proprietary
algorithms, or research prototypes—without modifying or recompiling
Basilisk itself.

Extensions vs. External Modules
-------------------------------

Extensions are the recommended way to write custom Basilisk modules.  Basilisk
also supports an older :ref:`buildExtModules` mechanism that folds modules
into a from-source Basilisk build, but it requires cloning and recompiling
all of Basilisk every time you make a change.

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Feature
     - Extension ✓ **recommended**
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
   * - Works with ``pip install "bsk[all]"``
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

Why Write an Extension?
-----------------------

.. tip::

    ``pip install "bsk[all]"`` + ``pip install bsk-sdk`` is all you need.
    No cloning. No building Basilisk from source. Ever.

- **No Basilisk source checkout required.** Install Basilisk and the SDK
  from PyPI and start writing your module immediately.  Upgrading Basilisk
  is a single ``pip install --upgrade "bsk[all]"``.

- **Compile only your code.** Extension builds take seconds, not minutes.
  You never wait on a full Basilisk recompile to test a one-line change.

- **Fully portable.** An extension is a standard Python wheel.  Check it into
  your own repo, build it in CI, and install it anywhere with
  ``pip install``.  No build environment setup required on the target machine.

- **Share with the community.** Publish to PyPI and anyone can install
  your module with ``pip install my-extension`` alongside their existing
  Basilisk installation.

- **Keep proprietary code private.** Your module lives in its own
  repository.  Nothing about the Basilisk source is exposed or required.


How Extensions Work
-------------------

Basilisk extensions are built with `bsk-sdk <https://pypi.org/project/bsk-sdk/>`_,
a companion Python package that ships the headers, SWIG interface files, and
CMake helpers needed to compile out-of-tree modules.

.. graphviz::

   digraph extension_arch {
      graph [rankdir=TB, splines=ortho, bgcolor=transparent, nodesep=0.5]
      node  [shape=box, style="rounded,filled", fontname="Helvetica",
             fontsize=13, margin="0.4,0.2", width=4, fixedsize=false]
      edge  [fontname="Helvetica", fontsize=11, minlen=2]

      extension [label="Your extension\n(separate repo / wheel)",
              fillcolor="#dce8fb"]
      sdk    [label="bsk-sdk\n(BSK headers · SWIG interfaces · CMake helpers)",
              fillcolor="#d4edda"]
      bsk    [label="Basilisk: pip install bsk[all]",
              fillcolor="#fff3cd"]

      extension -> sdk [label="  compiles against  "]
      sdk    -> bsk [label="  compatible with  "]
   }

The extension compiles against the same BSK headers and SWIG runtime that
Basilisk uses, so message types, base classes, and the module API are
all fully compatible.

Quick Start
-----------

**1. Install the SDK**

.. code-block:: bash

    pip install bsk-sdk

**2. Create your extension layout**

.. code-block:: text

    my-extension/
    ├── pyproject.toml
    ├── CMakeLists.txt
    ├── my_extension/               # importable Python package
    │   └── __init__.py
    └── exampleCppModule/           # C++/SWIG source
        ├── exampleCppModule.h
        ├── exampleCppModule.cpp
        ├── exampleCppModule.i
        └── _UnitTest/
            └── test_exampleCppModule.py

**3. Wire up CMakeLists.txt**

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.26)
    project(my_extension LANGUAGES C CXX)

    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c
        "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
      OUTPUT_VARIABLE bsk_sdk_dir
    )
    set(bsk-sdk_DIR "${bsk_sdk_dir}")
    find_package(bsk-sdk CONFIG REQUIRED)

    bsk_add_swig_module(
      TARGET exampleCppModule
      INTERFACE exampleCppModule/exampleCppModule.i
      SOURCES   exampleCppModule/exampleCppModule.cpp
      OUTPUT_DIR "${SKBUILD_PLATLIB_DIR}/my_extension"
    )

**4. Build and install**

.. code-block:: bash

    pip install scikit-build-core build
    python -m build --wheel
    pip install dist/*.whl

**5. Use it in a simulation**

.. code-block:: python

    from my_extension import exampleCppModule
    from Basilisk.utilities import SimulationBaseClass

    sim = SimulationBaseClass.SimBaseClass()
    mod = exampleCppModule.ExampleCppModule()
    sim.AddModelToTask("task", mod)

A complete working example is provided in the
`bsk-sdk repository <https://github.com/AVSLab/bsk_sdk/tree/master/examples/custom-atm-extension>`_.

Version Compatibility
---------------------

An extension wheel is compiled against a specific version of the BSK headers.
``bsk-sdk`` version numbers track Basilisk:  ``bsk-sdk==2.11.0`` contains
headers from Basilisk ``v2.11.0``.  If the installed Basilisk version does
not match, CMake will error at configure time with a clear message.

Always install matching versions:

.. code-block:: bash

    pip install "bsk[all]==2.11.0" "bsk-sdk==2.11.0"
