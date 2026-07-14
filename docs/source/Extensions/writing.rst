.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingExtensions:

Quick Start: Writing a Basilisk Extension
=========================================

.. sidebar:: BSK-SDK Repository

    The `bsk-sdk repository <https://github.com/AVSLab/bsk_sdk>`_ contains the
    SDK source and a complete
    `custom atmosphere extension
    <https://github.com/AVSLab/bsk_sdk/tree/master/examples/custom-atm-extension>`_.

.. note::

   Modules can also be implemented in Rust instead of C++/C — see
   :ref:`writingRustPlugins` (experimental).

This guide walks through the complete extension workflow: build the working
example, understand its files, create a package, wrap a C++ module, generate a
custom message, build a wheel, and run tests. Read :ref:`bskExtensions` first
for the architectural difference between an extension and an integrated
external-folder build.

Extension modules follow the same C and C++ conventions as built-in Basilisk
modules. See :ref:`cppModules` and :ref:`cModules` for module lifecycle,
messages, configuration, and testing conventions. This page focuses on the
out-of-tree build and packaging details.

You need Git, a supported Python version, a C++17 compiler, CMake 3.26 or
newer, and Ninja or another CMake build tool. The Python commands below install
the Python-provided CMake and Ninja packages for convenience.

Build the Working Example First
-------------------------------

Before creating a new project, build the SDK example unchanged. This verifies
the compiler, Python environment, Basilisk installation, SDK, SWIG runtime,
message generator, and wheel tooling together.

Create a clean environment. The activation command shown is for macOS and
Linux; on Windows use ``.venv\Scripts\activate``.

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install "bsk[all]" bsk-sdk build scikit-build-core pytest \
     "cmake>=3.26" "ninja>=1.5"

Verify that Basilisk and ``bsk-sdk`` target the same version:

.. code-block:: bash

   python -c "import Basilisk, bsk_sdk; print('Basilisk:', Basilisk.__version__); print('bsk-sdk:', bsk_sdk.bsk_version())"

Obtain the SDK repository and build its example:

.. code-block:: bash

   git clone https://github.com/AVSLab/bsk_sdk.git
   cd bsk_sdk
   python -m build --wheel --no-isolation examples/custom-atm-extension
   python -m pip install examples/custom-atm-extension/dist/*.whl
   python -c "import Basilisk, numba, custom_atm; from custom_atm import numbaAtmosphere"
   python -m pytest examples -v

The example contains a C++ atmosphere module, a pure-Python Numba module, a
small C source using a built-in Basilisk C message interface, and an
extension-defined message. Core Basilisk is not rebuilt. After this succeeds,
use the following sections to adapt the example or create a new extension.

Choose the Package Names
------------------------

An extension normally has two related names:

* The **distribution name** is used by ``pip``, such as ``my-bsk-extension``.
  Hyphens are conventional here.
* The **import package** is used by Python, such as ``my_bsk_extension``.
  It must be a valid Python identifier and normally uses underscores.

The extension's own version is independent of the Basilisk version. For
example, ``my-bsk-extension==1.3.0`` can target ``bsk==2.11.0``. The targeted
Basilisk and ``bsk-sdk`` versions must match each other exactly.

Create the Project Layout
-------------------------

A useful starting layout is:

.. code-block:: text

   my-bsk-extension/
   |-- pyproject.toml
   |-- CMakeLists.txt
   |-- my_bsk_extension/
   |   `-- __init__.py
   |-- messages/
   |   `-- MyStatusMsgPayload.h
   `-- exampleCppModule/
       |-- exampleCppModule.h
       |-- exampleCppModule.cpp
       |-- exampleCppModule.i
       `-- _UnitTest/
           `-- test_exampleCppModule.py

The import package initially contains only ``__init__.py``. CMake places the
compiled SWIG modules and generated message package into this directory when
building the wheel. Pure-Python modules can also be stored there directly.

Configure Python Packaging
--------------------------

Create ``pyproject.toml`` and replace every ``2.X.Y`` with the Basilisk release
being targeted:

.. code-block:: toml

   [build-system]
   requires = [
       "scikit-build-core>=0.9.3",
       "numpy>=1.24",
       "bsk-sdk==2.X.Y",
       "bsk==2.X.Y",
       "swig==4.4.1",
   ]
   build-backend = "scikit_build_core.build"

   [project]
   name = "my-bsk-extension"
   version = "0.1.0"
   requires-python = ">=3.9"
   dependencies = ["bsk==2.X.Y"]

   [tool.scikit-build]
   wheel.packages = ["my_bsk_extension"]

The build requirements create the native extension. The runtime dependency
ensures that installing a prebuilt extension wheel also installs the compatible
Basilisk release. ``bsk-sdk`` is a build dependency and should not be a runtime
dependency unless the installed package itself exposes SDK development tools.

Write the Module
----------------

Write the module header and implementation using normal Basilisk patterns. A
C++ module can inherit directly from ``SysModel`` or from an SDK-supported
Basilisk base class such as ``AtmosphereBase`` or ``DynamicEffector``.

Use the same include paths used inside Basilisk:

.. code-block:: cpp

   #include "architecture/messaging/messaging.h"
   #include "architecture/utilities/bskLogging.h"
   #include "architecture/_GeneralModuleFiles/sys_model.h"
   #include "MyStatusMsgPayload.h"

Do not copy Basilisk headers or implementation files into the extension. The
SDK supplies its supported headers and required implementation sources during
the build.

Create the SWIG Interface
-------------------------

The SWIG interface exposes the compiled C++ class to Python. A minimal
``exampleCppModule.i`` for a ``SysModel`` subclass is:

.. code-block:: swig

   %module exampleCppModule

   %include "architecture/utilities/bskException.swg"
   %default_bsk_exception();

   %{
   #include "exampleCppModule.h"
   %}

   %pythoncode %{
   from Basilisk.architecture.swig_common_model import *
   %}

   %include "swig_conly_data.i"
   %import "sys_model.i"
   %include "exampleCppModule.h"

   %pythoncode %{
   import sys
   protectAllClasses(sys.modules[__name__])
   %}

Use ``%import`` for a Basilisk base type that is already wrapped by a Basilisk
Python module. ``%import`` tells SWIG to reuse that existing Python type. Using
``%include`` for ``sys_model.i`` creates a second, incompatible wrapper type;
the resulting object can fail when passed to ``AddModelToTask()`` even though
the C++ inheritance is correct.

An intermediate C++ base class that has no existing Basilisk Python wrapper can
be included locally before the extension class:

.. code-block:: swig

   %import "sys_model.i"
   %include "simulation/environment/_GeneralModuleFiles/atmosphereBase.h"
   %include "customAtmosphere.h"

The complete SDK example demonstrates this pattern with ``AtmosphereBase``.

Configure CMake
---------------

Create ``CMakeLists.txt``. The first block locates the SDK installed in the
active Python environment; the second block defines extension targets.

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.26)
   project(my_bsk_extension LANGUAGES C CXX)

   find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

   execute_process(
     COMMAND "${Python3_EXECUTABLE}" -c
       "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
     OUTPUT_VARIABLE bsk_sdk_dir
     RESULT_VARIABLE rc
   )
   if(NOT rc EQUAL 0 OR bsk_sdk_dir STREQUAL "")
     message(FATAL_ERROR
       "bsk-sdk was not found in the active Python environment")
   endif()
   file(TO_CMAKE_PATH "${bsk_sdk_dir}" bsk_sdk_dir)
   set(bsk-sdk_DIR "${bsk_sdk_dir}")
   find_package(bsk-sdk CONFIG REQUIRED)

   set(EXTENSION_PKG_DIR
       "${SKBUILD_PLATLIB_DIR}/my_bsk_extension")

   bsk_add_swig_module(
     TARGET exampleCppModule
     INTERFACE
       "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule/exampleCppModule.i"
     SOURCES
       "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule/exampleCppModule.cpp"
     INCLUDE_DIRS
       "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule"
       "${CMAKE_CURRENT_SOURCE_DIR}/messages"
     OUTPUT_DIR "${EXTENSION_PKG_DIR}"
   )

``bsk_add_swig_module`` configures Python, SWIG, Eigen, Basilisk include paths,
built-in C message interfaces, and the SDK support sources automatically. List
only extension-owned sources and any additional third-party libraries in this
target.

Add a Custom Message
--------------------

Define extension-owned payloads under ``messages/`` using normal Basilisk
message naming. For example, ``MyStatusMsgPayload.h`` can contain:

.. code-block:: cpp

   #pragma once

   #include <stdint.h>

   typedef struct {
       double measurement;  //!< [m] Example measured distance
       int32_t valid;        //!< [-] 1 when the measurement is valid
   } MyStatusMsgPayload;

Add message generation to ``CMakeLists.txt`` after the package output directory
is defined:

.. code-block:: cmake

   bsk_generate_messages(
     OUTPUT_DIR "${EXTENSION_PKG_DIR}/messaging"
     MSG_HEADERS
       "${CMAKE_CURRENT_SOURCE_DIR}/messages/MyStatusMsgPayload.h"
   )

This produces a ``my_bsk_extension.messaging`` package containing
``MyStatusMsgPayload``, ``MyStatusMsg``, and recorder support. Add
``GENERATE_C_INTERFACE`` when generating the C message interface needed by an
extension C module.

If the module's public interface exposes the custom message, make the payload
type visible in its SWIG interface before including the module header:

.. code-block:: swig

   %include "MyStatusMsgPayload.h"
   struct MyStatusMsg_C;
   %include "exampleCppModule.h"

Changing this payload header regenerates and recompiles the affected extension
targets. It does not rebuild core Basilisk or its global message package.

Initialize the Python Package
-----------------------------

For a ``SysModel``-derived module with generated messages, use the following
import order in ``my_bsk_extension/__init__.py``:

.. code-block:: python

   import sys

   from Basilisk.architecture import cSysModel as _cSysModel

   sys.modules.setdefault("cSysModel", _cSysModel)

   from . import messaging
   from . import exampleCppModule

   __all__ = ["exampleCppModule", "messaging"]

The ``cSysModel`` alias must exist before importing the generated module
wrapper because SWIG resolves that base module while defining the Python class.
Importing ``messaging`` before the module wrapper registers the custom
``Message<T>`` and ``Recorder<T>`` proxy classes. Without this ordering,
custom message fields may lack methods such as ``recorder()``.

If the extension has no custom messages, omit the ``messaging`` import. If it
does not expose a ``SysModel``-derived class, the ``cSysModel`` alias may not be
needed.

Build and Install the Extension
-------------------------------

For the first development build, install the selected Basilisk and SDK versions
in the active environment and build without isolation:

.. code-block:: bash

   python -m pip install "bsk[all]==2.X.Y" "bsk-sdk==2.X.Y"
   python -m pip install build scikit-build-core pytest
   python -m build --wheel --no-isolation
   python -m pip install --force-reinstall dist/*.whl

The installed extension wheel contains native code and is specific to its
operating system, processor architecture, Python compatibility, and targeted
Basilisk version.

An editable install is useful while iterating:

.. code-block:: bash

   python -m pip install --no-build-isolation -e .

For release artifacts, also test a normal isolated build. This verifies that
``pyproject.toml`` declares every build dependency:

.. code-block:: bash

   python -m build --wheel

Run the Tests
-------------

Place module tests in the module's ``_UnitTest`` directory and run them against
the installed extension wheel:

.. code-block:: bash

   python -m pytest exampleCppModule/_UnitTest/ -v

At minimum, test that:

* the extension package and compiled module import successfully;
* the module can be added to a Basilisk task;
* built-in Basilisk input and output messages connect correctly;
* custom messages can be written, read, and recorded; and
* the module behavior is validated against known results.

Use the same unit and integration testing standards as a built-in Basilisk
module. Rebuild and reinstall the wheel after changing native source or message
headers.

Use the Extension in a Simulation
---------------------------------

Once installed, import the extension by its own package name and schedule the
module normally:

.. code-block:: python

   from Basilisk.utilities import SimulationBaseClass, macros
   from my_bsk_extension import exampleCppModule

   sim = SimulationBaseClass.SimBaseClass()
   process = sim.CreateNewProcess("process")
   task_time_step = macros.sec2nano(1.0)  # [ns], converted from 1 s
   task = sim.CreateNewTask("task", task_time_step)
   process.addTask(task)

   module = exampleCppModule.ExampleCppModule()
   sim.AddModelToTask("task", module)

Connect messages and configure the module exactly as for a built-in Basilisk
module.

Common Build and Import Problems
--------------------------------

``Basilisk version mismatch``
   The installed ``bsk`` and ``bsk-sdk`` versions differ. Install matching
   versions and rebuild the extension.

``SWIG runtime version mismatch``
   The extension build is using a SWIG runtime epoch that differs from
   Basilisk. Use the SWIG version required by the matching ``bsk-sdk`` release;
   do not bypass this check.

``ModuleNotFoundError: cSysModel``
   Add the ``cSysModel`` alias to the extension's ``__init__.py`` before
   importing the compiled module wrapper.

``TypeError`` from ``AddModelToTask()``
   Ensure the SWIG interface uses ``%import "sys_model.i"`` rather than
   ``%include`` so the extension reuses Basilisk's existing ``SysModel`` Python
   type.

Custom message fields do not provide ``recorder()``
   Import the generated ``messaging`` package before importing extension module
   wrappers in ``__init__.py``.

CMake cannot find ``bsk-sdk``
   Confirm the intended virtual environment is active and that ``python``,
   ``bsk-sdk``, and the build command all use that environment.

Python still imports an older build
   Rebuild the wheel and install it with ``--force-reinstall``. Also check that
   another copy of the extension is not present earlier on ``sys.path``.

Publishing
----------

An extension wheel can be published to PyPI or a private package index. Keep
``bsk==2.X.Y`` as an exact runtime dependency so package installation cannot
silently pair the compiled extension with a different Basilisk ABI. Build and
test wheels for every supported platform and Python version.

When adding support for a newer Basilisk release, update both ``bsk-sdk`` and
``bsk`` in ``pyproject.toml``, rebuild all wheels, and run the extension test
suite against that release.
