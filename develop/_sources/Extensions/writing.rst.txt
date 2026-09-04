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

This guide walks through the complete extension workflow: build the working
example, understand its files, create a package, add compiled modules, generate
a custom message, build a wheel, and run tests. Read :ref:`bskExtensions` first
for the architectural difference between an extension and an integrated
external-folder build.

Extension modules follow the same language conventions as built-in Basilisk
modules. See :ref:`cppModules`, :ref:`cModules`, and :ref:`rustModules` for
module lifecycle, messages, configuration, and testing conventions. This page
focuses on the out-of-tree build and packaging details.

You need Git, a supported Python version, a C++17 compiler, CMake 3.26 or
newer, and Ninja or another CMake build tool. The Python commands below install
the Python-provided CMake and Ninja packages for convenience. A source project
containing a Rust module also needs Rust 1.89 or newer and Cargo; install both
with the `rustup installer <https://rustup.rs/>`__. C/C++-only extensions do
not need Rust.

Build the Working Example First
-------------------------------

Before creating a new project, build the SDK example unchanged. This verifies
the compiler, Python environment, Basilisk installation, SDK, SWIG runtime,
message generator, and wheel tooling together.

Open a terminal in a new, otherwise empty parent directory where the SDK
checkout and its virtual environment can live. Create the environment there.
The activation command shown is for macOS and Linux; on Windows use
``.venv\Scripts\activate``.

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install "bsk[all]" bsk-sdk build scikit-build-core pytest \
     "cmake>=3.26" "ninja>=1.5"

With that environment still active, verify that Basilisk and ``bsk-sdk``
target the same version. This command can be run from the same parent
directory:

.. code-block:: bash

   python -c "import Basilisk, bsk_sdk; print('Basilisk:', Basilisk.__version__); print('bsk-sdk:', bsk_sdk.bsk_version())"

From that parent directory, obtain the SDK repository. The ``cd`` command then
makes the repository root the working directory for the remaining example
commands:

.. code-block:: bash

   git clone --recurse-submodules https://github.com/AVSLab/bsk_sdk.git
   cd bsk_sdk
   python -m build --wheel --no-isolation examples/custom-atm-extension
   python -m pip install examples/custom-atm-extension/dist/*.whl
   python -c "import Basilisk, numba, custom_atm; from custom_atm import numbaAtmosphere, rustAtmosphere"
   python -m pytest examples -v

The example contains C++, C, Rust, and pure-Python Numba modules. Its Rust
module exchanges both a built-in Basilisk message and an extension-defined
message. Core Basilisk is not rebuilt. After this succeeds, use the following
sections to adapt the example or create a new extension.

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
   |-- Cargo.toml                       # Rust extensions only
   |-- Cargo.lock                       # Rust extensions only
   |-- exampleRustModule/               # Optional native Rust module
   |   |-- Cargo.toml
   |   |-- build.rs
   |   |-- exampleRustModule.rs
   |   `-- _UnitTest/
   |       `-- test_exampleRustModule.py
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

Create the top-level ``pyproject.toml`` in the extension project root and
replace every ``2.X.Y`` with the Basilisk release being targeted:

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
Keep these exact versions in the checked-in project metadata when moving the
extension to a different Basilisk release.

Write the Module
----------------

Write the module header and implementation using normal Basilisk patterns. A
C++ module can inherit directly from ``SysModel`` or from an SDK-supported
Basilisk base class such as ``AtmosphereBase`` or ``DynamicEffector``.

Add each required include near the top of the extension module file that uses
the corresponding type. For example,
``exampleCppModule/exampleCppModule.h`` might begin with:

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

The SWIG interface exposes the compiled C++ class to Python. Create
``exampleCppModule/exampleCppModule.i``; a minimal file for a ``SysModel``
subclass is:

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
be included locally. In the same ``exampleCppModule/exampleCppModule.i`` file,
place that include after the ``sys_model.i`` import and before the extension
class header:

.. code-block:: swig

   %import "sys_model.i"
   %include "simulation/environment/_GeneralModuleFiles/atmosphereBase.h"
   %include "customAtmosphere.h"

The complete SDK example demonstrates this pattern with ``AtmosphereBase``.

Configure CMake
---------------

Create the top-level ``CMakeLists.txt`` in the extension project root. The
first part of the following block locates the SDK installed in the active
Python environment; the second part defines extension targets.

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

   bsk_add_extension_compatibility_guard(
     EXTENSION_NAME "my_bsk_extension"
     OUTPUT_DIR "${EXTENSION_PKG_DIR}"
   )

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

In the top-level ``CMakeLists.txt``, add message generation after
``EXTENSION_PKG_DIR`` is defined:

.. code-block:: cmake

   bsk_generate_messages(
     OUTPUT_DIR "${EXTENSION_PKG_DIR}/messaging"
     MSG_HEADERS
       "${CMAKE_CURRENT_SOURCE_DIR}/messages/MyStatusMsgPayload.h"
   )

This produces a ``my_bsk_extension.messaging`` package containing
``MyStatusMsgPayload``, ``MyStatusMsg``, and recorder support. Add
``GENERATE_C_INTERFACE`` when generating the C message interface needed by an
extension C or Rust module.

If the module's public interface exposes the custom message, edit
``exampleCppModule/exampleCppModule.i`` and make the payload type visible
before the final include of the module header:

.. code-block:: swig

   %include "MyStatusMsgPayload.h"
   struct MyStatusMsg_C;
   %include "exampleCppModule.h"

Changing this payload header regenerates and recompiles the affected extension
targets. It does not rebuild core Basilisk or its global message package.

Add a Rust Module
-----------------

This section covers only the extension build boundary. Implement the module,
its lifecycle, ports, configuration, and tests as described in
:ref:`rustModules`. The ``rustAtmosphere`` package in the SDK example is a
working extension-specific reference.

Create one Cargo workspace at the extension root and list every Rust module as
a member. The installed Basilisk wheel does not need to have been compiled with
in-tree Rust modules enabled: the extension compiles and carries its own Rust
module code, while ``bsk-sdk`` supplies the compatible C-message interfaces.

From the extension project root, with the intended virtual environment active,
query the Basilisk release, minimum compiler, and exact support-crate version
from the installed SDK rather than copying values from another release:

.. code-block:: console

   python -c "import bsk_sdk; print('Basilisk:', bsk_sdk.bsk_version()); print('Rust:', bsk_sdk.rust_minimum_version()); print('bsk-* crates:', bsk_sdk.rust_support_crate_version())"

Use those values in the extension project's top-level ``Cargo.toml``, which is
the Cargo workspace manifest. For example, the Basilisk 2.12 SDK release uses:

.. code-block:: toml

   [workspace]
   resolver = "2"
   members = ["exampleRustModule"]

   [workspace.package]
   license = "ISC"
   rust-version = "1.89"

   [workspace.dependencies]
   bsk-build = { version = "=0.1.0", git = "https://github.com/AVSLab/basilisk", tag = "v2.12.0" }
   bsk-messages = { version = "=0.1.0", git = "https://github.com/AVSLab/basilisk", tag = "v2.12.0" }

   [profile.dev]
   panic = "unwind"

   [profile.release]
   panic = "unwind"

Replace ``ISC`` with the extension's SPDX license identifier if it uses a
different license. The Git tag must match ``bsk_sdk.bsk_version()``. The
separate ``0.1.0`` value
is the exact version of the ``bsk-*`` Rust support crates, reported by
``bsk_sdk.rust_support_crate_version()``; it is intentionally independent of
the Basilisk release number. The generated C boundary has its own internal ABI
version, which is checked automatically during compilation. Cargo downloads
the tagged source once into its global cache. During BSK-SDK branch
development, ``tools/sync_all.py`` replaces these Git dependencies in the SDK
example with paths to the selected Basilisk checkout.

Each module keeps the normal ``Cargo.toml``, ``build.rs``, and matching
``exampleRustModule.rs`` source layout. Its manifest must set
``[package.metadata.basilisk] module = true`` and build one ``staticlib``
target. Commit the workspace's single ``Cargo.lock`` so local and wheel builds
use the reviewed dependency resolution.

Create ``exampleRustModule/Cargo.toml`` for the module itself. This is separate
from the top-level workspace ``Cargo.toml`` shown above. The module manifest
uses the shared runtime dependencies from the workspace and enables code
generation only for its build script:

.. code-block:: toml

   [package]
   name = "exampleRustModule"
   version = "0.1.0"
   edition = "2024"
   rust-version.workspace = true
   license.workspace = true

   [package.metadata.basilisk]
   module = true

   [lib]
   path = "exampleRustModule.rs"
   crate-type = ["staticlib"]

   [dependencies]
   bsk-build.workspace = true
   bsk-messages.workspace = true

   [build-dependencies]
   bsk-build = { version = "=0.1.0", git = "https://github.com/AVSLab/basilisk", tag = "v2.12.0", default-features = false, features = ["codegen"] }

The repeated build dependency deliberately disables the Basilisk runtime while
the host build script runs. Keep its version and Git tag aligned with the
workspace values above. The SDK's `rustAtmosphere module manifest
<https://github.com/AVSLab/bsk_sdk/blob/master/examples/custom-atm-extension/rustAtmosphere/Cargo.toml>`_
is the maintained copyable example; BSK-SDK synchronization replaces its Git
source with the selected local checkout during branch development.

In the extension project's top-level ``CMakeLists.txt``, after all calls to
``bsk_generate_messages()``, ask the SDK to discover and build every marked
workspace member:

.. code-block:: cmake

   bsk_add_rust_workspace(
     MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
     OUTPUT_DIR "${EXTENSION_PKG_DIR}"
   )

No Rust-specific ``bsk-sdk`` build option is needed. Calling this helper is the
opt-in: CMake locates Cargo, obtains the SDK's pinned Corrosion integration,
generates each module's C/SWIG boundary, and places the compiled Python modules
in the extension package. If the extension has no marked Rust workspace and
does not call this helper, Cargo and Rust are never required.

Rust modules use Basilisk C message types. Built-in C message interfaces ship
with the SDK. For an extension-owned payload, call
``bsk_generate_messages(GENERATE_C_INTERFACE ...)`` before the Rust workspace
helper so the generated message type is available to ``bsk-messages``.

Initialize the Extension's Python Package
-----------------------------------------

For a ``SysModel``-derived module with generated messages, run the generated
compatibility guard before importing any native wrapper, then preserve the
following import order in ``my_bsk_extension/__init__.py``:

.. code-block:: python

   import sys

   from ._bsk_compatibility import check_basilisk_compatibility as _check_basilisk

   _check_basilisk()
   del _check_basilisk

   from Basilisk.architecture import cSysModel as _cSysModel

   sys.modules.setdefault("cSysModel", _cSysModel)

   from . import messaging
   from . import exampleCppModule
   from . import exampleRustModule

   __all__ = ["exampleCppModule", "exampleRustModule", "messaging"]

The compatibility guard compares the exact Basilisk version and extension ABI
embedded during the wheel build with the installed runtime. It raises a
descriptive ``ImportError`` before incompatible native code is loaded. The
``cSysModel`` alias must exist before importing the generated module wrapper
because SWIG resolves that base module while defining the Python class.
Importing ``messaging`` before the module wrapper registers the custom
``Message<T>`` and ``Recorder<T>`` proxy classes. Without this ordering,
custom message fields may lack methods such as ``recorder()``.

If the extension has no custom messages, omit the ``messaging`` import. If it
does not expose a ``SysModel``-derived class, the ``cSysModel`` alias may not be
needed.

Build and Install the Extension
-------------------------------

For the first development build, open a terminal in the extension project root
(the directory containing ``pyproject.toml`` and ``CMakeLists.txt``), activate
its virtual environment, install the selected Basilisk and SDK versions, and
build without isolation. If the extension owns a Rust module, verify
``rustc --version`` reports Rust 1.89 or newer first:

.. code-block:: bash

   python -m pip install "bsk[all]==2.X.Y" "bsk-sdk==2.X.Y"
   python -m pip install build scikit-build-core pytest
   python -m build --wheel --no-isolation
   python -m pip install --force-reinstall dist/*.whl

The installed extension wheel contains native code and is specific to its
operating system, processor architecture, Python compatibility, and targeted
Basilisk version.

From the same extension project root, an editable install is useful while
iterating:

.. code-block:: bash

   python -m pip install --no-build-isolation -e .

For release artifacts, also run a normal isolated build from the extension
project root. This verifies that ``pyproject.toml`` declares every build
dependency:

.. code-block:: bash

   python -m build --wheel

Run the Tests
-------------

Place module tests in the module's ``_UnitTest`` directory. From the extension
project root, run them against the installed extension wheel:

.. code-block:: bash

   python -m pytest exampleCppModule/_UnitTest/ -v

At minimum, test that:

* the extension package and compiled module import successfully;
* the module can be added to a Basilisk task;
* built-in Basilisk input and output messages connect correctly;
* custom messages can be written, read, and recorded; and
* the module behavior is validated against known results.

For a Rust module, also run ``cargo test --workspace --locked`` with the SDK's
built-in and extension-generated C-message directories supplied as described
by the extension example. Cargo tests cover Rust-native logic, while the Python
tests remain responsible for the installed wrapper, scheduler lifecycle, and
message integration.

Use the same unit and integration testing standards as a built-in Basilisk
module. Rebuild and reinstall the wheel after changing native source or message
headers.

Use the Extension in a Simulation
---------------------------------

Once installed, import the extension by its own package name and schedule the
module normally. Put this code in a simulation script, for example
``examples/scenarioUseMyExtension.py``; it does not belong in
``my_bsk_extension/__init__.py``:

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

``Cargo was not found on PATH``
   The extension declares a Rust workspace, but Rust is not installed in the
   active build environment. Install Rust 1.89 or newer with ``rustup``, open a
   new terminal, and rebuild. This error does not apply to C/C++-only
   extensions.

``bsk-build is incompatible with this Basilisk/bsk-sdk Rust module ABI``
   The Rust support-crate version does not match the installed Basilisk and
   ``bsk-sdk`` release. Confirm both the exact support-crate version and
   Basilisk Git tag, update ``Cargo.lock``, and rebuild the extension wheel.

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
silently pair the compiled extension with a different Basilisk ABI. Also use
``bsk_add_extension_compatibility_guard()`` so a forced upgrade or other later
environment change fails on import before native code is loaded. Build and test
wheels for every supported platform and Python version.

The compiled Rust archive is part of the extension's native Python module; end
users installing a compatible prebuilt wheel do not need Rust, Cargo,
Corrosion, or ``bsk-sdk``. Extension publishers remain responsible for
including the notices required by statically linked Rust dependencies. The SDK
ships the same pinned ``cargo-about`` policy and deterministic report generator
used by Basilisk. Run the following commands from the extension project root.
They install the SDK-selected tool version and write the report directly into
the import package. Replace ``my_bsk_extension`` with the import-package name
and ``my-bsk-extension`` with the distribution name chosen for the extension:

.. code-block:: console

   python -c "import bsk_sdk, subprocess; subprocess.run(['cargo', 'install', 'cargo-about', '--version', '=' + bsk_sdk.rust_license_tool_version(), '--locked', '--features', 'cli'], check=True)"
   python -c "import bsk_sdk, subprocess, sys; subprocess.run([sys.executable, bsk_sdk.rust_license_generator(), '--manifest-path', 'Cargo.toml', '--config', bsk_sdk.rust_license_config(), '--output', 'my_bsk_extension/RUST-THIRD-PARTY.txt', '--project-name', 'my-bsk-extension', '--require-tool'], check=True)"

Commit the generated report and include it in the extension wheel. In CI,
repeat the second command with ``'--check'`` added to the argument list after
``'--require-tool'``. This checks the committed file without modifying it.
Regenerate the report whenever ``Cargo.lock`` changes. The generator's
project-license text defaults to Basilisk's ISC license; an extension using
another license should pass a suitable sentence with ``--project-license``.

When adding support for a newer Basilisk release, update both ``bsk-sdk`` and
``bsk`` in ``pyproject.toml``, rebuild all wheels, and run the extension test
suite against that release.
