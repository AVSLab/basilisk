.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingExtensions:

Writing a Basilisk Extension
============================

.. sidebar:: BSK-SDK Repo

    The BSK-SDK source code is found on GitHub at `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_.

An extension uses exactly the same C++ or C module structure as built-in
Basilisk modules — see :ref:`cppModules` and :ref:`cModules` for how to
write the module itself.  This page covers only the extension-specific
packaging: the project layout, ``CMakeLists.txt``, and ``pyproject.toml``.

A complete working example is in the
`bsk-sdk repository <https://github.com/AVSLab/bsk_sdk/tree/master/examples/custom-atm-extension>`_.

Prerequisites
-------------

- Basilisk installed: ``pip install "bsk[all]"``
- The SDK: ``pip install bsk-sdk``
- CMake ≥ 3.26 and a C++17 compiler (same requirement as Basilisk itself)
- ``scikit-build-core`` and ``build``: ``pip install scikit-build-core build``

Extension Layout
----------------

Every extension needs a Python package directory (``my_extension/`` below). This
is the importable namespace your users will ``import`` from, and where both
compiled SWIG extensions and pure-Python modules are installed.  C++/SWIG
source trees live alongside it at the repo root and get compiled into that
package at build time.  Pure-Python modules can be added directly inside the
package directory with no extra build steps:

.. code-block:: text

    my-extension/
    ├── pyproject.toml
    ├── CMakeLists.txt
    ├── my_extension/                   # Python package (SWIG outputs install here too)
    │   ├── __init__.py
    │   └── examplePythonModule.py      # (optional) pure-Python modules
    ├── messages/                       # (optional) custom message definitions
    │   └── MyMsgPayload.h
    └── exampleCppModule/               # C++/SWIG module source
        ├── exampleCppModule.h
        ├── exampleCppModule.cpp
        ├── exampleCppModule.i
        └── _UnitTest/
            └── test_exampleCppModule.py

SWIG Interface
--------------

The ``.i`` file is the same as any BSK module.  For C++ use
``swig_common_model.i``, for C use ``swig_c_wrap.i``:

.. code-block:: swig

    %module exampleCppModule
    %{
    #include "exampleCppModule.h"
    %}

    %include "swig_common_model.i"   /* C++ modules */
    /* %include "swig_c_wrap.i"      C modules — use %c_wrap_2 instead */
    %include "exampleCppModule.h"

If subclassing a BSK base class, ``%include`` its ``.i`` file before yours:

.. code-block:: swig

    %include "swig_common_model.i"
    %include "simulation/environment/_GeneralModuleFiles/atmosphereBase.i"
    %include "customAtmosphere.h"

CMakeLists.txt
--------------

The ``CMakeLists.txt`` has a boilerplate section (copy as-is) and an
extension-specific section:

.. code-block:: cmake

    # ==========================================================================
    # Boilerplate: copy as-is
    # ==========================================================================
    cmake_minimum_required(VERSION 3.26)
    project(my_extension LANGUAGES C CXX)

    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c
        "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
      OUTPUT_VARIABLE bsk_sdk_dir
      RESULT_VARIABLE rc
    )
    if(NOT rc EQUAL 0 OR bsk_sdk_dir STREQUAL "")
      message(FATAL_ERROR "bsk-sdk not found — is it installed in this Python environment?")
    endif()
    file(TO_CMAKE_PATH "${bsk_sdk_dir}" bsk_sdk_dir)
    set(bsk-sdk_DIR "${bsk_sdk_dir}")
    find_package(bsk-sdk CONFIG REQUIRED)

    set(PKG_DIR "${SKBUILD_PLATLIB_DIR}/my_extension")

    # ==========================================================================
    # Extension-specific configuration
    # ==========================================================================

    file(GLOB EXTENSION_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule/*.cpp")

    # If subclassing a BSK base class (AtmosphereBase, DynamicEffector, etc.),
    # bsk_add_swig_module automatically links the required SDK runtime sources —
    # no manual list(APPEND) is needed.  See the bsk-sdk example.
    bsk_add_swig_module(
      TARGET exampleCppModule
      INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule/exampleCppModule.i"
      SOURCES   ${EXTENSION_SOURCES}
      INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/exampleCppModule"
        "${CMAKE_CURRENT_SOURCE_DIR}/messages"
      OUTPUT_DIR "${PKG_DIR}"
    )

    # Optionally generate Python bindings for custom messages
    bsk_generate_messages(
      OUTPUT_DIR "${PKG_DIR}/messaging"
      MSG_HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/messages/MyMsgPayload.h"
    )

pyproject.toml
--------------

.. code-block:: toml

    [build-system]
    requires = ["scikit-build-core>=0.9.3", "bsk-sdk==2.X.Y", "bsk==2.X.Y", "swig==4.4.1"]
    build-backend = "scikit_build_core.build"

    [project]
    name = "my-extension"
    version = "1.0.0"
    requires-python = ">=3.9"
    dependencies = ["bsk==2.X.Y"]

    [tool.scikit-build]
    wheel.packages = ["my_extension"]

.. note::

   ``bsk-sdk``, ``bsk``, and the runtime ``dependencies`` entry must all be
   **pinned to the same version**.  The SDK compiles BSK sources into your
   extension at build time, so mismatched versions will produce a CMake error.
   Replace ``2.X.Y`` with the Basilisk version you are targeting.

Building and Installing
-----------------------

.. code-block:: bash

    # Development install
    pip install --no-build-isolation -e .

    # Build a distributable wheel
    python -m build --wheel
    pip install dist/*.whl

    # Run unit tests
    pytest exampleCppModule/_UnitTest/ -v

Publishing to PyPI
------------------

An extension is a standard Python wheel.  Declare ``bsk`` as a runtime
dependency so end users get Basilisk with ``pip install my-extension``.
``bsk-sdk`` is only needed at build time and does not need to be
declared as a runtime dependency.
