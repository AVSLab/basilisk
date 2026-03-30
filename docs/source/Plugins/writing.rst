.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingPlugins:

Writing a Basilisk Plugin
=========================

.. sidebar:: BSK-SDK Repo

    The plugin source code is found on GitHub at `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_.

A plugin uses exactly the same C++ or C module structure as built-in
Basilisk modules — see :ref:`cppModules` and :ref:`cModules` for how to
write the module itself.  This page covers only the plugin-specific
packaging: the project layout, ``CMakeLists.txt``, and ``pyproject.toml``.

A complete working example is in the
`bsk-sdk repository <https://github.com/AVSLab/bsk_sdk/tree/main/examples/custom-atm-plugin>`_.

Prerequisites
-------------

- Basilisk installed: ``pip install bsk``
- The SDK: ``pip install bsk-sdk``
- CMake ≥ 3.26 and a C++17 compiler (same requirement as Basilisk itself)
- ``scikit-build-core`` and ``build``: ``pip install scikit-build-core build``

Plugin Layout
-------------

Follow the same folder convention as BSK — one folder per module, named
identically to the module:

.. code-block:: text

    my-plugin/
    ├── pyproject.toml
    ├── CMakeLists.txt
    ├── messages/               # (optional) custom message definitions
    │   └── MyMsgPayload.h
    └── myModule/
        ├── myModule.h
        ├── myModule.cpp
        ├── myModule.i
        └── _UnitTest/
            └── test_myModule.py

SWIG Interface
--------------

The ``.i`` file is the same as any BSK module.  For C++ use
``swig_common_model.i``, for C use ``swig_c_wrap.i``:

.. code-block:: swig

    %module myModule
    %{
    #include "myModule.h"
    %}

    %include "swig_common_model.i"   /* C++ modules */
    /* %include "swig_c_wrap.i"      C modules — use %c_wrap_2 instead */
    %include "myModule.h"

If subclassing a BSK base class, ``%include`` its ``.i`` file before yours:

.. code-block:: swig

    %include "swig_common_model.i"
    %include "simulation/environment/_GeneralModuleFiles/atmosphereBase.i"
    %include "customAtmosphere.h"

CMakeLists.txt
--------------

The ``CMakeLists.txt`` has a boilerplate section (copy as-is) and a
plugin-specific section:

.. code-block:: cmake

    # ==========================================================================
    # Boilerplate: copy as-is
    # ==========================================================================
    cmake_minimum_required(VERSION 3.26)
    project(my_plugin LANGUAGES C CXX)

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

    set(PKG_DIR "${SKBUILD_PLATLIB_DIR}/my_plugin")

    # ==========================================================================
    # Plugin-specific configuration
    # ==========================================================================

    file(GLOB PLUGIN_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/myModule/*.cpp")

    # If subclassing a BSK base class (AtmosphereBase, DynamicEffector, etc.),
    # add its implementation from the SDK:
    # list(APPEND PLUGIN_SOURCES "${BSK_SDK_RUNTIME_MIN_DIR}/atmosphereBase.cpp")

    bsk_add_swig_module(
      TARGET myModule
      INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/myModule/myModule.i"
      SOURCES   ${PLUGIN_SOURCES}
      INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/myModule"
        "${CMAKE_CURRENT_SOURCE_DIR}/messages"
      LINK_LIBS bsk::plugin
      OUTPUT_DIR "${PKG_DIR}"
    )

    # Optional: generate Python bindings for custom messages
    bsk_generate_messages(
      OUTPUT_DIR "${PKG_DIR}/messaging"
      MSG_HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/messages/MyMsgPayload.h"
    )

pyproject.toml
--------------

.. code-block:: toml

    [build-system]
    requires = ["scikit-build-core>=0.9"]
    build-backend = "scikit_build_core.build"

    [project]
    name = "my-plugin"
    version = "1.0.0"
    requires-python = ">=3.9"
    dependencies = ["bsk"]

    [tool.scikit-build]
    wheel.packages = []

Building and Installing
-----------------------

.. code-block:: bash

    # Development install
    pip install --no-build-isolation -e .

    # Build a distributable wheel
    python -m build --wheel
    pip install dist/*.whl

    # Run unit tests
    pytest myModule/_UnitTest/ -v

Publishing to PyPI
------------------

A plugin is a standard Python wheel.  Declare ``bsk`` as a runtime
dependency so end users get Basilisk with ``pip install my-plugin``.
``bsk-sdk`` is only needed at build time and does not need to be
declared as a runtime dependency.
