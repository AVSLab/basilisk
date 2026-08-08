.. toctree::
   :hidden:

.. _configureBuildConan:

Advanced: Directly Building the Software Framework Using Conan
==============================================================

.. warning::

    This method of compiling Basilisk is not typically required and should only
    be attempted by advanced users familiar with both ``conan`` and ``cmake``.
    See :ref:`configureBuild` for documentation on the regular build process.


Configuring and Building with ``conan`` Commands
------------------------------------------------
Calling ``conanfile.py`` with Python dispatches one Conan command that installs missing dependencies, generates
the build files, and calls the Basilisk recipe's ``build()`` method. This section outlines the equivalent direct
Conan command for advanced users.

.. note::

    All commands are called from the Basilisk root directory.

Installing Dependencies and Building Basilisk
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The command, in its minimal form, is::

    conan build . --build=missing -s compiler.cppstd=17 -s:b compiler.cppstd=17

This command creates the ``dist3`` distribution folder when needed, resolves the required third-party resources,
compiles dependencies whose binaries are missing, generates the CMake files, and builds Basilisk. All build files
are stored in ``dist3``. It does not install or modify packages in the active Python environment. Install the
requirements and create the editable Basilisk installation separately, as described in the platform-specific
installation instructions. Applying C++17 to both the host and build profiles lets Conan match cached build tools
directly instead of searching compatible configurations or querying a remote for an avoidable profile mismatch.

There are several options that can be provided to this ``conan build`` command as shown in the following table.
Note that the option names for groupings of Basilisk modules are the same as with the one-step build above. The
``&:`` prefix scopes each option to the Basilisk consumer recipe. Keep the option expression in double quotes because
``&`` is a shell control character.

.. list-table:: Options for ``conan build``
    :widths: 15 15 10 60
    :header-rows: 1

    * - Option
      - Values
      - Default
      - Description
    * - ``-o "&:vizInterface=<value>"``
      - Boolean
      - True
      - Include :ref:`vizInterface` in the configuration and build
    * - ``-o "&:opNav=<value>"``
      - Boolean
      - False
      - Include the `OpenCV <https://opencv.org>`__ library dependent Basilisk modules.
    * - ``-o "&:clean=<value>"``
      - Boolean
      - False
      - Delete the configured build folder and Basilisk Numba cache artifacts before configuring to yield
        a fresh build. The build folder must either be empty or contain recognizable CMake/Conan build output.
    * - ``-o "&:buildFolder=<path>"``
      - Directory path
      - ``dist3``
      - Select the build folder. Relative paths are resolved from the Basilisk source directory.
    * - ``-o "&:buildProject=<value>"``
      - Boolean
      - True
      - Will build the project executable after the configuration step
    * - ``-o "&:buildTesting=<value>"``
      - Boolean
      - True
      - Builds the native GoogleTest executables and registers them with CTest
    * - ``-s build_type``
      - Release, Debug
      - Release
      - Specifies the build type
    * - ``-o "&:strictWarnings=<value>"``
      - Boolean
      - False
      - Enables the additional Basilisk compiler diagnostics described in
        :ref:`strictCompilerWarnings`
    * - ``-o "&:generator=<value>"``
      - see `here <https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html>`__
      - Automatically selected
      - Used to specify a specific ``cmake`` generator.  See discussion in Table :ref:`buildTable1Label`.

Thus, using the same build example as in the one-step section, to create a build with ``opNav`` modes enabled,
but no :ref:`vizInterface`, and using a clean distribution folder, and that is built right away, you could use::

    conan build . --build=missing -s compiler.cppstd=17 -s:b compiler.cppstd=17 \
        -o "&:clean=True" -o "&:buildProject=True" -o "&:opNav=True" -o "&:vizInterface=False"

Note how much more verbose this is, but it gives you full control if you want to store the compiled binaries and
cmake files in directories other than ``dist3/conan``.


Running ``cmake`` Directly
--------------------------

Advanced users can still run ``conan install . --build=missing`` separately when they intentionally want to
prepare dependencies and toolchain files without calling the recipe's ``build()`` method. They must then configure
CMake with the generated Conan toolchain before running CMake directly from the ``dist3`` distribution folder.
This optional split workflow is not used by ``python conanfile.py``.

The following table summarizes the optional Basilisk related flags that can be provided to ``cmake``.  If
they are not used, then the shown default behaviors are used.

.. list-table:: ``cmake`` Basilisk Build Flags
    :widths: 25 15 70
    :header-rows: 1

    * - cmake Flag
      - Default
      - Description
    * - ``BUILD_VIZINTERFACE``
      - ``ON``
      - will create :ref:`vizInterface`
    * - ``BUILD_OPNAV``
      - ``OFF``
      - will create the OpenCL dependent optical navigation related modules
    * - ``BUILD_TESTING``
      - ``ON``
      - fetches GoogleTest and builds the native C++ test executables
    * - ``BSK_STRICT_WARNINGS``
      - ``OFF``
      - enables additional compiler diagnostics for Basilisk C and C++ sources

macOS Example
~~~~~~~~~~~~~
::

    $ cmake ../src -G Xcode -DBUILD_OPNAV=ON

Linux Example
~~~~~~~~~~~~~
::

    $ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../src

Windows Examples
~~~~~~~~~~~~~~~~
- Example direct build on Windows::

    cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win<arch>" ../src -DCMAKE_BUILD_TYPE=Release
    cmake --build . --target ALL_BUILD --config Release

- Example ``cmake`` commands using x86::

    cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win32" ../src -DCMAKE_BUILD_TYPE=Release

  .. list-table:: MSVC Mapping
     :widths: 25 25
     :header-rows: 1

     * - MSVC Product Year
       - MSVC Version
     * - 2022
       - 17
     * - 2019
       - 16
     * - 2017
       - 15.9
     * -
       - 15.8
     * -
       - 15.7
     * -
       - 15.6
     * -
       - 15.5
     * -
       - 15.4 - 15.3
     * -
       - 15.2 - 15.0
     * - 2015
       - 14
     * - 2013
       - 12
     * - 2012
       - 11



Example build commands for Arch x86, MSVC Year 2017, MSVC Version 15::

    cmake -G “Visual Studio 15 2017 Win32” ../src

Example build commands forArch x64, MSVC Year 2019, MSVC Version 16::

    cmake -G “Visual Studio 16 2019” -A x64 ../src -DCMAKE_BUILD_TYPE=Release

    cmake -G “Visual Studio 15 2017 Win64” ../src -DCMAKE_BUILD_TYPE=Release
