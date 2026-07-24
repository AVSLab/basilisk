.. toctree::
   :hidden:

.. youtube:: t9VrmrVfJ0M
   :width: 560
   :height: 315

.. _configureBuild:

Building the Software Framework
===============================

This documents discusses the various  build options that can be
used when configuring and compiling Basilisk. The build system makes use of the ``conan``
`package manager <https://docs.conan.io/en/latest/>`__ which provides for cross platform manner support.
As a result the Basilisk build instructions are platform agnostic.

This page is broken up into distinct manners of building Basilisk starting with the simplest method that should work
for the majority of users.  This is followed by increasingly more detailed discussions on the build system for users
who want to customize.


One-Step Configuring and Building the Basilisk Framework
--------------------------------------------------------
If you just installed the Basilisk source code, or changed any source file, you will want to recreate the Basilisk IDE
project or makefile to compile it.  This creates or updates the project file to included the latest Basilisk source
code.  From the Basilisk root directory, this is done simply using::

    python3 conanfile.py

This one-line step will use ``conan`` to:

- pull and compile any resource dependencies such a protobuffer, etc.
- configure the Xcode (macOS) or Visual Studio (Windows) IDE project in the ``dist3`` folder,
  or create the makefile for Linux systems
- build the project.

By default the build is for Python3 with the support for :ref:`vizInterface`
included to enable recording data for or live-streaming to :ref:`Vizard <Vizard>`.

The script accepts the following options to customize this process.

.. _buildTable1Label:
.. list-table:: Options for One-Step Configure/Build Process
    :widths: 15 15 10 60
    :header-rows: 1

    * - Option
      - Values
      - Default
      - Description
    * - ``vizInterface``
      - Boolean
      - True
      - Includes the `Google Protobuffer <https://developers.google.com/protocol-buffers>`__ library to package
        up Vizard data messages, and the `Zero-MQ <https://zeromq.org>`__ library to communicate with Vizard.
    * - ``opNav``
      - Boolean
      - False
      - Includes `OpenCV <https://opencv.org>`__ library to create visual navigation modules that use OpenCV to
        process sensor images.  If this option is selected,
        then the dependencies of  ``vizInterface`` are also loaded as some components require the same libraries.
        Note that OpenCL related dependencies can take a while to compile, 10-20minutes is not unusual.  However,
        once install they don't need to be rebuilt unless ``.conan`` is deleted or the dependency changes.
    * - ``clean``
      -
      - None
      - If flag is set, this deletes the distribution folder ``dist3`` and Basilisk Numba cache
        artifacts to create a fresh setup and build
    * - ``buildProject``
      - Boolean
      - True
      - If set to True, this option will compile the project right away after creating the IDE or make file
    * - ``buildType``
      - Release, Debug
      - Release
      - Sets the build type.  This does not apply to the IDE project like Xcode and Visual Studio which
        control the build type through their interface.
    * - ``generator``
      - see `here <https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html>`__
      - ``XCode`` (macOS), ``Visual Studio 16 2019`` (Windows), ``None`` (Linux)
      - If not set the ``cmake`` build generator is automatically selected to be ``XCode`` for macOS,
        ``Visual Studio 16 2019`` on Windows, and ``None`` for Linux which create a make file on this platform.
        It can also be set through this
        flag.  If unsure what generators are supported on your platform, open a terminal window and
        type ``cmake --help`` to get a list of supported generator strings.
    * - ``autoKey``
      - String 's' or 'u'
      - Empty
      - This is used to automatically respond to the python packaging installation requests to install the
        package for the user (u) or system (s).
    * - ``allOptPkg``
      -
      - None
      - If flag is set the Basilisk python package depenencies to build documentation are installed
    * - ``pathToExternalModules``
      - String
      - Empty
      - path to external modules folder, see :ref:`buildExtModules`
    * - ``mujoco``
      - Boolean
      - False
      - :beta:`Mujoco Support` Includes the `MuJoCo <https://mujoco.org>`_ dependencies
    * - ``examples``
      - Boolean
      - True
      - Installs the optional Python dependencies used by Basilisk example scripts, such as ``scipy`` and ``numba``.
        Disable this flag if you want a leaner clone-based install and do not need the example-only Python packages.
    * - ``recorderPropertyRollback``
      - Boolean
      - False
      - Version 2.8 changed the output format when querying non-numeric types in message recorders.
        By using this flag, the previuos behavior is recovered. Note that this build flag will disappear
        in the near future.

Thus, for example, to create a build with ``opNav`` modes enabled, but no :ref:`vizInterface`, and using a
clean distribution folder, and that is built right away, you could use::

    python3 conanfile.py --clean --opNav True --vizInterface False --buildProject True

The ``buildProject`` argument here is optional as its default value is ``True``.
Likewise, the ``examples`` argument is optional as its default value is ``True``.
To skip the optional example Python packages during the clone-based install, use::

    python3 conanfile.py --examples False

.. warning::

    If you switch between building for release (default) and debug, you must re-run this command again.  Otherwise
    you will not have the correct conan dependencies included for your build type.  In the IDE like Xcode, for
    example, if you do a regular build you are building for debug, not for release.  Thus, be mindful of how
    you are building the code.

Inspecting the Build Toolchain
------------------------------
Every Basilisk build records the C and C++ compilers, build configuration, CMake generator, and versions of the
principal build tools in the installed Python package.  This information can be inspected when diagnosing a binary
or build issue.

For a concise, human-readable summary, use ``printBuildInfo()``::

    from Basilisk import printBuildInfo

    printBuildInfo()

This produces output similar to::

    Basilisk Build Information
      Version:        2.12.0 (plugin ABI 1)
      Target:         macOS arm64, 64-bit
      Build:          Release, Unix Makefiles
      C compiler:     AppleClang 21.0.0.21000101 (cc)
      C++ compiler:   AppleClang 21.0.0.21000101 (c++)
      C standard:     C17
      C++ standard:   C++17
      C++ ABI:        libc++ (210106, ABI 1), Itanium ABI
      C++ runtime:    system, exceptions, RTTI
      Python API:     0x03090000
      Eigen:          3.4.0, 16-byte max alignment, NEON
      SWIG runtime:   5
      Conan profile:  Release, C++17, libc++

    Build Tools
      CMake:          4.2.0
      Conan:          2.23.0
      SWIG:           4.4.1
      Python:         3.14.6

For programmatic inspection or custom formatting, use ``getBuildInfo()``.  It returns a copy of a versioned nested
dictionary with three principal sections::

    from Basilisk import getBuildInfo

    buildInfo = getBuildInfo()
    pluginAbiVersion = buildInfo["artifact"]["pluginAbiVersion"]
    standardLibrary = buildInfo["abi"]["cxx"]["standardLibrary"]["family"]
    compilerVersion = buildInfo["diagnostics"]["compilers"]["cxx"]["version"]

``artifact`` identifies the Basilisk version, source revision when available, and plugin ABI epoch.  The epoch is
incremented when Basilisk intentionally changes the C/C++ object contract exposed to SDK plugins.  It does not
promise compatibility between different Basilisk versions; the SDK's exact-version check remains required unless a
future compatibility policy explicitly relaxes it.

``abi`` is captured by C and C++ translation units compiled with the selected Basilisk build configuration.  It
records the actual target architecture, endianness, C and C++ language modes, compiler ABI, standard-library ABI and
debug modes, runtime linkage, exceptions, RTTI, Python ABI, SWIG runtime epoch, and Eigen alignment and vectorization
settings.  It also contains size, alignment, and field-offset canaries for important C, C++, messaging, and Eigen
types.  These values are suitable inputs to a future BSK-SDK compatibility policy.  Layout canaries detect common
binary mismatches but do not prove semantic compatibility.

The public ``architecture/utilities/bskAbiDescriptor.h`` header is the single source of truth for the descriptor and
plugin ABI versions, canary types, and compiler-side extraction rules.  It is included by the existing BSK-SDK
``architecture`` header synchronization, allowing Basilisk and an SDK plugin to compile the same contract rather
than maintaining parallel implementations.

``diagnostics`` contains values observed by CMake, requested Conan settings, and build-tool versions.  These remain
useful when reproducing a build, but they are not all binary-compatibility requirements.  For example, CMake and
Conan versions should not be compared as part of an SDK compatibility decision.  For Xcode and Visual Studio,
``diagnostics["build"]`` describes the multi-config generator while ``abi["build"]["configuration"]`` records the
configuration that actually compiled the installed descriptor.

The standard-library ``pprint`` module provides an indented view when all recorded fields should be displayed::

    from pprint import pprint
    from Basilisk import getBuildInfo

    pprint(getBuildInfo(), sort_dicts=False, width=100)

Absolute build paths and timestamps are intentionally omitted from this metadata.  Source revision is empty and
dirty state is reported as ``None`` when the source was built without Git metadata.  Source revision and dirty state
are captured when CMake configures the package metadata.  Local incremental module-only builds refresh the compiled
ABI descriptor, but they do not recapture Git provenance.  Reconfigure or do a clean package build before relying on
these fields for release provenance.


Doing Incremental Builds
------------------------
If you are developing new Basilisk capabilities you will be looking to do incremental builds.  Note that running
``python conanfile.py`` will delete the auto-created messaging related files and their compiled products.
Compiling the messaging related files is a large component of the Basilisk build time.  Thus, running
this command is required if you make changes to the message file definitions, or add a new message file.  However,
it is not the preferred manner to compile Basilisk if you just need to do an incremental build.

Rather, run ``python conanfile.py --buildProject False`` to create the IDE file for your platform
such as Xcode projeect file on macOS, MS Visual Studio project file on Windows, etc.  Next,
open the project file in your IDE and compile Basilisk there.  The initial build is a clean build and will take a
similar amount of time to compile the messaging related files.  However, after making changes to a particular module,
only this Basilisk module will need to be compiled and the compile times are drastically reduced.

Every loadable BSK module target depends on ABI metadata generation, so an incremental module-only build refreshes the
compiled descriptor for the selected configuration.  This refreshes the ABI details under ``getBuildInfo()["abi"]``;
the Git provenance fields under ``getBuildInfo()["artifact"]`` remain configure-time package metadata.  Internal
libraries consume the same ABI settings but do not carry this dependency because multi-configuration generators share
some of their generated sources.  Do not mix Debug and Release module binaries in one Basilisk package; switching
configurations requires rebuilding the complete package.


Speeding Up Builds with ``sccache``
------------------------------------
`sccache <https://github.com/mozilla/sccache>`__ is a compiler cache that caches individual C++ object
files by content hash.  When source files have not changed, sccache returns the cached object instead of
invoking the compiler, which can dramatically reduce clean-build times.

Install sccache with your system package manager, for example on macOS::

    brew install sccache

Then set the following environment variables before building::

    export CMAKE_C_COMPILER_LAUNCHER=sccache
    export CMAKE_CXX_COMPILER_LAUNCHER=sccache

With these set, any build invocation (``python3 conanfile.py``, ``pip install -e .``, etc.) will
automatically use sccache.  A warm cache reduces a full rebuild to only the linking step plus any
files that actually changed.

To inspect cache statistics after a build::

    sccache --show-stats

.. note::

    sccache is also enabled automatically in CI via ``CMAKE_C/CXX_COMPILER_LAUNCHER`` environment
    variables set by the GitHub Actions build workflow.


Running Project Tests
---------------------

The project employs two testing frameworks, specifically `Pytest <https://pytest.org>`__ for python
executed tests and `Google Test <https://github.com/google/googletest>`__ for C/C++ executed tests.

To run all tests execute the following from the project root directory

.. code-block:: console

    python run_all_test.py

To run only the python test use the following commands.

.. code-block:: console

    cd src
    pytest

To run only the C/C++ tests use

.. code-block:: console

    cd dist3
    ctest

or on macOS ``ctest -C <Release or Debug>``.
