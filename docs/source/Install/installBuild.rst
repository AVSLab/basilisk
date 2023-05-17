.. toctree::
   :hidden:

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/t9VrmrVfJ0M" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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
      - If flag is set, this forces the distribution folder ``dist3`` to be deleted to create a fresh setup and build
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
      - If flag is set the all optional Basilisk python package depenencies are installed
    * - ``pathToExternalModules``
      - String
      - Empty
      - path to external modules folder, see :ref:`buildExtModules`

Thus, for example, to create a build with ``opNav`` modes enabled, but no :ref:`vizInterface`, and using a
clean distribution folder, and that is built right away, you could use::

    python3 conanfile.py --clean --opNav True --vizInterface False --buildProject True

The ``buildProject`` argument here is optional as its default value is ``True``.

.. warning::

    If you switch between building for release (default) and debug, you must re-run this command again.  Otherwise
    you will not have the correct conan dependencies included for your build type.  In the IDE like Xcode, for
    example, if you do a regular build you are building for debug, not for release.  Thus, be mindful of how
    you are building the code.

Configuring and Building with ``conan`` Commands
------------------------------------------------
Calling ``conanfile.py`` with python executes a method that calls two separate ``conan`` function to setup and
configure process, and build the executable if desired.  This section outlines this 2-step build process for
those that seek to build it this way.

.. note::

    All commands are called from the Basilisk root directory.

Step 1: Installing Basilisk Dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The first command, in its minimalist form, is::

    conan install . -if dist3/conan --build=missing

This conan command will create the distribution folder, ``dist3`` in the above case, if needed, collect all the
require Basilisk 3rd party resources and compile them if their binaries are missing.  The cmake files to access these
3rd party libraries are stored in ``dist3/conan``.

There are several options that can be provided to this ``conan install`` command as shown in the following table.
Note that the option names for groupings of Basilisk modules are the same as with the one-step build above.

.. list-table:: Options for ``conan install`` Step
    :widths: 15 15 10 60
    :header-rows: 1

    * - Option
      - Values
      - Default
      - Description
    * - ``-o vizInterface``
      - Boolean
      - True
      - Include :ref:`vizInterface` in the configuration and build
    * - ``-o opNav``
      - Boolean
      - False
      - Include the `OpenCV <https://opencv.org>`__ library dependent Basilisk modules.
    * - ``-o clean``
      - Boolean
      - False
      - Delete the distribution folder before configuring to yield a fresh build
    * - ``-o buildProject``
      - Boolean
      - True
      - Will build the project executable after the configuration step
    * - ``-s build_type``
      - Release, Debug
      - Release
      - Specifies the build type
    * - ``-o generator``
      - see `here <https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html>`__
      - ``XCode`` (macOS), ``Visual Studio 16 2019`` (Windows), ``None`` (Linux)
      - Used to specify a specific ``cmake`` generator.  See discussion in Table :ref:`buildTable1Label`.
    * - ``-o autoKey``
      - String 's' or 'u'
      - Empty
      - This is used to automatically respond to the python packaging installation requests to install the
        package for the user (u) or system (s).
    * - ``-o allOptPkg``
      - Boolean
      - False
      - Install all of the optional Basilisk python package dependencies

Thus, using the same build example as in the one-step section, to create a build with ``opNav`` modes enabled,
but no :ref:`vizInterface`, and using a clean distribution folder, and that is built right away, you could use::

    conan install . -if dist3/conan --build=missing -o clean=True -o buildProject=True -o opNav=True -o vizInterface=False

Note how much more verbose this is, but it gives you full control if you want to store the compiled binaries and
cmake files in directories other than ``dist3/conan``.

Step 2: Creating the IDE Project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The final step is to create the IDE project file and possibly build the executables directly.
At this stage there are no options to be provided.  This step is done with::

    conan build . -if dist3/conan

.. warning::

    If you want the ``conan build`` command to automatically compile the code, the ``buildProject`` option
    must be set in the ``conan install`` step.


Running ``cmake`` Directly
--------------------------

The ``conan install`` command must always be run to install the required dependencies and compile them.  If the
developer wishes, the ``cmake`` can be run directly from the ``dist3`` distribution folder instead
of relying on the ``conan build`` step discussed above.

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

Running Project Tests
---------------------

The project employs two testing frameworks, specifically `Pytest <https://pytest.org>`__ for python
executed tests and `Google Test <https://github.com/google/googletest>`__ for C/C++ executed tests.

To run all tests execute the following from the project root directory

.. code-block:: console

    python run_all_test.py

To run only the python test use the following commands.  Please see :ref:`installOptionalPackages` on how to
run an optional multi-processing version of ``pytest``.

.. code-block:: console

    cd src
    pytest

To run only the C/C++ tests use

.. code-block:: console

    cd dist3
    ctest

or on macOS ``ctest -C <Release or Debug>``.
