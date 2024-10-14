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

