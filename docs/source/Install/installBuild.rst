.. toctree::
   :hidden:

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

This one-line step will use ``conan`` to pull and compile any resource dependencies such a protobuffer, etc. and
configure the Xcode (macOS) or Visual Studio (Windows) IDE project in the ``dist3`` folder (``dist`` folder for legacy Python 2 use),
or create the makefile for Linux systems.  By default the build is for Python3 with the support for :ref:`vizInterface`
included to enable recording data for or live-streaming to :ref:`Vizard <Vizard>`.

The script accepts the following options to customize this process.

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
        once install they don't need to be rebuilt unless `.conan` is deleted or the dependency changes.
    * - ``python3``
      - Boolean
      - True
      - (depreciated) Determines if the build is setup for Python 3.  This flag will go away when Python 2
        support is dropped soon.  If this is False, then the destination folder is set to ``dist``
        instead of ``dist3``
    * - ``clean``
      -
      - None
      - If option is set, this forces the distribution folder ``dist3`` to be deleted to create a fresh setup and build
    * - ``buildProject``
      -
      - Not Set
      - If set, this option will not only create the IDE or make file, but will also compile the project right away
    * - ``buildType``
      - Release, Debug
      - Release
      - Sets the build type.  This does not apply to the IDE project like Xcode and Visual Studio which
        control the build type through their interface.
    * - ``generator``
      - see `here <https://docs.conan.io/en/latest/reference/generators.html>`__
      - Not Set
      - The generator is not automatically selected to set to this generate case.

Thus, for example, to create a build with ``opNav`` modes enabled, but no :ref:`vizInterface`, and using a
clean distribution folder, and that is built right away, you could use::

    python3 conanfile.py --clean --buildProject --opNav True --vizInterface False


Configuring and Building with ``conan`` Commands
------------------------------------------------
Calling ``conanfile.py`` with python executes a method that calls two separate ``conan`` function to setup and
configure process, and build the executable if desired.  This section outlines this 2-step build process for
those that seek to build it this way.

All commands are called from the Basilisk root directory.  The first command, in its mimimalist form, is::

    conan install . -if dist3/conan --build=missing

This conan command will create the destination folder, ``dist3`` in the above case, if needed, collect all the
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
    * - ``-o python3``
      - Boolean
      - True
      - (depreciated) Determines if the build is setup for Python 3.
    * - ``-o clean``
      - Boolean
      - False
      - Make the destination folder to be deleted before configuring to yield a fresh build
    * - ``-o generateIdeProject``
      - Boolean
      - True
      - Automatically set the generator to Xcode (macOS) or Visual Studio (Windows)
    * - ``-o buildProject``
      - Boolean
      - False
      - Will build the project executable after the configuration step
    * - ``-s buildType``
      - Release, Debug
      - Release
      - Specifies the build type
    * - ``-g``
      - see `here <https://docs.conan.io/en/latest/reference/generators.html>`__
      - Not set
      - Used to specify a specific generator

Thus, using the same build example as in the one-step section, to create a build with ``opNav`` modes enabled,
but no :ref:`vizInterface`, and using a clean distribution folder, and that is built right away, you could use::

    conan install . -if dist3/conan --build=missing -o clean=True -o buildProject=True -o opNav=True -o vizInterface=False

Note how much more verbose this is, but it gives you full control if you want to store the compiled binaries and
cmake files in directories other than ``dist3/conan``.

The final step is to create the IDE project file and possibly build the executables directly.
At this stage there are no options to be provided.  This step is done with::

    conan build . -if dist3/conan

.. warning::

    If you want the ``conan build`` command to automatically compile the code, the ``buildProject`` option
    must be set in the ``conan install`` step.


Running ``cmake`` Directly
--------------------------

The ``conan install`` command must always be run to install the required dependencies and compile them.  If the
developer wishes, the ``cmake`` can be run directly instead of relying on the ``conan build`` step discussed above.

The following table summarizes the optional flags that can be provided to ``cmake``:

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
    * - ``USE_PYTHON3``
      - ``ON``
      - (depreciated) enables Basilisk to be compiled for Python 3





