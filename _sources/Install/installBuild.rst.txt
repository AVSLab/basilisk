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
