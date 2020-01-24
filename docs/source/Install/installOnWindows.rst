.. toctree::
   :hidden:

.. _installWindows:

Installing On Windows
=====================

The following was developed using Windows 7 and Visual Studio Community 2017 or 2019. The preferred method is to use Python 3. For now support is also provided to use the built-in Python 2, but Python 2 support is now a depreciated functionality.

Software setup
--------------

In order to run Basilisk, the following software will be necessary:

-  `Cmake <https://cmake.org/>`__
-  `Python <https://www.python.org/downloads/windows/>`__ 3.7 or 2.7
-  `pip <https://pip.pypa.io/en/stable/installing/>`__
-  Visual Studios 15 or Greater
-  `Swig <http://www.swig.org/download.html>`__ version 3 or 4

Configuration
-------------

Decide whether target deployment is 32 (win32) or 64 (x64) bit. Which ever chosen you will need the matching python and software for that architecture.

Configuring Python
~~~~~~~~~~~~~~~~~~

Python is installed using the Windows installer found on the Python website. Make sure to choose the correct installer for your architecture. The Additional required Python packages

- ``numpy``
- ``matplotlib``
- ``pytest``
- ``conan``
- ``pandas``

are installed using the Python package
manager pip (``pip.exe``) which comes default with the Windows Python installer. To install additional Python packages with pip the following
command is executed at command line::

   C:\Users\patrick> pip --trusted-host=pypi.python.org install <package name>

Configuring Swig
~~~~~~~~~~~~~~~~

The standard windows swig version 3 or 4 is suitable for Basilisk (see `Configuration
Instructions <http://www.swig.org/Doc1.3/Windows.html#Windows_swig_exe>`__).

- Download the swig zip file, which includes ``swig.exe`` file, and unzip it into somewhere like ``C:/Program Files/Swig``
- Add swig path into environment variables using the following steps:

  - Right-click on My Computer, Select Properties
  - Under the Advanced tab, Select Environment Variables
  - Under the System Variables panel, Select Path, and Click Edit
  - Add the ``swig.exe`` directory to your path

Example added path formats::

   PYTHON_INCLUDE = C:\Program Files\Python27\include 
   PYTHON_LIB = C:\Program Files\Python27\libs\python27.lib

Install Conan
~~~~~~~~~~~~~

Go to the `Conan Website <https://conan.io/downloads.html>`__ and download the windows installer. Proceed with installation. If it asks to be added to the PATH, allow it to add itself to the PATH.  The ``conan`` environment is setup using::

    $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
    $ conan remote add conan-community https://api.bintray.com/conan/conan-community/conan



Pulling and Building the Basilisk Project
-----------------------------------------

When all the prerequisite installations are complete, the project can be built as follows.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.

#. First step is to create the destination directory.  This is ``dist3`` for Python 3 and ``dist`` for Python 2::

       $ mkdir dist
       $ cd




#. Configuration and Build:

   - Python 2::

        cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win<arch>" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
        cmake --build . --target ALL_BUILD --config Release

   - Python 3::

        cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win<arch>" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF -DUSE_PYTHON3=ON
        cmake --build . --target ALL_BUILD --config Release

   - Example command using x86::

      cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win32" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF

     MSVC Mapping

        =================  ===============
        MSVC Product Year  MSVC Version
        =================  ===============
        2019               16
        2017               15.9
                           15.8
                           15.7
                           15.6
                           15.5
                           15.4 - 15.3
                           15.2 - 15.0
        2015               14
        2013               12
        2012               11
        =================  ===============

    Example build commands for Arch x86, MSVC Year 2017, MSVC Version 15::

        cmake -G “Visual Studio 15 2017 Win32” ../src

    Example build commands forArch x64, MSVC Year 2019, MSVC Version 16::

        cmake -G “Visual Studio 16 2019” -A x64 ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF

        cmake -G “Visual Studio 15 2017 Win64” ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF

#. If the build was not setup correctly, you can delete the ``dist3`` folder and re-run the above command to get another clean build attempt.

#. To test your setup you can run one of the example scenario scripts.

   -  For example, in the terminal window, make ``basilisk/src/examples`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       $ python3 scenarioBasicOrbit.py



