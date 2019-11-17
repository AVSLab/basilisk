.. toctree::
   :hidden:


.. _installLinux:

Installing On Linux
===================

The preferred method is to use Python 3. For now support is also provided to use the built-in Python 2, but Python 2 support is now a depreciated functionality.

Software setup
--------------

In order to run Basilisk, the following software will be necessary. This document outline how to install this support software.

-  `Cmake <https://cmake.org/>`__
-  `Python <https://www.python.org/>`__ 3.7.x OR Python 2.7
   (``numpy==1.15.4``, ``matplotlib``, ``pytest``, ``conan``, ``pandas``)
-  `SWIG <http://www.swig.org/>`__ (version 3 or 4)
-  `GCC <https://gcc.gnu.org/>`__

Dependencies
------------

.. Note:: Depending on your system setup, administrative permissions (sudo or su) may be required to install these dependencies. Some distributions of Linux will use other package management commands such as ``yum``, ``dnf``, of ``pgk``.

#. CMake: Available using CMake-GUI or CMake over the command line::

        # GUI installation
        $ apt-get install cmake-gui

        # Command line installation
        $ apt-get install cmake

#. Python 2.7 / Python 3.x with Pip::

    $ apt-get install python2.7
    $ apt-get install python3.x

#. SWIG: Available using::

    $ apt-get install swig3.0

#. A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Basilisk.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.

#. A install of Conan. Install with pip, an example is below::

       $ pip3 install conan

Build Process via Terminal
--------------------------

For Basilisk Python 2 and Python 3 inter-compatability, build using both following instructions then run using preferred python version.

#. First step is to create the destination directory.  This is ``dist3`` for Python 3 and ``dist`` for Python 2::

       $ mkdir dist
       $ cd

#. Setup Conan Repositories. These can be consolidated into a private conan server `conan getting started docs <https://docs.conan.io/en/latest/introduction.html>`__::

       $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
       $ conan remote add conan-community https://api

#. CMake here in the build directory with Unix Makefiles, where the source code is located at: ``../src``::

    $ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../src

   The ``CMAKE_BUILD_TYPE`` argument can be changed to ``Debug`` as well, but the run-time performance will be significantly slower.

#. Can do a multi core make by running ``make -j<number of cores +1>`` such as ``make -j5``.

#. More information is available on Basilisk ``cmake`` :ref:`flag options <cmakeOptions>`.

#. To test your setup you can run one of the example scenario scripts.

   -  For example, in the terminal window, make ``basilisk/src/examples/01-OrbitalSimulations`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       $ python3 scenarioBasicOrbit.py





Other packages some distributions of Linux may need
---------------------------------------------------

::

       # Update current software
       $ apt-get update

       # Helpful for Debian systems -  all packages need to compile such as gcc and g++ compilers and other utils.
       $ apt-get install build-essential

       # Installing the header files for the Python C API
       $ apt-get install python-dev 

       # Package development process library to facilitate packaging Python packages
       $ apt-get install python-setuptools

       # Tkinter
       $ apt-get install python-tk 

       # Python PIP
       $ apt-get install python-pip

       # Python pytest
       $ pip install pytest
