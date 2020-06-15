.. toctree::
   :hidden:


.. _installLinux:

Setup On Linux
==============

Software setup
--------------

In order to run Basilisk, the following software will be necessary. This document outline how to install this support software.

-  `Cmake <https://cmake.org/>`__ 3.14 or higher
-  `Python <https://www.python.org/>`__ 3.7.x.  The following python package dependencies are automatically
   checked and installed in the steps below.

   - .. include:: ../bskPkgRequired.txt

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

#. Python 3.x with Pip::

    $ apt-get install python3.x

#. SWIG: Available using::

    $ apt-get install swig

#. A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Basilisk.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.

#. Install Conan using pip, an example is below::

       $ pip3 install --user conan

   The conan repositories information is automatically setup by ``conanfile.py``.

#. `Optional Packages:` The above directions install the Basilisk base software. There are a series of :ref:`optional packages<installOptionalPackages>` that enhance this capability.

Build Process via Terminal
--------------------------

#. The ``conanfile.py`` will setup and configure the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

        python3 conanfile.py

   For other configure and build options, see :ref:`configureBuild`.
   This process will verify that the minimum required Basilisk python packages are installed, and that
   the version is correct.  If not, the user is prompted to install the package with ``pip3`` in the system or user
   folder.

#. Next, move to the distribution folder to build using a makefile::

        cd dist3

#. Can do a multi core make by running ``make -j<number of cores +1>`` such as ``make -j5``.

#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, switch ``/src/examples/`` to the
      current directory.
   -  Run one of the example scripts, such as::

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
