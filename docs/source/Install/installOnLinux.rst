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

#. On a new Linux system various developer packages and support libraries are requried::

       # Update current software
       $ apt-get update

       # Get GIT for source code version control
       # apt-get install git

       # Helpful for Debian systems -  all packages need to compile such as gcc and g++ compilers and other utils.
       $ apt-get install build-essential

       # Install Python 3
       $ apt-get install python3

       # Package development process library to facilitate packaging Python packages
       $ apt-get install python3-setuptools

       # Tkinter
       $ apt-get install python3-tk

       # Python PIP
       $ apt-get install python3-pip

#. SWIG: Available using::

    $ apt-get install swig

#. A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Basilisk.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.

#. Install Conan using pip, an example is below::

       $ pip3 install --user conan

   The conan repositories information is automatically setup by ``conanfile.py``.

   .. warning::

      If you are upgrading from a version of Basilisk prior to 1.8.0, be sure to delete the ``.conan`` folder in your
      home directory to create a clean copy compatible with the current build system.

#. CMake: You can install cmake using pip3.  This makes it easy to overcome limitations of which version of ``cmake``
   the ``apt-get`` command provides::

       $ pip3 install --user cmake

#. Note, if you choose to install python packages local in your user directory ``.local`` folder, be sure to add
   ``~/.local/bin`` to your ``PATH`` variable.

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

