.. toctree::
   :hidden:


.. _installLinux:

Setup On Linux
==============

Software setup
--------------

In order to run Basilisk, the following software will be necessary. This document outline how to install this support software.

-  `Python <https://www.python.org/>`__ 3.8 to 3.13.  Version 3.8 is deprecated and will be removed April 2026.
-  `SWIG <http://www.swig.org/>`__ (version 4.x)
-  `GCC <https://gcc.gnu.org/>`__
-  (Optional) Get the `GitKraken <https://www.gitkraken.com>`__
   application to be able to pull and manage a copy of Basilisk

The following python package dependencies are automatically checked and installed in the steps below.

.. literalinclude:: ../../../requirements.txt
   :language: python

.. caution::

    Building Basilisk on Linux using Intel processors is being regularly tested.
    With Basilisk version 2.1.5 we added
    experimental support for Linux on ARM processors.  However, this option is not currently
    tested on a regular manner.

Package Dependencies
--------------------

.. Note:: Depending on your system setup, administrative permissions (sudo or su) may be required to install these dependencies.


Debian/Ubuntu
^^^^^^^^^^^^^
Basilisk development mainly occurs in Debian-based systems. Other distributions of Linux will use other package management commands such as ``yum``, ``dnf``, of ``pgk``. Specific instructions for Red Hat / Fedora systems are given below.

#. On Debian-based systems, install the following developer packages and support libraries::

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

       # ensure that the python developer libraries are installed
       $ apt-get install python3-dev

       # Tkinter
       $ apt-get install python3-tk

       # if you want to build with opNav modules, install the following library
       $ apt-get install libgtk2.0

       # Python PIP
       $ apt-get install python3-pip

       #Check python version installed
       $ python3 --version

       #Python virtual environment with same version as Python. For example, python3.7-venv for python3.7.x
       $apt-get install python3.x-venv

#. SWIG: Available using::

    $ apt-get install swig

#. A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Basilisk.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.


Fedora
^^^^^^

#. On Fedora, install the following developer packages and support libraries::

    # Update current software
    $ dnf check-update
    $ dnf update

    # Get GIT for source code version control
    # dnf install git

    # All packages need to compile such as gcc and g++ compilers and other utils.
    $ sudo dnf install -y git curl @development-tools dkms perl wget
    $ sudo dnf install -y gcc make mysql-devel openssl-devel
    $ sudo dnf install -y zlib-devel bzip2-devel readline-devel \
      sqlite-devel llvm ncurses-devel ncurses-libs xz \
      tk-devel libffi-devel xz-devel python3-pyOpenSSL

    # Install Python 3.
    # On Fedora, you may have to install an older version since Basilisk maintainers test on Ubuntu and  Fedora adopts new versions earlier than Ubuntu.
    # Replace .xx below with the specific subversion.
    $ dnf install python3.xx

    # Package development process library to facilitate packaging Python packages
    $ dnf install python3-setuptools

    # ensure that the python developer libraries are installed
    $ dnf install python3.xx-devel

    # Tkinter
    $ dnf install python3.xx-tkinter

    # if you want to build with opNav modules, install the following library
    $ dnf install gtk2-devel

    # Python PIP
    $ dnf install python3-pip

    #Check python version installed
    $ python3.xx --version

#. SWIG: Available using::

    $ dnf install swig

#. A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Basilisk.

#. A Git compatible version control tool like `SourceTree <http://sourcetreeapp.com>`__ should be used to :ref:`pull/clone <pullCloneBSK>` the Basilisk repository.

Python Environment
------------------

#. Using a Python Virtual Environment

   .. attention:: We strongly recommend using a python virtual environment while installing basilisk or running basilisk modules.
      For more info, `read this <https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/>`__.
      The virtual environment has the benefit that you won't have conflicts with other versions of Python or
      python packages that your computer has installed.  It lets you install packages specific to this environment
      and they won't interfere with other python projects you may have.
      However, you must turn this environment on and off each time you want to use it.

   The following steps show how to create, active ad de-activate a virtual environment.  The remaining installation
   steps work regardless if done within a virtual environment or not.

   - In a Terminal window change your current directory to be the Basilisk folder,
     then create a virtual environment using::

        $ python3 -m venv .venv

     Or for a specific Python version::

         $ python3.xx -m venv .venv

     where .xx is the subversion (such as .12 in Python3.12).

     This creates a hidden folder inside the Basilisk folder which will store all the python packages and
     environment information.

   - Activate virtual environment when needing configure, build or run Basilisk::

        $ source .venv/bin/activate

     The above step will add (.venv) before the prompt.

   - Deactivate the virtual environment to return to the normal operating system environment::

        (.venv) $ deactivate

#. Ensure all build related pip packages are installed::

       (.venv) $ pip3 install -r requirements_dev.txt

   The ``conan`` repositories information is automatically setup by ``conanfile.py``.

#. Note, if are you not using a virtual environment and you choose to install python packages
   local in your user directory ``.local`` folder, be sure to add
   ``~/.local/bin`` to your ``PATH`` variable.


Build Process via Terminal
--------------------------

#. The ``conanfile.py`` will setup, configure and run the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

        (.venv) $ python3 conanfile.py

   For other configure and build options, see :ref:`configureBuild`.
   This process will verify that the minimum required Basilisk python packages are installed, and that
   the version is correct.  If not, the user is prompted to install the package with ``pip3`` in the system or user
   folder.

#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, switch ``/basilisk/examples/`` to the
      current directory.
   -  Run one of the example scripts, such as::

       (.venv) $ python3 scenarioBasicOrbit.py


Building the Project Separately
-------------------------------
If you are developing new modules you often just want to configure the Basilisk project file without compiling.

#. Run this command to only configure and not build the Basilisk project::

      python conanfile.py --buildProject False

   This will disable the build workflow so that you can build the project separately.

#. Next, move to the distribution folder to build using a makefile::

        (.venv) $ cd dist3

#. You can do a multi core make by running ``make -j<number of cores +1>`` such as::

       (.venv) $ make -j5
