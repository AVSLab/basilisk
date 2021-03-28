.. toctree::
   :hidden:


.. _installMacOSM1:

Setup On Apple Computer with M1 Processor
=========================================

These instructions allow you to run Basilisk on an Apple computer with the new ``arm64`` architecture, such as with the M1 chip.  As this chip is still very new, not all the required python support packages, like ``numpy``, have been converted.  Thus, for now Basilisk must be built and run as an Intel binary.  However, the good news is that Basilisk compiles about 2x faster on the M1 compared to the latest Intel MacBook Pro, and executing all Basilisk unit tests completes slightly faster.  The latter is true even though we are running Basilisk emulating an Intel processor within Rosetta 2.

We still follow the same general setup as discussed in :ref:`installMacOS`.  However, some extra steps are required and outlined below.

Install Python
--------------
The `Python.org <https://python.org>`__ web site contains a Universal binary for Python 3.9.  You can download the installer package there.

Using Homebrew
--------------
Install `HomeBrew <http://brew.sh>`__ using a Terminal window as normal.  The latest version 3.0 and up natively supports the M1 processor.  You can use ``brew`` to install ``swig``, ``cmake`` to support building Basilisk, and install ``doxygen`` to support building the Basilisk documentation.


Setting up Python Packages
--------------------------

Running a Terminal Shell Emulating an Intel Processor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To install and run the Python support packages, you need to run the Terminal shell using Rosetta.  You can configure a Terminal shell profile specially to emulate the Intel process as discussed `here <https://www.computerworld.com/article/2726136/use-the-mac-s-command-line-to-get-detailed-cpu-information.html>`__, or open Terminal itself using Rosetta 2.  The later is done by finding ``/Applications/Utilities/Terminal.app`` in the find, press ``cmd-I`` to open the Finder info panel as shown:

.. image:: /_images/static/m1TerminalInfo.png
    :align: center
    :scale: 50%

Toggle the `Open using Rosetta` checkbox.  If you now start the Terminal it will install code as if you are running on an Intel processor.

Running a Virtual Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Follow the :ref:`installMacOS` instructions to setup and activate the virtual environment.  This way you install the Intel python packages just for using Basilisk.  This won't interfere with other projects that might be running M1 python packages.

Be sure to update `pip` inside the virtual environment to the latest version using::

    pip3 install -U pip


Install Packages
~~~~~~~~~~~~~~~~
The first step is still to install ``conan``.  However, in this setup you also need to install the ``wheel`` package using::

    (.venv) $ pip3 install conan wheel

After this you can run the regular ``python3 conanfile.py`` command as discussed in :ref:`installMacOS`.  This will prompt you to install the remaining required packages for Basilisk.

Building with IDE
~~~~~~~~~~~~~~~~~

Conan file will build the project by default. To change this behavior, you need to disable the build.

#. Run this command to disable the build::

    (venv) $ python3 conanfile.py --buildProject False


When you open the Xcode project, select ``ALL_BUILD`` and build for Profile (i.e. Release) as normal.  It will build an Intel binary to run under Rosetta.

Installing Optional Packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
You can install the optional packages discussed in :ref:`installOptionalPackages`.  However, note that ``datashader`` package does not yet run with Python greater than 3.7, sadly.