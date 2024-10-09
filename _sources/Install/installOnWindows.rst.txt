.. toctree::
   :hidden:

.. _installWindows:

Setup On Windows
================

The following was developed using Windows 7 and Visual Studio Community 15 2017 or 16 2019.

Software setup
--------------

In order to run Basilisk, the following software will be necessary:

-  `Cmake <https://cmake.org/>`__ 3.14 or higher.  Make sure you can execute this
   program from the command line
-  `Python <https://www.python.org/downloads/windows/>`__ 3.8 to 3.11
-  `pip <https://pip.pypa.io/en/stable/installing/>`__
-  Visual Studios 15 2017 or greater
-  `Swig <http://www.swig.org/download.html>`__ version 4.X
-  (Optional) A GiT GUI application such as `GitKraken <https://www.gitkraken.com>`__
   to manage your copy of the Basilisk repository

The following python package dependencies are automatically checked and installed in the steps below.

.. literalinclude:: ../../../requirements.txt
   :language: python

Configuration
-------------

Strongly recommended to stick with default 64-bit installations.
Decide whether target deployment is 32 (win32) or 64 (x64) bit. Which ever chosen you will need the matching python and software for that architecture.

Installing Python
~~~~~~~~~~~~~~~~~~

Python is installed using the Windows installer found on the Python website. Make sure to choose the correct
installer for your architecture. When stepping through the installer there are two menus options menus. The
first is the optional features menu for which all options should be selected.

   .. image:: /_images/static/windows-installer-python-optional-features-installer-menu.png
      :align: center
      :scale: 75%

The second menu is the python advanced options menu where both "Download debugging symbols" and "Download debug binaries
(requires VS 2017 or later)" should be selected in order to build Basilisk with a Debug profile. A Debug profile is
required to place accurate breakpoints/attach a debugger to C/C++ code.

   .. image:: /_images/static/windows-installer-python-advanced-options-installer-menu.png
      :align: center
      :scale: 75%

Install Swig
~~~~~~~~~~~~
The standard windows swig version 4 is suitable for Basilisk (see `Configuration
Instructions <http://www.swig.org/Doc1.3/Windows.html#Windows_swig_exe>`__).
Download the swig zip file, which includes ``swig.exe`` file, and unzip it into somewhere like ``C:/Program Files/Swig``


Configuring User Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~
Add SWIG and Basilisk paths into environment variables using the following steps:

  - Right-click on My Computer, Select Properties
  - Under the Advanced tab, Select Environment Variables
  - Under the User (or System, depending on your setup) Variables panel, Select Path, and Click Edit

    - Add the ``swig.exe`` directory to your path.  See this `site <https://stackoverflow.com/questions/48382254/cmake-error-could-not-find-swig-missing-swig-dir>`__
      for more info on setting paths for swig.
    - add the path to ``CMake\bin``, such as ``C:\Program Files\CMake\bin``
    - Add the Basilisk library directory (``path2bsk/dist3/Basilisk``) to your path. Here,
      ``path2bsk`` is replaced with the actual path to the Basilisk folder.  Note, the ``dist3`` folder does not
      exist to begin with, but is created automatically when configuring Basilisk with ``python conanfile.py``
      as discussed below.

For more information on how to configure the path Variable on Windows see this
`help <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`__ link.
Example added path formats::

   PYTHON_INCLUDE = C:\Program Files\Python37\include
   PYTHON_LIB = C:\Program Files\Python37\libs\python37.lib

.. note::

    If any environment variables have been modified (``cmake``, ``swig``, ``path2bsk`` paths),
    Command Prompt needs
    to be closed and rebooted (and a machine reboot MAY be needed) for the change to take effect.


Using A Python Virtual Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. attention:: We strongly recommend using a python virtual environment while installing basilisk or running basilisk modules.
    For more info, `read this <https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/>`__.
    The virtual environment has the benefit that you won't have conflicts with other versions of Python or
    python packages that your computer has installed.  It lets you install packages specific to this environment
    and they won't interfere with other python projects you may have.
    However, you must turn this environment on and off each time you want to use it.

The following steps show how to create, active ad de-activate a virtual environment.  The remaining installation
steps work regardless if done within a virtual environment or not.

#. In a Terminal window change your current directory to be the Basilisk folder,
   then create a virtual environment using::

     $ python -m venv .venv

   This creates a hidden folder inside the Basilisk folder which will store all the python packages and
   environment information.

#. Activate virtual environment when needing configure, build or run Basilisk::

     $ .venv\Scripts\activate

   If the virtual environment is activated, users will see (venv) before the prompt

#. Deactivate the virtual environment to return to the normal operating system environment::

     (venv) $ deactivate


Installing required python support packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. Basilisk uses conan for package managing. In order to do so, users
   must ensure ``wheel`` is installed and install ``conan``::

       (venv) $ pip install wheel 'conan<2.0'

   The conan repositories information is automatically setup by ``conanfile.py``.


   .. warning::

      Don't use the ``conan`` binary installed from the `conan web site <https://conan.io/downloads.html>`__.
      This causes several issues with the current build system.


   .. warning::

      If you are upgrading from a version of Basilisk prior to 1.8.0, be sure to delete the ``.conan`` folder in your
      home directory to create a clean copy compatible with the current build system.

#. The required python packages for Basilisk will be installed automatically when running ``conanfile.py``.

#. `Optional Packages:` The above directions install the Basilisk base software.
   There are a series of :ref:`optional packages<installOptionalPackages>` that enhance this capability.

Build Project Process via Command line
--------------------------------------

When all the prerequisite installations are complete, the project can be built as follows.

#. The ``conanfile.py`` will setup, configure and run the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

    (venv) $ python conanfile.py

   This creates the Visual Studio 16 2019 IDE project in ``dist3`` and builds the project.
   You can also specify the generator directly in this build process and select other versions of Visual Studio.
   For other configure and build options, including running ``cmake`` directly, see :ref:`configureBuild`.
   This process will verify that the minimum required Basilisk python packages are installed, and that
   the version is correct.  If not, the user is prompted to install the package with ``pip3`` in the system or user
   folder.

   .. note::

        The default Window compiler is Visual Studio 16.  If you had VS 17 installed and downgraded to VS 16,
        then the system might still find VS 17 and give an error when running the above command.
        If you want to compile with latest VS 17, then use
        ``python conanfile.py --generator "Visual Studio 17 2022"``.

   .. note::

        To build on Windows you need to run an account with admin privileges.



#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, make ``basilisk/examples`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       (venv) $ python scenarioBasicOrbit.py



Building with IDE
-----------------
Conan file will build the project by default.  However, this can take longer to compile than building the
Basilisk project in the IDE directly.  Further, if you are developing for Basilisk you often just want to configure
the Basilisk Xcode project file and not build right away. To change the default behavior disable the automatic build
using:

#. Run this command to disable the build::

    python conanfile.py --buildProject False

   This will disable the build workflow so that you can build the project from the IDE.

#. Open the Visual Studio project file inside ``dist3``.  This is ``basilisk.sln`` on Windows.

     -  The source code should appear and be ready for use

      .. image:: /_images/static/visual-studio.png
         :align: center
         :scale: 50%

   -  Change the active config to Release instead of debug for solution properties.
   -  Within Visual Studio now go under `Build menu/Build Solution` to build.
