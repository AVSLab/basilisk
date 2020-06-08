.. toctree::
   :hidden:


.. _installMacOS:

Setup On macOS
==============

These instruction outline how to install Basilisk (BSK) on a clean version of macOS.
Basilisk requires the use of Python 3.  Don't use the Python 2 system that comes with macOS.


Developer Software Tools
------------------------

In order to run Basilisk on macOS, the following software is necessary:

#. Get the `Apple Xcode
   Developer <https://itunes.apple.com/us/app/xcode/id497799835?mt=12>`__
   tool from the App Store

   -  After Xcode is installed, start up the program to let it finish installing development components
   -  Open a Terminal window to install the command line tools using::

        $ xcode-select --install

#. (Optional) Get the `SourceTree <http://sourcetreeapp.com>`__
   application to be able to pull and manage a copy of Basilisk
#. (Optional) Get the `PyCharm <https://www.jetbrains.com/pycharm/>`__
   application to be able to edit python source files

Install Python 3
----------------
To install Python 3 on macOS there are two common options:

#. (Preferred) Download the installer package from `python.org <https://python.org>`__.  This will configure your
   your macOS environment for Python 3 and can readily be upgraded by downloaded a newer installer package.
#. Install python 3 through the `HomeBrew <http://brew.sh>`__ package management system. The site has the
   command line to install homebrew from a terminal window using ``brew install python3``.

Install HomeBrew Support Packages
---------------------------------

#. Install `HomeBrew <http://brew.sh>`__ using a Terminal window and
   pasting the install script from the HomeBrew web site.

#. The new SWIG version 4 is compatible with Basilisk. Install the SWIG software package using::

   $ brew install swig

#. If you want to install the HomeBrew version of ``cmake``, you can do so with::

   $ brew install cmake
   $ brew link cmake

   You need at least version 3.x or higher.

Setting up the Python Environment
---------------------------------

.. Note:: The following instructions recommend installing all the required python packages in the user ``~/Library/Python`` folder. This has the benefit that no ``sudo`` command is required to install and run Basilisk, and the user Python folder can readily be replaced if needed. If you are familiar with python you can install in other locations as well.

.. Note:: If you wish to use the HomeBrew version of python, or generally have multiple copies of python installed on your system, configure the CMake Python paths as described in :ref:`customPython` after following these instructions.

.. Note:: We suggest you remove any other python packages (such as Anaconda), or change the path in your terminal shell if you really want to keep it.

In the following instructions, be sure to follow the sequence of tasks as outlined below.

Setting the ``PATH`` Environment Variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As this installation will install all required Python packages in the
user home directory ``Library/Python`` folder, the ``PATH`` variable
must be setup within the terminal environment.

#. Open a terminal window
#. To open these system files in TextEdit.app for easy editing, you can use the shell ``open`` command in steps 3 through 6 below
#. If using a zsh shell, then

    - type::

        $ open .zshrc

    - Add the line::

        PATH=/Users/hp/Library/Python/3.7/bin:$PATH

#. If using a Bash shell, then

   -  type::

        $ open ~/.bash_profile

   -  Add the line::

        export PATH=~/Library/Python/3.7/bin:$PATH

#. Save and close the file
#. Open a new terminal window for the path to take effect

Installing required python support packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  Basilisk uses conan for package managing. In order to do so, users
   must install conan::

       $ pip3 install --user conan

   The conan repositories information is automatically setup by ``conanfile.py``.

   Note that ``conan`` version 1.25.x has issues on macOS where it doesn't properly link system frameworks.  Stick
   with 1.24.x for now.

- The following python packages are the minimum required packages for Basilisk.  They are installed and checked
  for the correction version in the steps below.

  - .. include:: ../bskPkgRequired.txt

-  `Optional Packages:` The above directions install the Basilisk base software. There are a series of :ref:`optional packages<installOptionalPackages>` that enhance this capability, including ``pytest`` to run an automated test suite of unit and integrated tests.

Build Project Process via Terminal
----------------------------------

When all the prerequisite installations are complete, the project can be built as follows.

#. The ``conanfile.py`` will setup and configure the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

        python3 conanfile.py

   For other configure and build options, see :ref:`configureBuild`.  This creates the Xcode project in
   ``dist3``.
   This process will verify that the minimum required Basilisk python packages are installed, and that
   the version is correct.  If not, the user is prompted to install the package with ``pip3`` in the system or user
   folder.

  .. Note:: If you wish to use the another version of python 3 configure the Python paths in :ref:`customPython`

  .. Warning:: If you get an error message in `cmake` saying it can’t find the compiler tools, open a Terminal window and type::

        $ xcode-select -p

    This should return::

        /Applications/Xcode.app/Contents/Developer

    If instead you get a different director such as ``/Library/Developer/CommandLineTools``, then correct this compiler directory path using::

        sudo xcode-select --reset

    Now clear the Cmake cache and try running the configure and build process again.


#. Open the Xcode project  file  inside ``dist3``.  This is ``basilisk.xcodeproj`` on macOS.

   -  The source code should appear and be ready for use

      .. image:: /_images/static/256564102-xcode.png
         :align: center
         :scale: 40%
   -  Ensure that the target scheme is set to ``ALL_BUILD`` as shown in figure above
   -  You can now build the project within the Xcode IDE

#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, make ``basilisk/src/examples/`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       $ python3 scenarioBasicOrbit.py


FAQs
----

#. Q: Experiencing problems when trying to change the directory in which to clone the url

   -  A: clone it in the default directory, and copy it into the preferred one after it is done cloning.

#. Q : Permission denied when using brew

   -  A: Add sudo to the start of the command. If you do not have superuser access, get superuser access.

#. Q : I updated my macOS system to the latest released, and I can no longer run CMake or build with Xcode.

   -  A: Do a clean build as described in :ref:`FAQ`.
