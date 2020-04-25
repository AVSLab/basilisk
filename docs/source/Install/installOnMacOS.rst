.. toctree::
   :hidden:


.. _installMacOS:

Setup On macOS
==============

These instruction outline how to install Basilisk (BSK) on a clean version of macOS.  The preferred method is to use Python 3.  For now support is also provided to use the built-in Python 2, but Python 2 support is now a depreciated functionality.


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

Choosing Python 3 (preferred) or Python 2 (depreciated) setups
--------------------------------------------------------------
Basilisk is able to create both Python 3.7 or Python 2.7 code. All the unit test and tutorial scenario files are written such that they work in both generations of Python. The Python 3 version is the preferred installation method. Python 2 remains supported, but should be treated as depreciated.

To install Python 3 on macOS there are two common options:

#. Download the installer package from `python.org <https://python.org>`__.  This is the preferred method of getting Python 3.
#. Install python 3 through the `HomeBrew <http://brew.sh>`__ package management system. The site has the command line to install homebrew from a terminal window.


Install HomeBrew Support Packages
---------------------------------

#. Install `HomeBrew <http://brew.sh>`__ using a Terminal window and
   pasting the install script from the HomeBrew web site.

   .. Note:: This must be done within a ``bash`` terminal window. The type of terminal emulation is shown on the top of the terminal window. If you are running another terminal type, type ``bash`` to engage the Bash terminal environment. This is just required to install HomeBrew. Once it is installed, you can run all other commands from any terminal type.

#. The new SWIG version 4 is compatible with Basilisk. Install the SWIG software package using::

   $ brew install swig

#. If you want to install the HomeBrew version of ``cmake``, you can do so with::

   $ brew install cmake
   $ brew link cmake


Setting up the Python Environment
---------------------------------

.. Note:: The following instructions recommend installing all the required python packages in the user ``~/Library/Python`` folder. This has the benefit that no ``sudo`` command is required to install and run Basilisk, and the user Python folder can readily be replaced if needed. If you are familiar with python you can install in other locations as well.

.. Note:: If you wish to use the HomeBrew version of python, or generally have multiple copies of python installed on your system, configure the CMake Python paths as described in :ref:`configureBuild` after following these instructions.

.. Note:: We suggest you remove any other python packages (such as Anaconda), or change the path in your terminal shell if you really want to keep it.

In the following instructions, be sure to follow the sequence of tasks as outlined below.

Setting the ``PATH`` Environment Variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As this installation will install all required Python packages in the
user home directory ``Library/Python`` folder, the ``PATH`` variable
must be setup within the terminal environment. If you are using Python 2, then replace ``3.7`` with ``2.7`` in the instructions below. It is ok to include both folders in your path if you are using both Python 2 and 3.

#. Open a terminal window
#. To open these system files in TextEdit.app for easy editing, you can use the shell ``open`` command in steps 3 through 6 below
#. If using a Bash shell, then

   -  type::

        $ open ~/.bash_profile

   -  Add the line::

        export PATH=~/Library/Python/3.7/bin:$PATH

#. If using a tcsh shell, then

   -  type::

        $ open .tcshrc

   -  Add the line::

        set path = ( ~/Library/Python/3.7/bin $path )

#. If using a zsh shell, then

    - type::

        $ open .zshrc

    - Add the line::

        PATH=/Users/hp/Library/Python/3.7/bin:$PATH

#. Save and close the file
#. Open a new terminal window for the path to take effect

Setup Required Python 2 packages, skip if using Python 3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. Note:: If you already have Python 3 installed and are trying to use Python 2 as well, then depending on the path dependencies the ``pip`` command might have to be called with ``python -m pip`` to ensure the Python 2 version of ``pip`` is called.

- First the python package manager ``pip`` must be installed. From the terminal window, enter the following commands::

    $ easy_install --user pip

  If you run into issues with ``pip`` installation setup, you can re-install pip by downloading a fresh copy using::

    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py

  and then executing the following command to install pip in the user’s home directory::

    python get-pip.py --user

  This step is sometimes needed if you were working with an earlier python installation.  Next, install setup tools using::

    $ pip install --user --ignore-installed setuptools

-  Copy the file called :download:`mac_fix_path.pth </resources/mac_fix_path.pth>` from basilisk/docs to the directory ``~/Library/Python/2.7/lib/python/site-packages/``.  For more information about this file see this `online
   discussion <https://apple.stackexchange.com/questions/209572/how-to-use-pip-after-the-os-x-el-capitan-upgrade/209577>`__.

   .. Note:: If you have installed python packages already using ``sudo pip install``, then these are stored in ``Library/Python/2.7/site-packages``. You need to add the ``mac_fix_path.pth`` file to this folder as well to make macOS ignore the system installed packages. Or, to only use home directory installed python packages, just remove ``Library/Python`` folder.

Installing required python support packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  From the terminal window, install the required general Python
   packages using either pip3 (for Python 3) or pip (for Python 2)::

       $ pip3 install --user numpy
       $ pip3 install --user matplotlib
       $ pip3 install --user pandas

-  Basilisk uses conan for package managing. In order to do so, users
   must install conan and set the remote repositories for libraries:::

       $ pip3 install --user conan
       $ conan remote add conan-community https://api.bintray.com/conan/conan-community/conan
       $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan

-  `Optional Packages:` The above directions install the Basilisk base software. There are a series of :ref:`optional packages<installOptionalPackages>` that enhance this capability, including ``pytest`` to run an automated test suite of unit and integrated tests.



Build Project Process via Terminal
----------------------------------

When all the prerequisite installations are complete, the project can be built as follows.

#. The ``conanfile.py`` will setup and configure the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

        python3 conanfile.py

   For other configure and build options, see :ref:`configureBuild`.  This creates the Xcode project in
   ``dist3``.

  .. Note:: If you wish to use the another version of python configure the Python paths in :ref:`customPython`

  .. Warning:: If you get an error message in `cmake` saying it can’t find the compiler tools, open a Terminal window and type::

        $ xcode-select -p

    This should return::

        /Applications/Xcode.app/Contents/Developer

    If instead you get a different director such as ``/Library/Developer/CommandLineTools``, then correct this compiler directory path using::

        sudo xcode-select --reset

    Now clear the Cmake cache and try running the configure and build process again.


#. Open the Xcode project  file  inside ``dist3`` or
   ``dist``.  This is ``basilisk.xcodeproj`` on macOS.

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

#. Q: swig not installing

    -  A: Make sure you have pcre installed (using brew install preferably)

#. Q: Experiencing problems when trying to change the directory in which to clone the url

   -  A: clone it in the default directory, and copy it into the preferred one after it is done cloning.

#. Q : Permission denied when using brew

   -  A: Add sudo to the start of the command. If you do not have superuser access, get superuser access.

#. Q : Python unexpectedly quit when trying to run pytest

   -  A: Check the python installation. If the path is wrong, uninstall, and reinstall python using brew.

#. Q : I updated my macOS system to the latest released, and I can no longer run CMake or build with Xcode.

   -  A: Do a clean build as described in :ref:`FAQ`.
