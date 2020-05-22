.. toctree::
   :hidden:

.. _installWindows:

Setup On Windows
================

The following was developed using Windows 7 and Visual Studio Community 2017 or 2019. The preferred method is to use Python 3. For now support is also provided to use the built-in Python 2, but Python 2 support is now a depreciated functionality.

Software setup
--------------

In order to run Basilisk, the following software will be necessary:

-  `Cmake <https://cmake.org/>`__ 3.14 or higher
-  `Python <https://www.python.org/downloads/windows/>`__ 3.7 or 2.7
-  `pip <https://pip.pypa.io/en/stable/installing/>`__
-  Visual Studios 15 or Greater
-  `Swig <http://www.swig.org/download.html>`__ version 3 or 4

Configuration
-------------

Strongly recommended to stick with default 64-bit installations.
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

Configuring BSK Library Destination Path
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Add the path to the Basilisk compiled library folder ``path2bsk/dist3/Basilisk`` to your path using the
above instructions for adding a path variable where ``path2bsk`` is replaced with the actual path to the
Basilisk folder.  If using depreciated Python 2, then replace ``dist3`` with ``dist``.
For more information on how to configure the path Variable on Windows see this
`help <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`__ link.

Install Conan
~~~~~~~~~~~~~

Go to the `Conan Website <https://conan.io/downloads.html>`__ and download the windows installer. Proceed with installation. If it asks to be added to the PATH, allow it to add itself to the PATH.  The ``conan`` environment is setup using::

    $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
    $ conan remote add conan-community https://api.bintray.com/conan/conan-community/conan



Build Project Process via Terminal
----------------------------------

When all the prerequisite installations are complete, the project can be built as follows.

#. The ``conanfile.py`` will setup and configure the Basilisk build.  For a basic installation,
   from the root Basilisk folder use::

        python3 conanfile.py

   This creates the Visual Studio 16 2019 IDE project in ``dist3``.
   You can also specify the generator directly in this build process and select other versions of Visual Studio.
   For other configure and build options, including running ``cmake`` directly, see :ref:`configureBuild`.

#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, make ``basilisk/src/examples`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       $ python3 scenarioBasicOrbit.py

