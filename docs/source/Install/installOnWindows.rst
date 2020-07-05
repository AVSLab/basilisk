.. toctree::
   :hidden:

.. _installWindows:

Setup On Windows
================

.. warning::

    The :ref:`vizInterface` and ``opNav`` related modules are not currently working with the Basilisk 1.8.x build
    system.  If you need this functionality, then please use Basilisk version 1.7.x for now.


The following was developed using Windows 7 and Visual Studio Community 2017 or 2019. The preferred method is to use Python 3. For now support is also provided to use the built-in Python 2, but Python 2 support is now a depreciated functionality.

Software setup
--------------

In order to run Basilisk, the following software will be necessary:

-  `Cmake <https://cmake.org/>`__ 3.14 or higher
-  `Python <https://www.python.org/downloads/windows/>`__ 3.7.x
-  `pip <https://pip.pypa.io/en/stable/installing/>`__
-  Visual Studios 15 or Greater
-  `Swig <http://www.swig.org/download.html>`__ version 3 or 4

Configuration
-------------

Strongly recommended to stick with default 64-bit installations.
Decide whether target deployment is 32 (win32) or 64 (x64) bit. Which ever chosen you will need the matching python and software for that architecture.

Configuring Python
~~~~~~~~~~~~~~~~~~

Python is installed using the Windows installer found on the Python website. Make sure to choose the correct
installer for your architecture.

Install Swig
~~~~~~~~~~~~
The standard windows swig version 3 or 4 is suitable for Basilisk (see `Configuration
Instructions <http://www.swig.org/Doc1.3/Windows.html#Windows_swig_exe>`__).
Download the swig zip file, which includes ``swig.exe`` file, and unzip it into somewhere like ``C:/Program Files/Swig``


Configuring User Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~
Add SWIG and Basilisk paths into environment variables using the following steps:

  - Right-click on My Computer, Select Properties
  - Under the Advanced tab, Select Environment Variables
  - Under the User Variables panel, Select Path, and Click Edit

    - Add the ``swig.exe`` directory to your path
    - Add the Basilisk library directory (``path2bsk/dist3/Basilisk``) to your path. Here, ``path2bsk`` is replaced with the actual path to the Basilisk folder.

For more information on how to configure the path Variable on Windows see this
`help <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`__ link.
Example added path formats::

   PYTHON_INCLUDE = C:\Program Files\Python37\include
   PYTHON_LIB = C:\Program Files\Python37\libs\python37.lib


Installing required python support packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Basilisk uses conan for package managing. In order to do so, users
  must install conan::

       $ pip3 install conan

  The conan repositories information is automatically setup by ``conanfile.py``.

  .. warning::

     Don't use the ``conan`` binary installed from the `conan web site <https://conan.io/downloads.html>`__.
     This causes several issues with the current build system.

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

   This creates the Visual Studio 16 2019 IDE project in ``dist3``.
   You can also specify the generator directly in this build process and select other versions of Visual Studio.
   For other configure and build options, including running ``cmake`` directly, see :ref:`configureBuild`.
   This process will verify that the minimum required Basilisk python packages are installed, and that
   the version is correct.  If not, the user is prompted to install the package with ``pip3`` in the system or user
   folder.

#. To test your setup you can run one of the :ref:`examples`:

   -  For example, in the terminal window, make ``basilisk/src/examples`` the
      current directory.
   -  Run one of the tutorial scenarios, such as::

       $ python3 scenarioBasicOrbit.py

