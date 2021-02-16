
.. _bskKnownIssues:

Basilisk Known Issues
=====================

Version |release|
-----------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.10
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.9
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.8
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.7
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.6
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.5

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.4

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.3**

- On Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.2**

- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- In Xcode, when editing ``vizInterface.c/h` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.1**

- When deleting ``.conan`` and doing a build with ``opNav`` set to true, the required dependencies can't be found
  on the repo on the first install run.  Running it again makes it work.  This is fixed in the next release to run
  properly on the first try.
- If ``openCV`` is conan installed for Release only the Xcode would give false error messages that it can't
  find the library.  This is now fixed in the current release.
- In Xcode, when editing ``vizInterface.c/h` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.
- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.

**Version 1.8.0**

- The new conan based built system might need the conan cache folder ``.conan`` to be deleted and reset.  This is
  typically in the user's home folder.  After this you need to re-run the conan setup commands::

    $ conan remote add conan-community https://api.bintray.com/conan/conan-community/conan
    $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan

- If running Windows the path to the Basilisk library destination folder must be set, see :ref:`installWindows`.
- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- If running Windows, be sure to use ``pip install conan`` to get conan, and don't download the binary installer,
  see :ref:`installWindows`.   The binary installer causes several issues with this new build system in that
  it contains its own copy of Python, and thus checking for required python packages does work.
- The new build system provides many speed improvements in doing a clean or partial build, but some small changes are
  required to update BSK python simulation scripts to be compatible with the new build system.
  These changes include:

  - In BSK python simulation scripts, BSK modules should be included using the indirect method.  Thus::

        from Basilisk.fswAlgorithms.fswModuleTemplate import fswModuleTemplate

    becomes::

        from Basilisk.fswAlgorithms import fswModuleTemplate

  - The ``pyswice`` package is now imported from ``topLevelModule``.  Thus::

        from Basilisk import pyswice

    becomes::

        from Basilisk.topLevelModules import pyswice

  - The support file ``pyswice_ck_utilities.py`` has become a regular suppoort file in ``src/utiliites``.  Thus,
    it is imported using::

        import Basilisk.pyswice.pyswice_ck_utilities

  - Similarly, ``pyswice_spk_utilities.py`` has moved to the utilities folder. To include ``spkRead`` function replace::

        from Basilisk.pyswice.pyswice_spk_utilities import spkRead

    with::

        from Basilisk.utilities.pyswice_spk_utilities import spkRead

  - To include ``loadGravFromFileToList`` function replace::

        from Basilisk.simulation.gravityEffector.gravCoeffOps import loadGravFromFileToList

    with::

        from Basilisk.simulation.gravityEffector import loadGravFromFileToList

- If you have written custom BSK modules outside of the BSK distribution, the swig ``*.i`` files and some code files
  will need to be adjusted as follows:

  - To include the ``swig_common_model.i`` file, replace::

        %include "swig_common_model.i"

    with::

        %pythoncode %{
        from Basilisk.simulation.swig_common_model import *
        %}

  - If Eigen variables are being swig'd, then import::

        %include "swig_eigen.i"

  - To swig C arrays of variables, then import::

        %include "swig_conly_data.i"

  - To provide support of C++ ``std::string`` `types <http://www.swig.org/Doc1.3/Library.html#Library_nn14>`__, then import::

        %include "std_string.i"

  - To provide support of C++ ``std::vector`` `class <http://www.swig.org/Doc1.3/Library.html#Library_nn15>`__, then import::

        %include "std_vector.i"

- The files in ``_GeneralModuleFiles`` folders are now built into a separate library with the parent folders name
  plus ``Lib``.
  This means in the IDE like Xcode and Visual Studio the code in ``_GeneralModuleFiles`` is shown in a folder with
  this library name.  Thus, for example, code in ``src/simulation/environment/_GeneralModuleFiles``
  are shown in the IDE folder ``environmentLib`` within the ``environment`` parent folder.  This keeps the BSK
  folders cleaner and with less duplicated code being displayed.
- A new python package dependency is ``Pillow``.  This is needed for the test scripts for :ref:`camera` to run.
- In Xcode the build will complain that it can't find the ``<Eigen/Dense>`` library.  The code compiles ok.  The work
  around this conan issue is to run the build twice, once for Debug and once for Release.  At that point it can
  be run just once.

**Version 1.7.5**

- :ref:`groundLocation` was not converting between the planet and inertial frame correctly.  This is now fixed in
  the later releases.

**Version 1.7.4**

- None

Version 1.7.3

- On Windows Basilisk didn't compile due to missing math ``#define`` delaration in ``geodeticConversion.cpp/h``.
  This is fixed in the latest release.

**Version 1.7.2**

- None

**Version 1.7.1**

- None

**Version 1.7.0**

- None

**Version 1.6.0**

- None

**Version 1.5.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.5.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- Here the reaction wheel dynamics have been modified such that the RW state output message is no longer hard-coded to ``rw_config_0_data``, etc.  Rather, now the ``ModelTag`` string is pre-pended to make this output msg name unique with.  Any scripts that is logging this RW state message will have to be updated.  The reason for this change is to allow multiple spacecraft to have RW devices and unique RW state messages.
- There was an issue doing a clean compile using Python 2 which is addressed in the next version
- :ref:`test_reactionWheelStateEffector_integrated` didn't run on Python 2, this is fixed in the next version.

**Version 1.4.2**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.4.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- We ran into issues compiling on Linux where ``libsodium`` and ``conan`` were not compiling properly  This is fixed in the next point release.

**Version 1.4.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- ``BSK_PRINT`` has been replaced within Basilisk modules using :ref:`bskLogging` (for C++) and ``_bskLog`` (for C).
- WINDOWS ONLY: there appears to be an issue compiling ``vizInterface`` with the new bskLog method on Windows.  We are working a point release that will fix this.

**Version 1.3.2**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.3.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.3.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.2.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.


**Version 1.2.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- The magnetometer unit tests don't pass on all platforms. This is corrected in the next release.

**Version 1.1.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- the unit tests of the magnetometer module don't pass on all operating systems as the test tolerances are too tight.  This is resolved in the next release.

**Version 1.0.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.9.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.2**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.2**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The enableUnityViz python function how has different inputs. Earlier
python scripts must be updated. See the scenarios for examples. The
arguments are now provided as optional keywords.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

This version of Basilisk no longer support the ASIO module that
communicated with the Qt-based visualization as the BOOST library has
been removed. This visualization has been replaced with the new Vizard
visualization.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``thrMomentumDumping`` now reads in a 2nd required input message to
determine if a new dumping sequence is requested.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

the ``exponentialAtmosphere`` module has been replaced with a module
based on the new atmospheric density base class. BSK simulations that
used the older module must update to use the new module. The module unit
test scripts illustrate how to use this module, and the module PDF
documentation discusses this as well. The ``dragEffector`` integrated
test is also updated to make use of the new module

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``MRP_Feedback()`` has the control vector ``domega0`` removed and
keeps this term now as a permanent zero vector. Any code that was
setting this needs to be updated to not set this parameter anymore.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``numpy`` python package can’t be the current version 1.16.x as this
causes some incompatibilities and massive amounts of depreciated
warnings. These warnings are not related to BSK python code, but other
support code. Thus, for now be sure to install version 1.15.14 of
``numpy``.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Version 4.x.x and higher of pytest works again with Basilisk. You are
free to install the latest version of pytest.

.. raw:: html

   </li>

.. raw:: html

   <li>

As we are now using the conan package management system, you can’t
double the the Cmake GUI application. Instead, you must either launch
the Cmake GUI application from the command line, or run CMake from the
command line directly. See the platform specific Basilisk installation
instructions.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``numpy`` python package can’t be the current version 1.16.x as this
causes some incompatibilities and massive amounts of depreciated
warnings. These warnings are not related to BSK python code, but other
support code. Thus, for now be sure to install version 1.15.14 of
``numpy``.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.3**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The latest version of pytest (version 3.7.1) has a conflict with the
RadiationPressure module unit test. We are still investigating. In the
meantime, using pytest version 3.6.1 is working correctly.

.. raw:: html

   </li>

.. raw:: html

   </ul>
