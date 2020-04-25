
.. _bskKnownIssues:

Basilisk Known Issues
=====================

Version |release|
-----------------
- None

Version 1.7.3
-------------
- None

Version 1.7.2
-------------
- None

Version 1.7.1
-------------
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
