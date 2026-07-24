.. toctree::
   :maxdepth: 1
   :hidden:

.. _extensionsInstall:

Installing Basilisk Extensions
==============================

This page is for users installing an extension that someone else has built.
To create or compile an extension, follow :ref:`writingExtensions` instead.
See :ref:`bskExtensions` for an explanation of how extensions relate to core
Basilisk and integrated external-folder modules.

An extension is normally distributed as a Python wheel. A compatible prebuilt
wheel can be installed without a compiler, a Basilisk source checkout, or
``bsk-sdk``. The wheel still depends on a specific Basilisk version and must
support the user's operating system, processor architecture, and Python
version.

Create a Clean Environment
--------------------------

Using a virtual environment keeps the extension and its required Basilisk
version separate from other Python projects. The activation command shown is
for macOS and Linux; on Windows use ``.venv\Scripts\activate``.

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip

Use ``python -m pip`` throughout this guide so installation uses the same
Python interpreter that will run the simulation.

Install a Published Extension
-----------------------------

Install the extension by its distribution name. A correctly packaged
extension declares its compatible ``bsk`` release as a dependency, so ``pip``
installs or retains that version automatically:

.. code-block:: bash

   python -m pip install <extension-name>

For example, an extension published as ``my-atm-extension`` is installed with:

.. code-block:: bash

   python -m pip install my-atm-extension

Most users should also request the recommended Basilisk optional components:

.. code-block:: bash

   python -m pip install "bsk[all]" my-atm-extension

The ``all`` extra installs the core Basilisk wheel and all published optional
component wheels. An extension that only requires core Basilisk does not need
this extra. Installing it provides the standard complete Basilisk environment
at the cost of additional packages and download size.

Do not install ``bsk-sdk`` merely to run a prebuilt extension. The SDK contains
headers, message generators, and build tools for extension authors; it is not a
normal runtime dependency.

Install Specific Versions
-------------------------

The extension's own version does not have to match Basilisk. Consult the
extension's compatibility documentation, then constrain both packages when a
reproducible installation is required:

.. code-block:: bash

   python -m pip install \
     "my-atm-extension==1.4.0" \
     "bsk[all]==2.11.0"

If those versions are incompatible according to the extension's package
metadata, ``pip`` stops with a dependency-resolution error instead of creating
an unsafe environment.

Avoid ``--no-deps`` when installing an extension. It bypasses the dependency
metadata that protects the Basilisk ABI relationship.

Install a Local Wheel
---------------------

An extension developer or internal build system may provide a wheel file
directly. Install it by path:

.. code-block:: bash

   python -m pip install /path/to/my_atm_extension-1.4.0-*.whl

``pip`` still resolves the wheel's declared Basilisk dependency. A wheel can
only be installed if its platform and Python compatibility tags match the
current environment.

For a wheel hosted on a private package index, follow the index's authentication
instructions and install it with the appropriate ``--index-url`` or
``--extra-index-url`` option. Do not place credentials directly in shell
history or documentation.

Verify the Installation
-----------------------

First ask ``pip`` to verify that every installed dependency is satisfied:

.. code-block:: bash

   python -m pip check

Inspect the installed distribution and Basilisk version:

.. code-block:: bash

   python -m pip show my-atm-extension bsk
   python -c "import Basilisk; print(Basilisk.__version__)"

Finally, import the extension using its Python package name. The distribution
name used by ``pip`` may contain hyphens, while the import package normally uses
underscores:

.. code-block:: python

   import Basilisk
   import my_atm_extension

   print("Basilisk:", Basilisk.__version__)
   print("Extension imported from:", my_atm_extension.__file__)

Instantiate one of the extension's modules or run its supplied smoke test
before using it in a larger simulation:

.. code-block:: python

   from my_atm_extension import customAtmosphere

   atmosphere = customAtmosphere.CustomAtmosphere()
   print(atmosphere.ModelTag)

The extension's documentation should identify its import package, exported
modules, required message connections, and any initialization test.

Version and ABI Compatibility
-----------------------------

An extension wheel contains native code compiled against a specific Basilisk
C/C++ and SWIG interface. Its compatible Basilisk version is determined when
the wheel is built, not when the extension is imported.

For example, ``my-atm-extension==1.4.0`` might have been built with
``bsk-sdk==2.11.0`` and declare ``bsk==2.11.0`` as its runtime dependency. The
extension version remains ``1.4.0``; it does not need to copy the Basilisk
version number.

Compatibility is enforced differently depending on how the extension is
obtained:

* **Prebuilt wheel:** ``pip`` uses the extension's declared ``bsk`` dependency
  to select a compatible Basilisk release.
* **Source build:** ``bsk-sdk`` checks the installed Basilisk version and SWIG
  runtime while CMake configures the extension.

If a prebuilt extension does not declare an exact Basilisk dependency, its
documentation must state the supported Basilisk version. Treat missing or vague
compatibility information as a packaging defect and ask the extension
maintainer for clarification.

Wheel Availability and Source Builds
------------------------------------

Extension wheels are platform-specific native artifacts. Publishers normally
build separate wheels for each supported operating system and processor
architecture, with Python compatibility encoded in the wheel filename.

If no compatible wheel is available, ``pip`` may report ``No matching
distribution found`` or download a source archive and attempt a local build. A
source build requires the tools described in :ref:`writingExtensions`,
including a C++17 compiler, CMake, ``bsk-sdk``, and the matching Basilisk
release.

Users who only want to run the extension should normally request a compatible
wheel from its publisher rather than setting up a native development
toolchain.

Upgrade an Extension
--------------------

Upgrade the extension and let its dependency metadata select the compatible
Basilisk version:

.. code-block:: bash

   python -m pip install --upgrade "bsk[all]" my-atm-extension
   python -m pip check

When deliberately moving to a specific Basilisk release, confirm that a
compatible extension release exists and upgrade them together:

.. code-block:: bash

   python -m pip install --upgrade \
     "bsk[all]==2.11.0" \
     "my-atm-extension==1.4.0"
   python -m pip check

Do not upgrade only ``bsk`` and ignore a dependency conflict. A wheel built for
an older Basilisk ABI must be replaced by a compatible wheel or rebuilt by the
extension maintainer.

Uninstall an Extension
----------------------

Remove the extension by its distribution name:

.. code-block:: bash

   python -m pip uninstall my-atm-extension

This does not normally remove Basilisk because other installed packages or the
user may still require it.

Common Installation Problems
----------------------------

``ResolutionImpossible`` or conflicting dependencies
   The requested extension and Basilisk versions are incompatible. Install a
   documented compatible pair rather than forcing the resolver to ignore the
   conflict.

``No matching distribution found``
   The requested extension version may not exist, or the publisher may not
   provide a wheel for the current operating system, architecture, or Python
   version. Check the available wheel filenames or contact the publisher.

``pip`` starts compiling the extension
   No compatible prebuilt wheel was selected, so ``pip`` downloaded a source
   distribution. Cancel the build if a compiler toolchain was not intended and
   obtain a compatible wheel instead.

An optional Basilisk module cannot be imported
   Install the appropriate Basilisk extra. The recommended complete set is
   ``python -m pip install "bsk[all]"``.

The extension imports in one terminal but not another
   The terminals are using different Python environments. Compare
   ``python -c "import sys; print(sys.executable)"`` and reinstall using that
   interpreter's ``python -m pip``.

Undefined symbols, SWIG type errors, or a crash during import
   These can indicate an extension wheel built for a different Basilisk or SWIG
   ABI, especially if it was installed with ``--no-deps``. Create a clean
   environment, install a documented compatible package pair, and report a
   reproducible failure to the extension maintainer if it persists.
