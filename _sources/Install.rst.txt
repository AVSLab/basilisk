.. _bskInstall:

Install
=======

Basilisk can be installed either from `PyPI <https://pypi.org/project/bsk/>`_ as
a prebuilt wheel or built locally from source.
The recommended prebuilt installation uses the ``all`` extra so ``pip`` installs
the core Basilisk wheel plus all optional Basilisk component wheels. MuJoCo
dynamics are included in the core wheel, while optical navigation is provided by
the optional ``bsk-opnav`` package. Prebuilt wheels can be used with
:ref:`Basilisk extensions <writingExtensions>` for out-of-tree C++ modules, but they
cannot be modified in place to include legacy ``ExternalModules`` through the
``pathToExternalModules`` build option.

If you want to use legacy ``ExternalModules`` or native build options not
covered by the published package extras, you must build Basilisk from source. If
you prefer a smaller prebuilt install and do not need optional Basilisk
components, use the core ``bsk`` package variant described below. See the
:ref:`Building from Source <bskInstall-build>` for more information.


The easiest way to install Basilisk is using ``pip`` to install the prebuilt
package from PyPI. Run:

.. code-block:: bash

   pip install "bsk[all]"

The most common install variants are:

- ``bsk``: core Basilisk wheel only
- ``bsk[opnav]``: core Basilisk plus optical navigation components
- ``bsk[all]``: recommended install with all optional Basilisk components

If you also want the optional Python packages used by example scripts, install:

.. code-block:: bash

   pip install "bsk[all,examples]"

Or, if using `uv <https://docs.astral.sh/uv/>`_ (a modern Python package manager):

.. code-block:: bash

   uv pip install "bsk[all]"

This installs the latest stable version of Basilisk and all recommended
dependencies.
To install a specific version, run:

.. code-block:: bash

   pip install "bsk[all]==<version>"

Replace ``<version>`` with the desired release number, e.g. ``2.11.0``.

Container images are also available for users who prefer Docker-based
workflows. See :ref:`bskContainers` for registry locations, tags, and
architecture support.


**Prebuilt wheel availability:**

- **Windows:** Windows 10/11 (x86_64)
- **macOS:** macOS 11+ (Apple Silicon arm64)
- **Linux:** Manylinux 2.24+ (x86_64, aarch64)

All wheels are built as **ABI3** packages for Basilisk supported Python
versions.

.. note::
   On unsupported systems or Python versions, ``pip`` will automatically
   download the source archive (``.tar.gz``) and build Basilisk locally.
   This requires a C++ compiler toolchain and standard build tools
   to be installed on your system.

Nightly Builds
--------------

Nightly pre-release wheels are built automatically from the ``develop`` branch and
published to the Basilisk nightly index for all supported platforms. These
builds are replaced each night and reflect the latest state of active development.
They are intended for testing and early access to new features, but may be
unstable and are not recommended for production use.

To install the latest nightly build, use the Basilisk nightly index as the primary
source, allow pip to fall back to PyPI for dependencies, and pass ``--pre`` since
nightly wheels are pre-release versions:

.. code-block:: bash

   pip install \
     --index-url https://avslab.github.io/basilisk/nightly/ \
     --extra-index-url https://pypi.org/simple/ \
     --pre \
     "bsk[all]"

Because nightly wheels are built from the ``develop`` branch and carry the same
version string until ``bskVersion.txt`` changes, pip may report
``Requirement already satisfied`` even when a newer build is available. To
force an overwrite of whatever is currently installed, add
``--upgrade --force-reinstall --no-cache-dir``:

.. code-block:: bash

   pip install \
     --index-url https://avslab.github.io/basilisk/nightly/ \
     --extra-index-url https://pypi.org/simple/ \
     --pre \
     --upgrade --force-reinstall --no-cache-dir \
     "bsk[all]"

.. warning::

   Nightly builds are development snapshots and are not suitable for production use.
   For stable releases, install from `PyPI <https://pypi.org/project/bsk/>`_ as described above.


Basilisk Support Data
---------------------

To keep the wheel size smaller, the large BSK data files are not installed by
default.  If the user wants to use a script that needs BSK data it will
be downloaded automatically using ``pooch``.  If you want to force all data to
be downloaded at once, then go to the command line, change the current
directory to be inside the environment where Basilisk was ``pip`` installed,
and run the command::

    bskLargeData

This command runs a python file stored in the ``src/utilities`` folder.
The ``pip install`` process automatically
creates this console command in the current python environment to call this
python file.  The file directly downloads the missing large BSK data files and put
them into the local Basilisk python package installation.

Basilisk Examples
-----------------

If you want to run the example files provided by the Basilisk repository, they can be downloaded to the local
directory using the command line::

    bskExamples
