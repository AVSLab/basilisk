.. _bskInstall:

Install
=======

Basilisk can be installed either from `PyPI <https://pypi.org/project/bsk/>`_ as
a prebuilt wheel or built locally from source.
The prebuilt wheels include all build options, such as optical navigation
and MuJoCo dynamics, but do **not** support linking external C++ modules, as
this requires rebuilding Basilisk.

If you want to use custom C++ modules, or prefer smaller install sizes by
excluding unused features, you must build Basilisk from source.
See the :ref:`Building from Source <bskInstall-build>` for more information.

.. note::
   We are currently investigating ways to allow users to link external C++
   modules while using the prebuilt PyPI wheel. Stay tuned!


The easiest way to install Basilisk is using ``pip`` to install the prebuilt
package from PyPI. Run:

.. code-block:: bash

   pip install bsk

Or, if using `uv <https://docs.astral.sh/uv/>`_ (a modern Python package manager):

.. code-block:: bash

   uv pip install bsk

This installs the latest stable version of Basilisk and all dependencies.
To install a specific version, run:

.. code-block:: bash

   pip install bsk==<version>

Replace ``<version>`` with the desired release number, e.g. ``2.9.0``.


**Prebuilt wheel availability:**

- **Windows:** Windows 10/11 (x86_64)
- **macOS:** macOS 11+ (x86_64 and Apple Silicon arm64)
- **Linux:** Manylinux 2.24+ (x86_64, aarch64)

All wheels are built as **ABI3** packages for Basilisk supported Python
versions.

.. note::
   On unsupported systems or Python versions, ``pip`` will automatically
   download the source archive (``.tar.gz``) and build Basilisk locally.
   This requires a C++ compiler toolchain and standard build tools
   to be installed on your system.

To keep the wheel size smaller, the large BSK data files are not installed by
default.  If the user wants to use a script that that needs BSK data it will
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
