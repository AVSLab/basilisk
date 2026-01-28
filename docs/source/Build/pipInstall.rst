.. toctree::
   :hidden:

.. _pipInstall:

Advanced: Building Pre-Compiled Basilisk Wheels
===============================================

The following instructions explain how to build wheels with Basilisk.
While we distribute pre-compiled wheels already in :ref:`bskInstall`,
you can also build them manually using the steps below. This is desirable if
you want to use Python wheels but still need to enable custom build options
or link against a custom external module location.


Building Wheels and Installing with ``pip``
-------------------------------------------

.. note::

    All commands are called from the Basilisk root directory.

A clean Basilisk wheel can be built and installed using standard Python packaging tools [#f1]_ such as ``pip``.
Note that this will always result in a clean build.
The simplest usage is::

    pip install -v .

This command compiles and installs Basilisk into the user's current Python environment. Note that the optional ``-v`` flag is
added to display verbose compilation messages, otherwise it can look like the installation process is stuck.

Build options (as passed to ``conanfile.py`` and described in :ref:`configureBuild`) can be provided using the
``CONAN_ARGS`` environment variable::

    CONAN_ARGS="--pathToExternalModules='/path/to/external' --opNav True" pip install -v .

.. warning::

    For backwards compatibility reasons, and due to issues arising from temporary build environments, editable
    installations (``pip install -e .``) are not currently supported. Please follow the standard
    :ref:`configureBuild` process.


Building Basilisk ``wheel`` File
--------------------------------

Using ``pip``, the command below will generate a ``bsk-*.whl`` file in the current directory::

    pip wheel --no-deps -v .

The resulting wheel file can then be installed using ``pip``::

    pip install bsk-*.whl

This allows the user to create a custom Basilisk wheel to distribute within their organization.

To keep the wheel size smaller, the BSK data files are not installed by default.  If the user
wants to use script that assumes they are included into the Basilisk python package, then go to the
command line, change the current directory to be inside the environment where Basilisk was ``pip`` installed,
and run the command::

    bskLargeData

This command runs a python file stored in the ``src/utilities`` folder.
The ``pip install`` process automatically
creates this console command in the current python environment to call this python file.  The file
directly downloads the missing BSK data files and put them into a local pooch cache.

.. note::

    If the computer does not have local internet access and the ``pip install`` is done via
    a local wheel, then these missing Spice ``*.bsp`` data files can be manually added to::

        .venv/lib/python3.11/site-packages/Basilisk/supportData/EphemerisData

If installing Basilisk via a wheel the user does not have direct access to the full Basilisk source
folder which contains the ``examples`` folder.  The Terminal command ``bskExamples``
will download a copy of the examples folder into the local directory.

Alternatively, if you download a zip'd folder of the Basilisk source code you can install it via ``pip``
using::

    pip install Basilisk*.tar.gz

The following command is used to both download the code and compile Basilisk with pip::

    pip install git+https://github.com/AVSLab/basilisk.git


For Maintainers: Overview of Implementation
-------------------------------------------

Python packaging support is provided through the ``pyproject.toml`` file as specified by the `PEP-517
<https://peps.python.org/pep-0517/>`_ standard. This file defines the desired "back-end" tool used to build
Basilisk, as well as other packaging settings including which Python packages are required for building
and running Basilisk.

At the time of this writing, the build backend is ``setuptools``, which invokes the ``setup.py`` file to
perform the build. In ``setup.py``, the Python extension builder is overridden with a custom builder that
computes an appropriate Python Limited C API version (based on the minimum supported Python version
specified in ``pyproject.toml``). The builder then invokes ``python conanfile.py``, setting the
``--managePipEnvironment False`` option so that Conan does not directly manipulate the user's ``pip``
environment. The main reasons for this setting was to maintain the current default behaviour of
``conanfile.py``-based installation.

Editable installations (i.e. ``pip install -e .``) are are partially supported. Python code changes are reflected
automatically with this flag, but C++ components are not automatically rebuilt.

This limitation exists because editable mode still routes through ``python conanfile.py``, but skips C++ rebuilds
once the initial build exists. This avoids long native rebuilds and preserves the Conan/CMake configuration.
...

.. rubric:: Footnotes

.. [#f1] "Standard Python packaging tools" are those which comply with modern Python packaging standards
   such as `PEP-517 <https://peps.python.org/pep-0517/>`_.
