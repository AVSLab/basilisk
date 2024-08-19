.. toctree::
   :hidden:

.. _pipInstall:

Advanced: (Beta) Building and Installing Pre-Compiled Basilisk Wheels
=====================================================================

.. warning::

    This method of building Basilisk is currently a beta feature, and should only be attempted by advanced users
    familiar with `Python packaging and distribution
    <https://packaging.python.org/en/latest/guides/distributing-packages-using-setuptools/>`_.
    This method is not yet guaranteed to work on every platform, and there are still some annoyances
    to iron out, but we appreciate any feedback you may have.

    Most users should see :ref:`configureBuild` for documentation on the regular build process.


Building and Installing with ``pip``
------------------------------------

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

On its own, there is no significant benefit to installing Basilisk in this way. However, supporting standard Python
packaging tools means that Basilisk can now be built into a pre-compiled `"wheel" (.whl) file
<https://packaging.python.org/en/latest/guides/distributing-packages-using-setuptools/#wheels>`_ that can be shared
and installed on other compatible machines.

Using ``pip``, the command below will generate a ``Basilisk-*.whl`` file in the current directory::

    pip wheel --no-deps -v .

The resulting wheel file can then be installed using ``pip``::

    pip install Basilisk-*.whl

The main benefit of this approach will come in the future, when a set of pre-compiled wheels will be made available,
allowing most users to easily ``pip install Basilisk`` without compilation, in the same way that packages like
``numpy``, ``scipy``, and ``pandas`` are available.


Wheel Compatibility
-------------------

.. warning::

    Wheel compatibility is currently being verified across Python versions and different platforms. So far,
    some users have reported successfully installing the same wheel on different Ubuntu versions and with
    different Python versions, but your mileage may vary.

    If you choose to try this feature and run into and problems, please `raise an issue
    <https://github.com/AVSLab/basilisk/issues>`_ for the Basilisk maintainers to investigate.

When installed using a standard Python packaging tool such as ``pip``, Basilisk will automatically be
built using the `Python Limited C API <https://docs.python.org/3/c-api/stable.html#limited-c-api>`_. The
resulting wheel file will contain a set of compiled Python extension modules that, at least in theory, can be
run using any compatible Python version (currently, any from Python 3.8 to 3.11).

To maintain backwards compatibility, standard Basilisk installations via ``python conanfile.py`` are not
compatible between different Python versions, as is stated in :ref:`customPython`. However, users can
also forcibly build with the Python Limited C API by providing the ``--pyLimitedAPI`` flag::

    python conanfile.py --pyLimitedAPI 0x030800f0


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

Editable installations (i.e. ``pip install -e .``) are not currently supported for two main reasons:

1. ``pip install -e .`` is currently overridden to maintain the existing ``python conanfile.py`` behaviour.
2. Due to the underlying CMake build system, editable installs with ``pip`` require setting the
   ``--no-build-isolation`` option, which means that the user must manually install the build requirements.
   Since these build requirements are installed automatically by the standard ``conanfile.py`` installation,
   it is the recommended method for editable installations.

Tools such as `cibuildwheel <https://cibuildwheel.pypa.io/en/stable/>`_ can be used to build wheels that are
automatically compatible with a broad range of operating system versions. These pre-compiled wheels can then
be shared publicly using tools like `twine <https://pypi.org/project/twine/>`_ to upload them to the
`Python Package Index (PyPi) <https://pypi.org/>`_, allowing users to simply ``pip install Basilisk``. We
expect that these tools will be added into the Basilisk Continuous Integration pipeline in the future.

Further discussions and implementation details can be found in
`Basilisk merge request #737 <https://github.com/AVSLab/basilisk/pull/737>`_.


.. rubric:: Footnotes

.. [#f1] "Standard Python packaging tools" are those which comply with modern Python packaging standards
   such as `PEP-517 <https://peps.python.org/pep-0517/>`_.
