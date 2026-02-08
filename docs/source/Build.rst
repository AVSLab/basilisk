Building from Source
====================

.. _bskInstall-build:

For advanced users and developers who want to customize or debug Basilisk. For
most users the precompiled version available on PyPI is sufficient. See the
:ref:`Install Instructions <bskInstall>` for more information.

.. note::
   To use custom C++ modules, Basilisk must be built from source.
   The prebuilt PyPI wheels are designed for most users and include all standard
   features, but they do not yet support linking external C++ modules.
   We're actively exploring ways to enable this in future releases.

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   Build/pullCloneBSK
   Build/installOnLinux
   Build/installOnMacOS
   Build/installOnWindows
   Build/installBuild
   Build/buildExtModules
   Build/customPython
   Build/installBuildConan
   Build/pipInstall
