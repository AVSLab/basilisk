Building from Source
====================

.. _bskInstall-build:

For advanced users and developers who want to customize or debug Basilisk. For
most users the precompiled version available on PyPI is sufficient. See the
:ref:`Install Instructions <bskInstall>` for more information.

.. note::
   To use the legacy ``ExternalModules`` path that compiles custom modules into
   the Basilisk package, Basilisk must be built from source with
   ``pathToExternalModules``.
   The recommended prebuilt PyPI install, ``pip install "bsk[all]"``, is designed
   for most users and includes all optional Basilisk component wheels. For
   out-of-tree C++ modules that do not need to live inside
   ``Basilisk.ExternalModules``, use :ref:`Basilisk plugins <writingPlugins>`.

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
   Build/containers
