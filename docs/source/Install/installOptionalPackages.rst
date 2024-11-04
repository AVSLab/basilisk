.. toctree::
   :hidden:



.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/8Ex1YE7YnoY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

.. _installOptionalPackages:

Installing Optional Packages
============================

This page contains all the optional python packages that can be included to unlock additional Basilisk
features or utilities.  For convenience the complete set of optional packages, including any constraints on
acceptable versions, are listed here:

.. literalinclude:: ../../../requirements_optional.txt
   :language: python

To automatically ensure that the system has all optional packages installed, use the ``allOptPkg``
flag as discussed in :ref:`configureBuild`.


Creating the Sphinx Basilisk Documentation
------------------------------------------
Go to :ref:`createHtmlDocumentation` to learn what associated python tools are required.
The following python packages must be installed via ``pip``::

    pip3 install sphinx sphinx_rtd_theme breathe recommonmark docutils

See the list at the top of this page for what versions of these packages are acceptable.


Formatting Code Files using ``pre-commit`` and ``clang-format``
---------------------------------------------------------------
If you are developing new code to contribute back to Basilisk it must follow the
:ref:`codingGuidelines`.  This requires installing::

    pip3 install pre-commit clang-format

The file `CONTRIBUTING.md <https://github.com/AVSLab/basilisk/blob/develop/CONTRIBUTING.md>`__
explains how to setup and use these code formating tools.
