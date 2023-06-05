.. toctree::
   :hidden:



.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/8Ex1YE7YnoY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

.. _installOptionalPackages:

Installing Optional Packages
============================

Listing of All Optional Packages
--------------------------------
This page contains all the optional python packages that can be included to unlock additional Basilisk
features or utilities.  For convenience the complete set of optional packages, including any constraints on
acceptable versions, are listed here:

    .. include:: ../bskPkgOptions.txt

To automatically ensure that the system has all optional packages installed, use the ``allOptPkg``
flag as discussed in :ref:`configureBuild`.

Running unit and integrated tests via ``pytest``
------------------------------------------------

The ``pytest`` program can run a series of test on python scripts that begin with ``test_``. Install the ``pytest`` program with::

    pip3 install pytest

Note that version 4.0.1 or higher works properly with Basilisk, while versions between 3.6.1 and 4.0.0 had some bugs that impacted some Basilisk tests.

Generating HTML Report of the ``pytest`` Results
------------------------------------------------
If you want to use ``pytest`` to generate a validation HTML report,
then the ``pytest-html`` package must be installed::

   pip3 install pytest-html

When running ``pytest`` from the terminal, add the ``--html`` argument followed by the path to where
to generate the report html folder.  It is recommended to put this inside a folder as HTML
support folder will be created::

    pytest --html report/report.html


Multi-proccesing ``pytest``
---------------------------

One can distribute python tests across multiple processes. This is achieved with the ``pytest-xdist``
package using::

   pip3 install pytest-xdist

After installing this package you can now pytest such that it distribute tests across multi-processes.
``pytest`` for the same number of processes as processors on the host machine using::

   python3 -m pytest -n auto

or replace `auto` with the number of processors (virtual or otherwise) you'd like to dedicate as test executions
proccesses.


Creating the Sphinx Basilisk Documentation
------------------------------------------
Go to :ref:`createHtmlDocumentation` to learn what associated python tools are required.
The following python packages must be installed via ``pip``::

    pip3 install 'sphinx<5.0' sphinx_rtd_theme==0.5.1 breathe recommonmark docutils

See the list at the top of this page for what versions of these packages are acceptable.


