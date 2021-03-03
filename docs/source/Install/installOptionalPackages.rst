.. toctree::
   :hidden:


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

    pip3 install --user pytest

Note that version 4.0.1 or higher works properly with Basilisk, while versions between 3.6.1 and 4.0.0 had some bugs that impacted some Basilisk tests.

If you want to use ``pytest`` to generate a validation HTML report using the ``--report`` argument,
then the ``pytest-html`` package must be installed::

   pip3 install --user pytest-html


Running ``pytest`` in a multi-threaded manner
---------------------------------------------

While Basilisk is a single threaded simulation, it is possible to run
``pytest`` in a multi-threaded manner. Install the ``pytest-xdist``
package using::

   pip3 install --user pytest-xdist

After installing this utility you now run the multi-threaded version of
``pytest`` for 8 threads using::

   python3 -m pytest -n 8

or replace 8 with the number of cores your computer has available


Creating the Sphinx Basilisk Documentation
------------------------------------------
Go to :ref:`createHtmlDocumentation` to learn what associated python tools are required.
The following python packages must be installed via ``pip``::

    pip3 install --user sphinx sphinx_rtd_theme breathe recommonmark

See the list at the top of this page for what versions of these packages are acceptable.


