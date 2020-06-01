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

Graphing via datashader
-----------------------

Installing
~~~~~~~~~~

In order to run the full ``datashader`` capabilities of the :ref:`Monte Carlo example <scenarioMonteCarloAttRW>`, you must run the following commands::

   pip3 install --user datashader
   pip3 install --user holoviews

Installing ``datashader`` will automatically install ``bokeh`` and
``pandas`` packages. It is possible to use just ``pandas`` and
``datashader`` to output images of the data; however, without
``holoviews`` and ``bokeh`` there will be no graph axis, title, etc.

Further Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here is a list of documents about the related packages to
``datashader``:

-  `Datashader.org <http://datashader.org/>`__
-  `Many notebook examples of using
   datashader <https://anaconda.org/jbednar/notebooks>`__
-  `Using holoviews and
   datashader <http://holoviews.org/user_guide/Large_Data.html>`__
-  `Plotting with bokeh and
   holoviews <http://holoviews.org/user_guide/Plotting_with_Bokeh.html>`__
-  `Tip for using holoviews outside of a notebook
   context <https://github.com/ioam/holoviews/issues/2376>`__
-  `Another tip for using holoviews and bokeh outside of a notebook
   context <https://github.com/ioam/holoviews/issues/1819>`__
-  `Deploying Bokeh
   Apps <http://pyviz.org/tutorial/13_Deploying_Bokeh_Apps.html>`__
-  `Running a bokeh
   server <https://bokeh.pydata.org/en/latest/docs/user_guide/server.html>`__
-  `Pandas
   dataframes <https://pandas.pydata.org/pandas-docs/stable/generated/pandas.DataFrame.html>`__

Important features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Incorporating ``holoviews``, ``datashader``, and ``bokeh``, we can now rasterize large amounts of data and plot them faster than using ``matplotlib``. Theoretically, the number of points is now irrelevant
while plotting. Using ``datashader``, it is now possible to plot 5 million points in 30 seconds. Aggregating the data (2.5 gigs) took 1 minute to populate the dataframes, and 2 minutes to write to file (which
is only needed if you want to avoid running the monte carlo again). To graph the existing without re-running the simulations, set ``ONLY_DATASHADE_DATA = 1`` in the `Monte Carlo scenarios <@ref%20MonteCarloSimulation>`__.

In order to generate graphs that are zoomed in to a specific x and y range modify the following. For ``holoviews`` and ``bokeh`` interface::

   plot.x_range = Range1d(df.x.min(), df.x.max())
   plot.y_range = Range1d(df.y.min(), df.y.max())

The analagous lines to zoom using just ``datashader`` are::

   x_range = df.x.min(), df.x.max()
   y_range = df.y.min(), df.y.max()

After installing all of the packages, ``pytest`` will use those libraries by default. To change this back to ``matplotlib`` modify the ``pytest`` parameters in ``test_scenarioMonteCarloAttRW.py`` to the
following::

   @pytest.mark.parametrize("MCCases, datashader",
                            [(1, False),
                             (2, False)])

