# Installing Optional Packages{#installOptionalPackages}


## Graphing via datashader

In order to run the full `datashader` capabilities of the  [Monte Carlo scenarios](@ref MonteCarloSimulation), you must run the following commands (either as sudo or with the `--user` flag):

```
pip install datashader
pip install holoviews
```
Installing `datashader` will automatically install `bokeh` and `pandas` packages.  It is possible to use just `pandas` and `datashader` to output images of the data; however, without `holoviews` and `bokeh` there will be no graph axis, title, etc.


**macOS NOTE:** if you are using the built-in Python 2.7 installation, you may need to install with the `--user` flag and adjust your `PATH` variable to include `~/Library/Python/2.7/bin`


Here is a list of documents about the related packages to `datashader`:

* [Datashader.org](http://datashader.org/)
* [Many notebook examples of using datashader](https://anaconda.org/jbednar/notebooks)
* [Using holoviews and datashader](http://holoviews.org/user_guide/Large_Data.html)
* [Plotting with bokeh and holoviews](http://holoviews.org/user_guide/Plotting_with_Bokeh.html)
* [Tip for using holoviews outside of a notebook context](https://github.com/ioam/holoviews/issues/2376)
* [Another tip for using holoviews and bokeh outside of a notebook context](https://github.com/ioam/holoviews/issues/1819)
* [Deploying Bokeh Apps](http://pyviz.org/tutorial/13_Deploying_Bokeh_Apps.html)
* [Running a bokeh server](https://bokeh.pydata.org/en/latest/docs/user_guide/server.html)
* [Pandas dataframes](https://pandas.pydata.org/pandas-docs/stable/generated/pandas.DataFrame.html)

###Important features of `datashader`

Incorporating `holoviews`, `datashader`, and `bokeh`, we can now rasterize large amounts of data and plot them faster then using `matplotlib`. Theoretically, the number of points is now irrelevant while plotting. Using `datashader`, it is now possible to plot 5 million points in 30 seconds. Aggregating the data (2.5 gigs) took 1 minute to populate the dataframes, and 2 minutes to write to file (which is only needed if you want to avoid running the monte carlo again). To graph the existing without re-running the simulations, set `ONLY_DATASHADE_DATA = 1` in the [Monte Carlo scenarios](@ref MonteCarloSimulation). 

In order to generate graphs that are zoomed in to a specific x and y range modify the following:

For `holoviews` and `bokeh` interface:

```
plot.x_range = Range1d(df.x.min(), df.x.max())
plot.y_range = Range1d(df.y.min(), df.y.max())
```

The analagous lines to zoom using just `datashader` are:

```
x_range = df.x.min(), df.x.max()
y_range = df.y.min(), df.y.max()
```

After installing all of the packages, `pytest` will use those libraries by default. To change this back to `matplotlib` modify the `pytest` paramaters  in `test_scenarioMonteCarloAttRW.py` to the following:

```
@pytest.mark.parametrize("MCCases, datashader",
                         [(1, False),
                          (2, False)])
```

## Protobuffers

To use Google Protobuffers in a C++ context by building the source, please follow the following documentation [here](https://github.com/google/protobuf/blob/master/src/). To use Google Protobuffers as a pre-built library, download the release from [here](https://github.com/google/protobuf/releases).
