#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


r"""
Motivation
----------
This script is a basic demonstration of a script that can be used to plot Monte Carlo data with 
bokeh and datashaders.   These tools are very efficient to plot large amounts of simulation data
that is likely to occur with Monte Carlo sensitivity analysis studies.  For example, running this script will
create an HTML interactive view of the simulation data.   Instead of seeing a fixed resolution, the user can
zoom into the data dynamically to see more detail.  This process recreates a newly render view of the simulation data.

The following two plots illustrate what this particular simulation setup will yield.

.. _scenarioAnalyzeMonteCarlo-ds0:
.. figure:: /_images/static/ds-0.png
    :align: center
    :scale: 50%

    Figure 1: Full view of the attitude error plot data

.. _scenarioAnalyzeMonteCarlo-ds1:
.. figure:: /_images/static/ds-1.png
    :align: center
    :scale: 50%

    Figure 2: Zoomed in and nearly rendered view of the attitude error data details

The next plot illustrates the output if you run ``scenario_AttFeedbackMC.py`` with more simulation cases,
40 in this plot.

.. _scenarioAnalyzeMonteCarlo-ds2:
.. figure:: /_images/static/ds-2.png
    :align: center
    :scale: 50%

    Figure 3: Larger simulation run with 40 simulation cases shown

Configuring a Python Environment For this Script
------------------------------------------------
.. danger::

    Running this script is different from running other BSK scripts.  There are very particular python
    package requirements that must be carefully followed.  It is recommended the user create a
    virtual python environment as discussed in the installation setup.  This environment might have to be
    specific to running this script because of these dependency challenges.

The setup steps are as follows:

#. The datashaders etc. require that this script be run with Python 3.7, not higher
#. Create dedicated virtual environment and compile Basilisk for this environment
#. Install this particular version of ``panel`` package first.  It must be done alone as it upgrades
   ``bokeh`` to a version that is too new::

        pip3 install --upgrade panel==0.9.7

#. Next, install the following particular python package versions::

        pip3 install --upgrade bokeh==1.2.0 holoviews==1.12.3 param==1.9.3 hvplot==0.6.0

How to Run the Script
---------------------
.. important::

    Read all three steps before advancing.

The next steps outline how to run this script. 

1.  This script can only be run once there exists data produced by the ``scenario_AttFeedbackMC.py`` script.

2.  At the bottom of this script, comment out the name guard and associated ``run()`` statement,
    and un-comment the following ``run()`` statement before this script can run.
    These lines are provided in their commented/uncommented form
    to ensure that the sphinx documentation generation process does not
    run this script automatically.

3.  This script must be called from command line using::

        /$path2bin/bokeh serve --show /$path2script/scenarioAnalyzeMonteCarlo.py

This will process the data created with ``scenario_AttFeedbackMC.py`` and open a browser window showing
Figure 1 above.  To end the script you need to press the typical key strokes to interrupt a process as the
bokeh server will keep running until stopped.

"""

import inspect
import os
FOUND_DATESHADER = True
try:
    from Basilisk.utilities.datashader_utilities import DS_Plot, curve_per_df_component, pull_and_format_df
    from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter
    from bokeh.plotting import figure
    from bokeh.models import ColumnDataSource
    from bokeh.palettes import RdYlBu9
    from bokeh.palettes import Category10  # Import a color palette
    from bokeh.models import HoverTool, Legend
    from bokeh.layouts import column
    from bokeh.io import curdoc

    from bokeh.palettes import Blues9, Reds9, Greens9, \
        Blues3, Reds3, Greens3, Oranges3, RdYlBu9
except:
    print("Wasn't able to include the datashader_utilities.")
    FOUND_DATESHADER = False

import Basilisk.utilities.macros as macros
import holoviews as hv
from holoviews.operation.datashader import datashade, dynspread
from holoviews import opts
import datashader as ds
import datashader.transfer_functions as tf
import pandas as pd
from bokeh.plotting import figure
from bokeh.models import ColorBar, BasicTicker, LinearColorMapper
from bokeh.palettes import Viridis256

# Load the Bokeh extension for HoloViews
hv.extension('bokeh')

filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__

bskPath = __path__[0]

def plotSuite(dataDir, components):
    plotter = MonteCarloPlotter(dataDir)
    
    # Load the data for both attitude error and attitude rate error
    plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])
    
    # Generate the plots for specified components
    plotter.generate_plots(components)
    
    # Return the dictionary of plots
    return plotter.plots

def create_datashader_plot(df, x, y_columns, title, y_label):
    """
    Create a datashader plot for large datasets with multiple runs and a colorbar.
    
    Args:
        df (pd.DataFrame): Input DataFrame
        x (str): Column name for x-axis (time in seconds)
        y_columns (list): List of column names for y-axis (one per run)
        title (str): Plot title
        y_label (str): Label for y-axis
    
    Returns:
        bokeh.plotting.figure: Bokeh figure object with colorbar
    """
    # Create a color key for each run
    num_runs = len(y_columns)
    color_key = {y: Viridis256[int(i * 255 / (num_runs - 1))] for i, y in enumerate(y_columns)}

    # Create a single DataFrame with all runs and a 'run' column
    all_runs_df = pd.DataFrame({'x': df[x]})
    for y in y_columns:
        all_runs_df[y] = df[y]
        all_runs_df[f'run_{y}'] = y

    # Melt the DataFrame to long format
    melted_df = pd.melt(all_runs_df, id_vars=['x'], value_vars=y_columns,
                        var_name='run', value_name='y')

    # Create a HoloViews Dataset
    dataset = hv.Dataset(melted_df, kdims=['x', 'run'], vdims=['y'])

    # Create Curve element instead of Points
    curves = hv.Curve(dataset, kdims=['x', 'y'], vdims=['run'])

    # Apply datashading with explicit downsampling and increased line width
    shaded = datashade(curves, aggregator=ds.count_cat('run'), color_key=color_key, min_alpha=200)
    
    # Apply dynamic spreading for better visibility when zooming
    spread = dynspread(shaded, max_px=10, threshold=0.5)  # Increased from 5 to 10
    
    # Set plot options
    plot = spread.opts(
        width=800, height=400, 
        title=title,
        xlabel='Time (seconds)', ylabel=y_label,
        tools=['hover', 'pan', 'box_zoom', 'wheel_zoom', 'reset'],
        show_legend=False
    )

    # Convert HoloViews plot to Bokeh figure
    bokeh_plot = hv.render(plot, backend='bokeh')

    # Create a color mapper for the colorbar
    color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=num_runs-1)

    # Create a ColorBar
    color_bar = ColorBar(
        color_mapper=color_mapper,
        ticker=BasicTicker(desired_num_ticks=min(10, num_runs)),
        label_standoff=12,
        border_line_color=None,
        location=(0, 0),
        title="Run Number",
        width=20
    )

    # Add the ColorBar to the plot
    bokeh_plot.add_layout(color_bar, 'right')

    return bokeh_plot

def run(show_plots):
    """
    This script is meant to be configured based on the user's needs. It can be configured using the following
    three booleans:

    :param show_all_data: plot all MC runs for the plots specified in the plotSuite method
    :param show_extreme_data: call plotSuite method for user-defined number of extrema MC runs
    :param optional_plots: plots additional user-defined plots
    """

    show_all_data = True
    show_extreme_data = False
    optional_plots = False

    # Specify which components to plot (0, 1, 2 correspond to x, y, z)
    components_to_plot = [0, 1, 2]

    plotList = []
    analysis = MonteCarloPlotter(path + "/scenario_AttFeedbackMC/")

    # save_as_static: save off static .html files of the plots generated into the staticDir directory.
    # The staticDir will be created inside the dataDir folder.
    # (Note: This inhibits dynamic plotting!)
    analysis.save_as_static = False
    analysis.staticDir = "/plots/"

    if show_all_data:
        try:
            plotter = MonteCarloPlotter(analysis.dataDir)
            plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])
            plotter.generate_plots(components_to_plot)
            
            # Get downsampled plots and plot info
            downsampled_plots = plotter.get_downsampled_plots()
            plot_info = plotter.get_plot_info()
            
            # Create datashader plots for large datasets
            ds_plots = []
            for title, df in downsampled_plots.items():
                y_columns = [col for col in df.columns if col.startswith('run')]
                y_label = plot_info[title]['y_label']
                bokeh_plot = create_datashader_plot(df, x='time', y_columns=y_columns, title=title, y_label=y_label)
                ds_plots.append(bokeh_plot)
            
            # Use column layout with sizing_mode='stretch_both' for responsive layout
            layout = column(*ds_plots, sizing_mode='stretch_both')
            curdoc().add_root(layout)
        except FileNotFoundError as e:
            print(f"Error: {str(e)}")
            print("Please make sure you have run scenario_AttFeedbackMC.py to generate the necessary .data files before running this script.")
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}")
            print(f"Error details: {type(e).__name__}: {str(e)}")
            print(f"DataFrame columns: {df.columns}")  # Add this line for debugging

    if show_extreme_data:
        analysis.variableName = "attGuidMsg.omega_BR_B"
        analysis.variableDim = 1

        extremaRunNumbers = analysis.getExtremaRunIndices(numExtrema=1, window=[500 * 1E9, 550 * 1E9])

        analysis.extractSubsetOfRuns(runIdx=extremaRunNumbers)
        plotList.extend(plotSuite(analysis.dataDir + "/subset", components_to_plot))

    if optional_plots:
        # This section needs to be updated if you want to include optional plots
        pass

# The following must be commented out before this script can run.  It is provided here
# to ensure that the sphinx documentation generation process does not
# run this script automatically.
# if __name__ == "__main__":
#     run(False)

# uncomment the following line to run this script.
run(False)