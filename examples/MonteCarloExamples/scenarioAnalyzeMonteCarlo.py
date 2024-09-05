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
    from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import mcAnalysisBaseClass
    from bokeh.plotting import figure
    from bokeh.models import ColumnDataSource
    from bokeh.palettes import RdYlBu9
    from bokeh.palettes import Category10  # Import a color palette
    from bokeh.models import HoverTool, Legend
    from bokeh.layouts import column
    from bokeh.models import WheelZoomTool
    from bokeh.io import curdoc

    from bokeh.palettes import Blues9, Reds9, Greens9, \
        Blues3, Reds3, Greens3, Oranges3, RdYlBu9
except:
    print("Wasn't able to include the datashader_utilities.")
    FOUND_DATESHADER = False

import Basilisk.utilities.macros as macros


filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__

bskPath = __path__[0]

def reshape_df_for_plotting(df):
    """
    Reshape DataFrame from wide format with MultiIndex columns to long format.
    Args:
        df: DataFrame with MultiIndex columns
    Returns:
        Reshaped DataFrame suitable for plotting
    """
    # Reset MultiIndex columns to a single level with appropriate names
    df.columns = [f'{i}_{j}' for i, j in df.columns]
    
    # Reset index to make it a column and rename the columns
    df = df.reset_index()
    
    # Melt DataFrame to long format, assuming 'time' is the index and should be retained
    df_long = df.melt(id_vars=['time[ns]'], var_name='variable', value_name='value')
    
    # Extract run number and component from the variable name
    df_long[['runNum', 'component']] = df_long['variable'].str.split('_', expand=True)
    
    # Drop the original variable column
    df_long = df_long.drop(columns=['variable'])
    
    # Rename 'time[ns]' to 'time' for simplicity
    df_long = df_long.rename(columns={'time[ns]': 'time'})
    
    return df_long

def plotSuite(dataDir):
    """
    Create interactive plots for each dataset using Bokeh, color-coded by run number,
    with time converted to seconds, and interactive tools.
    
    Args:
        dataDir: Directory containing data files
    
    Returns:
        List of Bokeh plots
    """
    plotList = []
    
    # Define a color map using a Bokeh color palette
    color_map = Category10[10]  # You can use other palettes or specify your own colors
    
    # Create a plot for attitude error
    sigma_BR = pull_and_format_df(dataDir + "attGuidMsg.sigma_BR.data", 3)
    sigma_BR = reshape_df_for_plotting(sigma_BR)
    sigma_BR['time'] = sigma_BR['time'] / 1e9  # Convert time from nanoseconds to seconds
    
    p1 = figure(title="Attitude Error", x_axis_label='time [s]', y_axis_label='Sigma_BR',
                width=800, height=400, tools="pan,wheel_zoom,box_zoom,reset,hover,save")
    
    # Set wheel_zoom as the active scroll tool
    p1.toolbar.active_scroll = p1.select(dict(type=WheelZoomTool))[0]

    # Add hover tool
    hover = HoverTool()
    hover.tooltips = [("Run", "@runNum"), ("Time", "@time{0.00}"), ("Value", "@value{0.00}")]
    p1.add_tools(hover)
    
    lines = []
    legend_items = []
    for i, run_num in enumerate(sigma_BR['runNum'].unique()):
        df_run = sigma_BR[sigma_BR['runNum'] == run_num]
        color = color_map[i % len(color_map)]  # Cycle through colors if there are more runs than colors
        source = ColumnDataSource(df_run[df_run['component'] == '0'])
        line = p1.line(x='time', y='value', source=source,
                       legend_label=f'Run {run_num}', line_width=2, color=color)  # Adjust color and line width as needed
        lines.append(line)
        legend_items.append((f'Run {run_num}', [line]))

    
    plotList.append(p1)

    # Create a plot for attitude rate error
    sigma_BR = pull_and_format_df(dataDir + "attGuidMsg.omega_BR_B.data", 3)
    sigma_BR = reshape_df_for_plotting(sigma_BR)
    sigma_BR['time'] = sigma_BR['time'] / 1e9  # Convert time from nanoseconds to seconds
    
    p2 = figure(title="Attitude Rate Error", x_axis_label='time [s]', y_axis_label='omega_BR_B',
                width=800, height=400, tools="pan,wheel_zoom,box_zoom,reset,hover,save")
    
    # Set wheel_zoom as the active scroll tool
    p2.toolbar.active_scroll = p2.select(dict(type=WheelZoomTool))[0]

    # Add hover tool
    hover = HoverTool()
    hover.tooltips = [("Run", "@runNum"), ("Time", "@time{0.00}"), ("Value", "@value{0.00}")]
    p2.add_tools(hover)
    
    lines = []
    legend_items = []
    for i, run_num in enumerate(sigma_BR['runNum'].unique()):
        df_run = sigma_BR[sigma_BR['runNum'] == run_num]
        color = color_map[i % len(color_map)]  # Cycle through colors if there are more runs than colors
        source = ColumnDataSource(df_run[df_run['component'] == '0'])
        line = p2.line(x='time', y='value', source=source,
                       legend_label=f'Run {run_num}', line_width=2, color=color)  # Adjust color and line width as needed
        lines.append(line)
        legend_items.append((f'Run {run_num}', [line]))
    
    
    plotList.append(p2)

    return plotList


def run(show_plots):
    """
    **This script is meant to be configured based on the user's needs. It can be configured using the following
    three booleans:**

    First, set ``show_all_data = True`` to get a broad view of the data and find a time window to investigate closer.

    Once the data is characterized, the user can set ``show_extreme_data = True`` to look at specific run cases
    within the window.

    Finally, the user can set ``show_optional_data = True`` to look at any extra data to determine why the extrema
    cases exist.

    :param show_all_data: plot all MC runs for the plots specified in the plotSuite method
    :param show_extreme_data: call plotSuite method for user-defined number of extrema MC runs
    :param optional_plots: plots additional user-defined plots
    """

    if not FOUND_DATESHADER:
        return

    show_all_data = True
    show_extreme_data = False
    optional_plots = False

    plotList = []
    analysis = mcAnalysisBaseClass()
    analysis.dataDir = path + "/scenario_AttFeedbackMC/"

    # save_as_static: save off static .html files of the plots generated into the staticDir directory.
    # The staticDir will be created inside the dataDir folder.
    # (Note: This inhibits dynamic plotting!
    analysis.save_as_static = False
    analysis.staticDir = "/plots/"

    if show_all_data:
        plotList.extend(plotSuite(analysis.dataDir))

    if show_extreme_data:
        analysis.variableName = "attGuidMsg.omega_BR_B"
        analysis.variableDim = 1

        extremaRunNumbers = analysis.getExtremaRunIndices(numExtrema=1, window=[500 * 1E9, 550 * 1E9])

        analysis.extractSubsetOfRuns(runIdx=extremaRunNumbers)
        plotList.extend(plotSuite(analysis.dataDir + "/subset"))

    if optional_plots:
        # nominalRuns = analysis.getNominalRunIndices(50)
        # statPlots = analysis.generateStatPlots()

        shadowFactor = pull_and_format_df(analysis.dataDir + "/eclipse_data_0.shadowFactor.data", 1)
        shadowFactor = shadowFactor.dropna(axis=1)
        shadowFactorPlot = DS_Plot(shadowFactor, title="Optional Plots: Eclipse",
                                               xAxisLabel='time[s]', yAxisLabel='Eclipse Factor',
                                               macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                                               cmap=RdYlBu9,
                                               plotFcn=curve_per_df_component)

        # plotList.extend([statPlots])
        plotList.extend([shadowFactorPlot])

    analysis.renderPlots(plotList)

# The following must be commented out before this script can run.  It is provided here
# to ensure that the sphinx documentation generation process does not run this script
# automatically.
# if __name__ == "__main__":
#     run(False)


# uncomment the following line to run this script.
run(False)