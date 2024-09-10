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
from bokeh.models import ColorBar, BasicTicker, LinearColorMapper, ColumnDataSource
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

from bokeh.models import CustomJS, ColumnDataSource, HoverTool, Legend
from bokeh.palettes import Category10  # Import a color palette

import numpy as np
from bokeh.models import HoverTool, Legend, ColorBar, BasicTicker, LinearColorMapper
from bokeh.palettes import Viridis256
from bokeh.plotting import figure
from bokeh.layouts import column

def create_datashader_plot(df, title, y_label, max_points=1000):
    print(f"Creating datashader plot for {title}")
    print(f"Original DataFrame shape: {df.shape}")
    print(f"Original DataFrame index: {df.index}")
    print(f"Original DataFrame columns: {df.columns}")

    # Create a list to store individual plots
    plots = []

    # Map components to x, y, z
    component_map = {0: 'x', 1: 'y', 2: 'z'}

    # Get the number of runs
    num_runs = len(set(col[0] for col in df.columns))
    print(f"Number of runs detected: {num_runs}")

    # Create a color mapper
    color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=num_runs-1)

    # Convert time to seconds for plotting
    time_seconds = (df.index - df.index[0]).total_seconds()
    print(f"Time range: {time_seconds.min()} to {time_seconds.max()} seconds")

    # Iterate through each component
    for component in range(3):  # Assuming 3 components (0, 1, 2)
        component_label = component_map[component]
        print(f"\nProcessing {component_label.upper()} Component")
        
        # Create a figure for this component
        p = figure(title=f"{title} - {component_label.upper()} Component", x_axis_label='Time (s)', y_axis_label=f"{y_label} ({component_label})",
                   width=800, height=400, tools='pan,wheel_zoom,box_zoom,reset')

        # Plot each run for this component
        for run in range(num_runs):
            col_name = (run, component)
            if col_name not in df.columns:
                print(f"Warning: Column {col_name} not found in DataFrame. Available columns: {df.columns}")
                continue
            
            # Print some debug information about the data
            print(f"  Run {run}:")
            print(f"    Data range: {df[col_name].min()} to {df[col_name].max()}")
            print(f"    Number of unique values: {df[col_name].nunique()}")
            
            source = ColumnDataSource(data=dict(x=time_seconds, y=df[col_name], run_num=[run]*len(time_seconds)))
            p.line('x', 'y', source=source, color=color_mapper.palette[int(run * (len(color_mapper.palette) - 1) / (num_runs - 1))], 
                   line_alpha=0.6, legend_label=f"Run {run}")

        # Add HoverTool
        hover = HoverTool(
            tooltips=[
                ('Time', '$x{0.000} s'),
                (f'{y_label} ({component_label})', '$y{0.0000}'),
                ('Run', '@run_num')
            ],
            mode='mouse'  # Use mouse mode to show tooltip for the closest line
        )
        p.add_tools(hover)

        # Add ColorBar
        color_bar = ColorBar(color_mapper=color_mapper, ticker=BasicTicker(desired_num_ticks=10),
                             label_standoff=12, border_line_color=None, location=(0, 0))
        p.add_layout(color_bar, 'right')

        # Adjust the x-range and y-range
        p.x_range.start = time_seconds.min()
        p.x_range.end = time_seconds.max()
        y_min = min(df[[(run, component) for run in range(num_runs)]].min().min() for component in range(3))
        y_max = max(df[[(run, component) for run in range(num_runs)]].max().max() for component in range(3))
        p.y_range.start = y_min
        p.y_range.end = y_max

        # Configure legend
        p.legend.click_policy = "hide"
        p.legend.location = "top_left"

        print(f"  Plot ranges - X: [{p.x_range.start}, {p.x_range.end}], Y: [{p.y_range.start}, {p.y_range.end}]")

        plots.append(p)

    # Create a layout with all plots
    layout = column(*plots)

    print("Datashader plot created successfully")
    return layout

from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter

class MonteCarloPlotterWrapper(MonteCarloPlotter):
    def __init__(self, dataDir):
        super().__init__(dataDir)
        self.custom_debug_info = []

    def load_data(self, variables):
        self.custom_debug_info.append(f"Loading data for variables: {variables}")
        self.custom_debug_info.append(f"All .data files in the directory:")
        data_files = [f for f in os.listdir(self.dataDir) if f.endswith('.data')]
        self.custom_debug_info.extend(data_files)
        
        for variable in variables:
            file_path = os.path.join(self.dataDir, f"{variable}.data")
            self.custom_debug_info.append(f"Looking for file: {file_path}")
            if os.path.exists(file_path):
                self.custom_debug_info.append(f"File found: {file_path}")
                df = pd.read_pickle(file_path)
                self.custom_debug_info.append(f"DataFrame shape: {df.shape}")
                self.custom_debug_info.append(f"DataFrame columns: {df.columns}")
                self.custom_debug_info.append(f"DataFrame index: {df.index}")
                self.custom_debug_info.append(f"First few rows:\n{df.head().to_string()}")
                self.data[variable] = df
            else:
                self.custom_debug_info.append(f"File not found: {file_path}")
        
        self.custom_debug_info.append(f"Data loaded. Number of variables: {len(self.data)}")
        for key, df in self.data.items():
            self.custom_debug_info.append(f"Variable: {key}, DataFrame shape: {df.shape}")

    def get_downsampled_plots(self):
        self.custom_debug_info.append("Starting get_downsampled_plots method")
        downsampled_plots = {}
        for key, df in self.data.items():
            self.custom_debug_info.append(f"\nProcessing {key}")
            self.custom_debug_info.append(f"DataFrame shape: {df.shape}")
            self.custom_debug_info.append(f"DataFrame columns: {df.columns}")
            self.custom_debug_info.append(f"DataFrame index: {df.index}")
            
            # Assuming the index is the time
            if not pd.api.types.is_datetime64_any_dtype(df.index):
                df.index = pd.to_datetime(df.index, unit='ns')
            
            self.custom_debug_info.append("Time conversion successful")
            self.custom_debug_info.append(f"Index after conversion: {df.index}")
            
            # Perform downsampling
            downsampled_df = df.groupby(df.index.floor('1s')).mean()
            self.custom_debug_info.append(f"Downsampling successful")
            self.custom_debug_info.append(f"Downsampled DataFrame shape: {downsampled_df.shape}")
            self.custom_debug_info.append(f"Downsampled DataFrame columns: {downsampled_df.columns}")
            self.custom_debug_info.append(f"First few rows of downsampled data:\n{downsampled_df.head().to_string()}")
            
            if not downsampled_df.empty:
                downsampled_plots[key] = downsampled_df
            else:
                self.custom_debug_info.append(f"Warning: Downsampled DataFrame is empty for {key}")
        
        self.custom_debug_info.append(f"\nNumber of downsampled plots: {len(downsampled_plots)}")
        return downsampled_plots

    def get_debug_info(self):
        return self.custom_debug_info

def get_plotter_debug_info(plotter):
    debug_info = []
    try:
        debug_info.append(f"Data directory: {plotter.dataDir}")
    except AttributeError:
        debug_info.append("Unable to access dataDir")
    
    try:
        debug_info.append(f"Number of loaded variables: {len(plotter.data)}")
        for key, df in plotter.data.items():
            debug_info.append(f"\nVariable: {key}")
            debug_info.append(f"DataFrame shape: {df.shape}")
            debug_info.append(f"DataFrame columns: {df.columns}")
            debug_info.append(f"First few rows of data:\n{df.head().to_string()}")
            debug_info.append(f"Data types of columns:\n{df.dtypes}")
            
            if 'time' in df.columns:
                debug_info.append(f"Time column data type: {df['time'].dtype}")
                debug_info.append(f"Time column first few values: {df['time'].head().tolist()}")
    except AttributeError:
        debug_info.append("Unable to access data attribute")
    except Exception as e:
        debug_info.append(f"Error while getting debug info: {str(e)}")
    
    return debug_info

def run(show_plots):
    print("Starting run function")
    print(f"Current working directory: {os.getcwd()}")

    # Create the MonteCarloPlotter instance
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenario_AttFeedbackMC")
    print(f"Data directory path: {data_dir}")

    if not os.path.exists(data_dir) or not os.listdir(data_dir):
        print("Error: The data directory is empty or doesn't exist.")
        print("Make sure you've run the scenario_AttFeedbackMC.py script to generate the data files.")
        return

    if os.path.exists(data_dir):
        print(f"Contents of {data_dir}:")
        for file in os.listdir(data_dir):
            print(f"  {file}")
    else:
        print(f"Error: Directory {data_dir} does not exist")

    show_all_data = True
    show_extreme_data = False
    optional_plots = False

    # Specify which components to plot (0, 1, 2 correspond to x, y, z)
    components_to_plot = [0, 1, 2]

    plotList = []

    # save_as_static: save off static .html files of the plots generated into the staticDir directory.
    # The staticDir will be created inside the dataDir folder.
    # (Note: This inhibits dynamic plotting!)
    analysis = MonteCarloPlotter(data_dir)
    analysis.save_as_static = False
    analysis.staticDir = "/plots/"

    if show_all_data:
        try:
            print("Initializing MonteCarloPlotterWrapper")
            plotter = MonteCarloPlotterWrapper(data_dir)
            
            print("Loading data")
            plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])
             
            # Get and print debug info
            debug_info = plotter.get_debug_info()
            print("\n".join(debug_info))
            
            print("Getting downsampled plots")
            downsampled_plots = plotter.get_downsampled_plots()
            
            # Get and print updated debug info
            debug_info = plotter.get_debug_info()
            print("\n".join(debug_info))
            
            print(f"Number of downsampled plots: {len(downsampled_plots)}")
            
            if downsampled_plots:
                all_plots = []
                for title, df in downsampled_plots.items():
                    y_label = title.split('.')[-1]  # Use the last part of the variable name as y_label
                    try:
                        plots = create_datashader_plot(df, title, y_label)
                        all_plots.append(plots)
                    except Exception as e:
                        print(f"Error creating plot for {title}: {str(e)}")
                        print(f"DataFrame info for {title}:")
                        print(df.info())
                        print(f"DataFrame head for {title}:")
                        print(df.head())
                
                if all_plots:
                    # Use column layout with sizing_mode='stretch_both' for responsive layout
                    layout = column(*all_plots, sizing_mode='stretch_both')
                    curdoc().add_root(layout)
                else:
                    print("No plots were created successfully.")
            else:
                print("No downsampled plots were created.")
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}")
            import traceback
            traceback.print_exc()

    if show_extreme_data:
        analysis.variableName = "attGuidMsg.omega_BR_B"
        analysis.variableDim = 1

        extremaRunNumbers = analysis.getExtremaRunIndices(numExtrema=1, window=[500 * 1E9, 550 * 1E9])

        analysis.extractSubsetOfRuns(runIdx=extremaRunNumbers)
        plotList.extend(plotSuite(analysis.dataDir + "/subset", components_to_plot))

    if optional_plots:
        # This section needs to be updated if you want to include optional plots
        pass

    print("Run function completed")

# The following must be commented out before this script can run.  It is provided here
# to ensure that the sphinx documentation generation process does not
# run this script automatically.
# if __name__ == "__main__":
#     run(False)

# uncomment the following line to run this script.
run(False)