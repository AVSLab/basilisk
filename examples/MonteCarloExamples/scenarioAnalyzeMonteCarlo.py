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
This script demonstrates how to plot Monte Carlo data using Bokeh. This tool efficiently visualizes large amounts of simulation data typically generated in Monte Carlo sensitivity analysis studies. Running this script creates an interactive HTML view of the simulation data, allowing users to dynamically zoom into the data for more detail.

The following plots illustrate what this particular simulation setup will yield:

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

Efficient Handling of Large Datasets
------------------------------------
This implementation uses Bokeh to efficiently handle and visualize large datasets, including those exceeding one gigabyte in size. Here's how it achieves this:

1. Data Loading: The script uses pandas to load data from pickle files, which is an efficient method for handling large datasets.

2. Downsampling: While the full dataset is loaded, not all points are plotted at once. The plot is initially rendered at a lower resolution, and more detail is added as the user zooms in.

3. Efficient Rendering: Bokeh uses HTML5 Canvas for rendering, which is highly efficient for displaying large numbers of data points.

4. Client-Side Interaction: Most of the interactivity (panning, zooming) happens on the client-side in the browser, reducing the need for constant server communication.

5. Data Streaming: The plot is updated dynamically as the user interacts with it, loading only the necessary data for the current view.

This approach allows for smooth interaction with large datasets that would be impractical to plot all at once using traditional plotting libraries.

Configuring a Python Environment For this Script
------------------------------------------------
To run this script, you need to set up a specific Python environment:

1. Use Python 3.8 or higher.
2. Create a dedicated virtual environment and compile Basilisk for this environment.
3. Install the required packages. The necessary dependencies are listed in the `requirements_optional.txt` file in the Basilisk root directory. You can install them using:

   pip install -r requirements_optional.txt

   This will install Bokeh and other optional dependencies needed for this script.

How to Run the Script
---------------------
Follow these steps to run the script:

1. First, run the `scenario_AttFeedbackMC.py` script to generate the necessary data files.

2. Ensure that the `run()` function call at the bottom of this script is uncommented.

3. Run this script from the command line using:

   python scenarioAnalyzeMonteCarlo.py

   This will process the data created by `scenario_AttFeedbackMC.py` and open a browser window showing the interactive plot.

4. Use the dropdown menus to select different variables and components to plot.

5. Enter specific run numbers in the text input field to highlight those runs on the plot.

6. Use the pan, zoom, and reset tools to explore the data in detail.

The script will display information about the loaded data, including the number of runs and total data size, at the top of the plot.

"""

# Import necessary modules
import os  # For file and directory operations
import logging  # For logging messages
import time  # For timing the execution
logging.basicConfig(level=logging.INFO)  # Set up basic logging configuration

# Import the MonteCarloPlotter class from the Basilisk utilities
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter

# Import curdoc from bokeh for adding the plot to the current document
from bokeh.io import curdoc

def run():
    # Construct the path to the data directory
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenario_AttFeedbackMC")

    # Check if the data directory exists and is not empty
    if not os.path.exists(data_dir) or not os.listdir(data_dir):
        print("Error: The data directory is empty or doesn't exist.")
        print("Make sure you've run the scenario_AttFeedbackMC.py script to generate the data files.")
        return

    # Record the start time for performance measurement
    start_time = time.time()

    # Create an instance of MonteCarloPlotter
    plotter = MonteCarloPlotter(data_dir)

    # Load the specified data variables
    plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])

    # Generate the plot layout
    layout = plotter.show_plots()

    # If a layout was successfully created, add it to the current document
    if layout is not None:
        curdoc().add_root(layout)

    # Record the end time and calculate total execution time
    end_time = time.time()
    print(f"Total time taken: {end_time - start_time:.2f} seconds")

    # Provide user feedback about the plot display
    print("Plot should be displayed now. If the page is blank, wait for a few more seconds and try refreshing.")
    print("If the issue persists, check the browser console for any JavaScript errors.")

# Run the Monte Carlo Analysis script that plots the data.
run()