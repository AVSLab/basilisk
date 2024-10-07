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

.. include:: docs/attGuidMsg.sigma_BR_x.rst

.. include:: docs/attGuidMsg.sigma_BR_y.rst

.. include:: docs/attGuidMsg.sigma_BR_z.rst

.. include:: docs/attGuidMsg.omega_BR_B_x.rst

.. include:: docs/attGuidMsg.omega_BR_B_y.rst

.. include:: docs/attGuidMsg.omega_BR_B_z.rst

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
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter
from bokeh.io import output_file, save

logging.basicConfig(level=logging.INFO)  # Set up basic logging configuration

def run():
    # Construct the path to the data directory
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenario_AttFeedbackMC")
    doc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs")

    # Check if the data directory exists and is not empty
    if not os.path.exists(data_dir) or not os.listdir(data_dir):
        print("Error: The data directory is empty or doesn't exist.")
        print("Make sure you've run the scenario_AttFeedbackMC.py script to generate the data files.")
        return

    # Record the start time for performance measurement
    start_time = time.time()

    # Create an instance of MonteCarloPlotter
    plotter = MonteCarloPlotter(data_dir, save_plots=True, doc_dir=doc_dir)

    # Load the specified data variables
    plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])

    # Generate and save all initial plots
    plotter.save_all_initial_plots()

    # Record the end time and calculate total execution time
    end_time = time.time()
    print(f"Total time taken: {end_time - start_time:.2f} seconds")

    # Provide user feedback about the plot display
    print("Plots have been generated and saved.")
    print("Check the 'saved_plots' directory for the generated HTML files.")
    print("Check the 'docs' directory for the generated RST files.")

    return None

# Only run this if the script is executed directly, not when imported
if __name__ == "__main__":
    run()