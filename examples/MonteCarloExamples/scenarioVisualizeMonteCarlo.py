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

.. include:: ../MonteCarloPlots_attGuidMsg.sigma_BR_1.rst

.. include:: ../MonteCarloPlots_attGuidMsg.sigma_BR_2.rst

.. include:: ../MonteCarloPlots_attGuidMsg.sigma_BR_3.rst

.. include:: ../MonteCarloPlots_attGuidMsg.omega_BR_B_1.rst

.. include:: ../MonteCarloPlots_attGuidMsg.omega_BR_B_2.rst

.. include:: ../MonteCarloPlots_attGuidMsg.omega_BR_B_3.rst

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

1. First, run the `scenarioBskSimAttFeedbackMC.py` script to generate the necessary data files.

2. Ensure that the `run()` function call at the bottom of this script is uncommented.

3. Run this script from the command line using:

    To generate a static HTML file:

    .. code-block:: python

        from scenarioVisualizeMonteCarlo import run
        run(use_bokeh_server=False)

    To start an interactive Bokeh server:

    .. code-block:: python

        from scenarioVisualizeMonteCarlo import run
        run(use_bokeh_server=True)

    Alternatively, you can run this script directly from the command line:

    .. code-block:: bash

        # For static HTML generation
        python scenarioVisualizeMonteCarlo.py

        # For interactive Bokeh server
        python scenarioVisualizeMonteCarlo.py --bokeh-server

   This will process the data created by `scenarioBskSimAttFeedbackMC.py` and open a browser window showing the interactive plot.

4. Use the dropdown menus to select different variables and components to plot.

5. Enter specific run numbers in the text input field to highlight those runs on the plot.

6. Use the pan, zoom, and reset tools to explore the data in detail.

The script will display information about the loaded data, including the number of runs and total data size, at the top of the plot.

"""

import os
import logging
import importlib
import webbrowser
from threading import Timer
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

bokeh_spec = importlib.util.find_spec("bokeh")
bokeh_available = bokeh_spec is not None

if bokeh_available:
    from bokeh.io import output_file, show, curdoc
    from bokeh.layouts import column
    from bokeh.models import Div
    from bokeh.server.server import Server
    from bokeh.application import Application
    from bokeh.application.handlers.function import FunctionHandler
    from tornado.ioloop import IOLoop

def create_document(doc):
    logger.info("Starting the create_document function")

    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarioBskSimAttFeedbackMC")
    doc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs")
    plot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_plots")

    logger.info(f"Data directory: {data_dir}")
    logger.info(f"Doc directory: {doc_dir}")
    logger.info(f"Plot directory: {plot_dir}")

    try:
        logger.info("Creating MonteCarloPlotter instance")
        plotter = MonteCarloPlotter(data_dir, save_plots=True, doc_dir=doc_dir)

        logger.info("Loading data")
        plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])

        logger.info("Creating plot layout")
        layout = plotter.show_plots()

        doc.add_root(layout)
        doc.title = "BSK Monte Carlo Visualization"

    except Exception as e:
        error_message = f"An error occurred: {str(e)}"
        logger.error(error_message)
        doc.add_root(Div(text=error_message))

def open_browser():
    webbrowser.open("http://localhost:5006")

def run(use_bokeh_server=False):
    if not bokeh_available:
        logger.error("Bokeh is not available. This script requires Bokeh to run.")
        return

    if use_bokeh_server:
        def bk_worker():
            app = Application(FunctionHandler(create_document))
            server = Server({'/': app}, io_loop=IOLoop(), allow_websocket_origin=["*"])
            server.start()
            Timer(1, open_browser).start()  # Open browser after 1 second
            server.io_loop.start()

        logger.info("Starting Bokeh server. A browser window should open automatically.")
        bk_worker()
    else:
        # Create a document for the static case
        doc = curdoc()
        create_document(doc)

        # Set up the output file
        output_file("monte_carlo_plots.html", title="BSK Monte Carlo Visualization")

        # Show the layout
        show(doc.roots[0])

if __name__ == "__main__":
    import sys
    use_bokeh_server = "--bokeh-server" in sys.argv
    run(use_bokeh_server)