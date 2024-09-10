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

import os
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter
from bokeh.layouts import column
from bokeh.io import curdoc

def plotSuite(dataDir, components):
    plotter = MonteCarloPlotter(dataDir)
    plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])
    plotter.generate_plots(components)
    return plotter.plots

def run():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenario_AttFeedbackMC")

    if not os.path.exists(data_dir) or not os.listdir(data_dir):
        print("Error: The data directory is empty or doesn't exist.")
        print("Make sure you've run the scenario_AttFeedbackMC.py script to generate the data files.")
        return

    show_all_data = True
    show_extreme_data = False
    components_to_plot = [0, 1, 2]

    plotter = MonteCarloPlotter(data_dir)
    plotter.save_as_static = False
    plotter.staticDir = "/plots/"

    if show_all_data:
        plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])
        downsampled_plots = plotter.get_downsampled_plots()
        
        if downsampled_plots:
            all_plots = []
            for title, df in downsampled_plots.items():
                y_label = title.split('.')[-1]
                try:
                    plots = plotter.create_datashader_plot(df, title, y_label)
                    all_plots.append(plots)
                except Exception as e:
                    print(f"Error creating plot for {title}: {str(e)}")
            
            if all_plots:
                layout = column(*all_plots, sizing_mode='stretch_both')
                curdoc().add_root(layout)
            else:
                print("No plots were created successfully.")
        else:
            print("No downsampled plots were created.")

    if show_extreme_data:
        plotter.variableName = "attGuidMsg.omega_BR_B"
        plotter.variableDim = 1

        extremaRunNumbers = plotter.getExtremaRunIndices(numExtrema=1, window=[500 * 1E9, 550 * 1E9])

        plotter.extractSubsetOfRuns(runIdx=extremaRunNumbers)
        plotSuite(plotter.dataDir + "/subset", components_to_plot)

# The following must be commented out before this script can run.  It is provided here
# to ensure that the sphinx documentation generation process does not
# run this script automatically.
# if __name__ == "__main__":
#     run(False)

# uncomment the following line to run this script.
run()