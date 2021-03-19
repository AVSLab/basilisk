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

def plotSuite(dataDir):
    """
    This is the function to populate with all of the plots to be generated using datashaders and bokeh.
    Each variable requires a call to ``pull_and_format_df()`` to ensure the dataframe will be compatible with
    the developed datashader utilities.

    Args:
        dataDir: (str) directory containing all of the dataframes created from the Monte Carlo run

    Returns: List of DS_Plots

    """
    plotList = []
    sigma_BR = pull_and_format_df(dataDir + "attGuidMsg.sigma_BR.data", 3)
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Error",
                        xAxisLabel='time [s]', yAxisLabel='Sigma_BR',
                        macro_x=macros.NANO2SEC,
                        labels = ['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_component)
    plotList.append(sigmaPlot)

    sigma_BR = pull_and_format_df(dataDir + "attGuidMsg.omega_BR_B.data", 3)
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Rate Error",
                        xAxisLabel='time [s]', yAxisLabel='omega_BR_B',
                        macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                        labels = ['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_component)
    plotList.append(sigmaPlot)
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
if __name__ == "__main__":
    run(False)


# uncomment the following line to run this script.
# run(False)