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
This script is a basic demonstration of how to run Monte Carlo simulations. Look at the source code for
further discussion and instructions.

.. note::

    In these Monte Carlo simulations the retained data is stored as the data array with the time
    information added as the first column.  This is the same retained data format as used
    with BSK 1.x.

"""

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

# import general simulation support files
import sys
from Basilisk.utilities.MonteCarlo.Controller import Controller
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import (UniformEulerAngleMRPDispersion, UniformDispersion,
                                                       NormalVectorCartDispersion)

sys.path.append(path+"/../BskSim/scenarios/")
import scenario_AttFeedback

sNavTransName = "sNavTransMsg"
attGuidName = "attGuidMsg"

def run(show_plots):
    """This function is called by the py.test environment."""

    # A MonteCarlo simulation can be created using the `MonteCarlo` module.
    # This module is used to execute monte carlo simulations, and access
    # retained data from previously executed MonteCarlo runs.
    monteCarlo = Controller()
    monteCarlo.setSimulationFunction(scenario_AttFeedback.scenario_AttFeedback)  # Required: function that configures the base scenario
    monteCarlo.setExecutionFunction(scenario_AttFeedback.runScenario)  # Required: function that runs the scenario
    monteCarlo.setExecutionCount(4)  # Required: Number of MCs to run

    monteCarlo.setArchiveDir(path + "/scenario_AttFeedbackMC")  # Optional: If/where to save retained data.
    monteCarlo.setShouldDisperseSeeds(True)  # Optional: Randomize the seed for each module
    monteCarlo.setThreadCount(2)  # Optional: Number of processes to spawn MCs on
    monteCarlo.setVerbose(True)  # Optional: Produce supplemental text output in console describing status
    monteCarlo.setVarCast('float')  # Optional: Downcast the retained numbers to float32 to save on storage space
    monteCarlo.setDispMagnitudeFile(True)  # Optional: Produce a .txt file that shows dispersion in std dev units

    # Statistical dispersions can be applied to initial parameters using the MonteCarlo module
    dispMRPInit = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
    dispOmegaInit = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
    dispMass = 'TaskList[0].TaskModels[0].hub.mHub'
    dispCoMOff = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
    dispInertia = 'hubref.IHubPntBc_B'
    dispList = [dispMRPInit, dispOmegaInit, dispMass, dispCoMOff, dispInertia]

    # Add dispersions with their dispersion type
    monteCarlo.addDispersion(UniformEulerAngleMRPDispersion('TaskList[0].TaskModels[0].hub.sigma_BNInit'))
    monteCarlo.addDispersion(NormalVectorCartDispersion('TaskList[0].TaskModels[0].hub.omega_BN_BInit', 0.0, 0.75 / 3.0 * np.pi / 180))
    monteCarlo.addDispersion(UniformDispersion('TaskList[0].TaskModels[0].hub.mHub', ([750.0 - 0.05*750, 750.0 + 0.05*750])))
    monteCarlo.addDispersion(NormalVectorCartDispersion('TaskList[0].TaskModels[0].hub.r_BcB_B', [0.0, 0.0, 1.0], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))

    # A `RetentionPolicy` is used to define what data from the simulation should be retained. A `RetentionPolicy`
    # is a list of messages and variables to log from each simulation run. It also can have a callback,
    # used for plotting/processing the retained data.
    retentionPolicy = RetentionPolicy()
    samplingTime = int(2E9)
    retentionPolicy.addMessageLog(sNavTransName, ["r_BN_N"])
    retentionPolicy.addMessageLog(attGuidName, ["sigma_BR", "omega_BR_B"])
    retentionPolicy.setDataCallback(displayPlots)
    monteCarlo.addRetentionPolicy(retentionPolicy)

    failures = monteCarlo.executeSimulations()

    if show_plots:
        monteCarlo.executeCallbacks()
        plt.show()

    return

def displayPlots(data, retentionPolicy):
    states = data["messages"][attGuidName + ".sigma_BR"]
    time = states[:, 0]
    plt.figure(1)
    plt.plot(time, states[:,1],
             time, states[:,2],
             time, states[:,3])



if __name__ == "__main__":
    run(True)
