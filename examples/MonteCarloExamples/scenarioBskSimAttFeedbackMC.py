#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
Monte Carlo Simulation for Attitude Feedback Scenario
=====================================================

This script demonstrates how to set up and run Monte Carlo simulations using the Basilisk framework.
It uses the attitude feedback scenario as a base and applies various dispersions to create multiple simulation runs.

Key Features:
-------------
1. Monte Carlo Controller Setup: Uses the `Controller` class from Basilisk's Monte Carlo utilities.
2. Dispersion Application: Applies statistical dispersions to initial parameters.
3. Retention Policy: Defines which data should be retained from each simulation run.
4. Data Analysis: Includes a callback function for plotting retained data.

How to Use:
-----------
1. Ensure you have Basilisk installed with all required dependencies.
2. Run this script directly to execute the Monte Carlo simulations::

    python scenarioBskSimAttFeedbackMC.py

3. The script will run 4 Monte Carlo simulations by default.
4. Results will be saved in the 'scenarioBskSimAttFeedbackMC' directory within the script's location.

Monte Carlo Configuration:
--------------------------
- Simulation Function: Uses ``scenario_AttFeedback.scenario_AttFeedback`` to set up the base scenario.
- Execution Function: Uses ``scenario_AttFeedback.runScenario`` to run each simulation.
- Execution Count: Set to 4 simulations.
- Archive Directory: Results are saved in ``scenarioBskSimAttFeedbackMC``.
- Seed Dispersion: Enabled to randomize the seed for each module.
- Variable Casting: Downcasts retained numbers to float32 to save storage space.
- Dispersion Magnitude File: Produces a ``.txt`` file showing dispersion in standard deviation units.

Applied Dispersions:
--------------------
1. MRP Initial Condition: Uniform Euler Angle MRP Dispersion
2. Angular Velocity Initial Condition: Normal Vector Cartesian Dispersion
3. Hub Mass: Uniform Dispersion
4. Center of Mass Offset: Normal Vector Cartesian Dispersion
5. Hub Inertia: (Dispersion defined but not explicitly added in the provided code)

Retention Policy:
-----------------
- Logs ``r_BN_N`` from ``sNavTransMsg``
- Logs ``sigma_BR`` and ``omega_BR_B`` from ``attGuidMsg``
- Uses a callback function ``displayPlots`` for data visualization

Output:
-------
- The script generates plots of the retained data if run directly.
- Plots show the evolution of attitude error (sigma_BR) over time for all simulation runs.

Note:
-----
This script serves as a template for setting up Monte Carlo simulations in Basilisk.
Users can modify the dispersions, retention policy, and analysis methods to suit their specific simulation needs.
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

    monteCarlo.setArchiveDir(path + "/scenarioBskSimAttFeedbackMC")  # Optional: If/where to save retained data.
    monteCarlo.setShouldDisperseSeeds(True)  # Optional: Randomize the seed for each module
    # monteCarlo.setThreadCount(2)  # Optional: Number of processes to spawn MCs on, automatically sizes for personal computer.
    # monteCarlo.setVerbose(True)  # Optional: Produce supplemental text output in console describing status
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
