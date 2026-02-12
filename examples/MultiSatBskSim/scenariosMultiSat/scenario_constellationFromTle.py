#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
Overview
--------

This script demonstrates how to use the Basilisk v2.1 multi-threading to simulate a constellation
of spacecraft initialized from TLE data using multiple threads.  The simulation scenario setup is
very similar to :ref:`scenario_BasicOrbitMultiSat_MT`, but each spacecraft's initial orbit elements
are read from the selected TLE catalog instead of being generated analytically.  The following Vizard
screen capture illustrates the family of orbits being simulated.

Inside the ``run()`` command the number of threads is specified using::

    TheScenario.TotalSim.resetThreads(numThreads)

With this basic setup it is assumed that each BSK process can run independently in a thread.  Note that this script
does not use any spacecraft flight software which is running in a separate process.  The Basilisk v2.1
multi-threading only functions for simulations where each Basilisk process can be evaluated independently.

.. warning::

    The Basilisk v2.1 multi-threading does not have a thread-safe messaging system.  This will be
    added in a later release.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True, tleData = tleHandling.satTle2elem(path/to/tleFile), numThreads = 4

.. image:: /_images/Scenarios/scenario_constellationFromTle_spacecraft_orbits.svg
   :align: center


"""

# Get current file path
import inspect
import os
import pathlib
import sys

from Basilisk.architecture import messaging
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport, tleHandling

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../modelsMultiSat')
sys.path.append(path + '/../plottingMultiSat')
from BSK_MultiSatMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_MultiSatDynamics

# Import plotting files for your scenario
import BSK_MultiSatPlotting as plt

DATA_DIR = pathlib.Path(__file__).resolve().parent.parent.joinpath('tleData')
# Create your own scenario child class
class scenario_BasicOrbitFormationFlying(BSKSim, BSKScenario):
    def __init__(self, tleData):
        super(scenario_BasicOrbitFormationFlying, self).__init__(len(tleData), fswRate=10, dynRate=10, envRate=10)
        self.name = 'scenario_constellationFromTle'

        self.set_EnvModel(BSK_EnvironmentEarth)

        # Here we are setting the list of spacecraft dynamics to be a homogeneous formation.
        # To use a heterogeneous spacecraft formation, this list can contain different
        # BSKDynamicModels classes.
        self.set_DynModel([BSK_MultiSatDynamics] * self.numberSpacecraft)

        # Declare empty class variables
        self.samplingTime = []
        self.snTransLog = []
        self.snAttLog = []
        self.rwSpeedLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]

        # Declare empty containers for orbital elements
        self.oe = []

        self.configure_initial_conditions(tleData)
        self.log_outputs()

        if vizSupport.vizFound:
            # If this scenario is to interface with the BSK Viz, keep the saveFile line enabled
            DynModelsList = []
            rwStateEffectorList = []
            for i in range(self.numberSpacecraft):
                DynModelsList.append(self.DynModels[i].scObject)
                rwStateEffectorList.append(self.DynModels[i].rwStateEffector)

            batteryPanel = vizSupport.vizInterface.GenericStorage()
            batteryPanel.label = "Battery"
            batteryPanel.units = "Ws"
            batteryPanel.color = vizSupport.vizInterface.IntVector(
                vizSupport.toRGBA255("red") + vizSupport.toRGBA255("green"))
            batteryPanel.thresholds = vizSupport.vizInterface.IntVector([20])
            batteryInMsg = messaging.PowerStorageStatusMsgReader()
            batteryInMsg.subscribeTo(self.DynModels[0].powerMonitor.batPowerOutMsg)
            batteryPanel.batteryStateInMsg = batteryInMsg

            tankPanel = vizSupport.vizInterface.GenericStorage()
            tankPanel.label = "Tank"
            tankPanel.units = "kg"
            tankPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
            tankInMsg = messaging.FuelTankMsgReader()
            tankInMsg.subscribeTo(self.DynModels[0].fuelTankStateEffector.fuelTankOutMsg)
            tankPanel.fuelTankStateInMsg = tankInMsg

            # Add this line to maintain Python references
            self.vizPanels = [batteryPanel, tankPanel]

            storageList = [None] * self.numberSpacecraft
            storageList[0] = [batteryPanel, tankPanel]

            viz = vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, DynModelsList
                                                      # , saveFile=__file__
                                                      , rwEffectorList=rwStateEffectorList
                                                      , genericStorageList=storageList
                                                      )
            vizSupport.setInstrumentGuiSetting(viz, showGenericStoragePanel=True)

    def configure_initial_conditions(self, tleData):
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        for i in range(self.numberSpacecraft):
            self.oe.append(orbitalMotion.ClassicElements())
            self.oe[i].a = tleData[i].oe.a
            self.oe[i].e = tleData[i].oe.e
            self.oe[i].i = tleData[i].oe.i
            self.oe[i].Omega = tleData[i].oe.Omega
            self.oe[i].omega = tleData[i].oe.omega
            self.oe[i].f = tleData[i].oe.f
            rN, vN = orbitalMotion.elem2rv(EnvModel.mu, self.oe[i])
            orbitalMotion.rv2elem(EnvModel.mu, rN, vN)
            DynModels[i].scObject.hub.r_CN_NInit = rN  # [m] - r_CN_N
            DynModels[i].scObject.hub.v_CN_NInit = vN  # [m/s] - v_CN_N
            DynModels[i].scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
            DynModels[i].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # [rad/s] - omega_BN_B

    def log_outputs(self):
        # Process outputs
        DynModels = self.get_DynModel()

        # Set the sampling time
        self.samplingTime = macros.sec2nano(1)

        # Loop through every spacecraft
        for spacecraft in range(self.numberSpacecraft):

            # Log the navigation messages
            self.snTransLog.append(DynModels[spacecraft].simpleNavObject.transOutMsg.recorder(self.samplingTime))
            self.snAttLog.append(DynModels[spacecraft].simpleNavObject.attOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snTransLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snAttLog[spacecraft])

            # Log the RW wheel speed information
            self.rwSpeedLog.append(DynModels[spacecraft].rwStateEffector.rwSpeedOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.rwSpeedLog[spacecraft])

            # Log additional RW information (power, etc)
            for item in range(DynModels[spacecraft].numRW):
                self.rwLogs[spacecraft].append(
                    DynModels[spacecraft].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwLogs[spacecraft][item])

    def pull_outputs(self, show_plots):
        # Retrieve the logged data
        r_BN_N = []
        for i in range(self.numberSpacecraft):
            r_BN_N.append(self.snTransLog[i].r_BN_N)

        # Plot results
        plt.clear_all_plots()

        plt.plot_orbits(r_BN_N, self.numberSpacecraft, 1)

        figureList = {}
        if show_plots:
            plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["spacecraft_orbits"]
            figureList = plt.save_all_plots(fileName, figureNames)

        # Close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()

        return figureList

def runScenario(scenario):
    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure run time and execute simulation
    simulationTime = macros.hour2nano(2)
    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()

def run(show_plots, tleData, numThreads):
    """
    The scenario can be run with the following setup parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        tleData (list): List of TLE-derived orbital element objects for each satellite
        numThreads (int): Number of threads
    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbitFormationFlying(tleData)

    # This call allocates out the requested number of threads into the simulation
    # Note that unless the user does something further, processes will be assigned
    # In a round-robin fashion to the allocated threads.
    # To specify which processes execute on a given thread, use addProcessToThread in
    # The SimModel (TotalSim in SimulationBaseClass).
    TheScenario.TotalSim.resetThreads(numThreads)
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(show_plots)

    return figureList

if __name__ == "__main__":
    # Select a TLE file
    tleFile = "oneWeb25.tle" # Options are "hypso1.tle, ISS_ZARYA.tle", "oneWeb25.tle", "spacestations.2le"
    tleDataClass = tleHandling.satTle2elem(os.path.join(DATA_DIR, tleFile))

    run(show_plots=True,
        tleData=tleDataClass,
        numThreads=2
        )
