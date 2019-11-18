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
Overview
--------

This script duplicates the ``bskSim`` basic orbit simulation in the scenario
:ref:`scenario_BasicOrbit`.
The difference is that instead of plotting the results after the simulation has stopped in this script a separate
thread is created to update the plots live during the simulation run itself.  For more information on doing live
plotting see help file :ref:`usingLivePlotting`.

The script is found in the folder ``src/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_BasicOrbit_LivePlot.py

# To enable live plotting with a ``bskSim`` Basilisk simulation additional
python packages ``Pipe``, ``Process``, ``Lock`` must be imported.


Next, the ``bskSim`` plotting routines must be called with a unique ID number.  This is done in this example
with::

        plotID = min([num for num in range(1,10) if not plt.fignum_exists(num)])
        BSK_plt.plot_orbit(r_BN_N, plotID)
        plotID = min([num for num in range(1,10) if not plt.fignum_exists(num)])
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN, plotID)

The next step is to setup the ``live_outputs()`` method which details what to plot in a live manner.

The ``setup_live_outputs()`` method returns the required ``dataRequests`` structure as
discussed in the :ref:`usingLivePlotting` documentation page.


"""





# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

from multiprocessing import Pipe, Process, Lock
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.simulation import clock_synch

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')


# Create your own scenario child class
class scenario_BasicOrbitLive(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_BasicOrbitLive, self).__init__()
        self.name = 'scenario_BasicOrbitLive'

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)
        self.initInterfaces()

        self.configure_initial_conditions()
        self.log_outputs()

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Configure FSW mode
        self.modeRequest = 'standby'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
        oe.e = 0.1
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        print('%s: log_outputs' % self.name)
        # Dynamics process outputs
        samplingTime = self.get_DynModel().processTasksTimeStep
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputAttName, samplingTime)
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputTransName, samplingTime)

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        # Dynamics process outputs
        sigma_BN = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", range(3))
        r_BN_N = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
        v_BN_N = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))

        # Plot results
        BSK_plt.clear_all_plots()
        timeLineSet = r_BN_N[:, 0] * macros.NANO2MIN

        plotID = min([num for num in range(1,10) if not plt.fignum_exists(num)])
        BSK_plt.plot_orbit(r_BN_N, plotID)
        plotID = min([num for num in range(1,10) if not plt.fignum_exists(num)])
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN, plotID)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()

        return figureList

    def live_outputs(self, plotComm, rate):
        dataRequests = self.setup_live_outputs()
        lock = Lock()
        while True:
            for request in dataRequests:
                #send request for data
                plotComm.send(request)
                response = plotComm.recv()

                if response == "TERM":
                    plt.close("all")
                    return
                pltArgs = []
                if response["plotFun"] == "plot_orbit":
                    lock.acquire()
                    for resp in response["dataResp"]:
                        pltArgs.append(np.array(resp))
                    pltArgs.append(response["plotID"])
                    getattr(BSK_plt, response["plotFun"])(*pltArgs)
                    lock.release()
                    plt.pause(.01)
                elif response["plotFun"] == "plot_orientation":
                    lock.acquire()
                    pltArgs.append(response["dataResp"][0][:, 0] * macros.NANO2MIN)
                    for resp in response["dataResp"]:
                        pltArgs.append(np.array(resp))
                    pltArgs.extend((response["plotID"], True))
                    getattr(BSK_plt, response["plotFun"])(*pltArgs)
                    lock.release()
                    plt.pause(.01)
            sleep(rate/1000.)

    def setup_live_outputs(self):
        #define plots of interest here
        dataRequests = [{"plotID" : 1,
                        "plotFun" : "plot_orbit",
                        "dataReq" : [self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N"]},
                        {"plotID" : 2,
                        "plotFun" : "plot_orientation",
                        "dataReq" : [self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N",
                                    self.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N",
                                    self.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN"]}]
        return dataRequests


def runScenario(scenario, livePlots=False, showPlots=False):

    if livePlots:
        clockSync = clock_synch.ClockSynch()
        clockSync.accelFactor = 50.0
        scenario.AddModelToTask(scenario.DynModels.taskName, clockSync)

    # Initialize simulation
    scenario.InitializeSimulationAndDiscover()

    # Configure run time
    simulationTime = macros.min2nano(10.)
    scenario.ConfigureStopTime(simulationTime)

    # Run simulation
    figureList = {}
    if livePlots:
        # plotting refresh rate in ms
        refreshRate = 1000
        plotComm, simComm = Pipe()
        plotArgs = [showPlots]
        simProc = Process(target=scenario.ExecuteSimulation,
                          args=(showPlots, livePlots, simComm, scenario.pull_outputs, plotArgs))
        plotProc = Process(target=scenario.live_outputs, args=(plotComm, refreshRate))
        # Execute simulation and live plotting
        simProc.start(), plotProc.start()
        simProc.join(), plotProc.join()
    else:
        scenario.ExecuteSimulation()

def run(showPlots, livePlots=False):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbitLive()
    runScenario(TheScenario, livePlots=livePlots, showPlots=showPlots)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True, True)
