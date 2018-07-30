''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

## \defgroup Tutorials_6_3
## @{
# Demonstrates how to use the hill pointing model within the BSK_Sim architecture.
#
# BSK Simulation: Attitude Guidance {#scenario_AttGuidance}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting earth, using the MRP_Steering module with a rate sub-servo system
# to control the attitude all within the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttGuidance.py
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_AttGuidance.svg "Illustration")
# Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
# modules. The initial setup for the simulation closely models that of scenario_FeedbackRW.py.
#
# To begin, one must first create a class that will
# inherent from the masterSim class and provide a name to the sim.
# This is accomplished through:
# ~~~~~~~~~~~~~{.py}
#   class scenario_AttGuidance(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_AttGuidance, self).__init__(masterSim)
#          self.name = 'scenario_AttGuidance'
# ~~~~~~~~~~~~~
#
# Following the inheritance, there are three functions within the scenario class that need to be configured by the user:
# configure_initial_conditions(), log_outputs(), and pull_outputs().
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "hillPoint"
# ~~~~~~~~~~~~~
# which triggers the Hill Point event within the BSK_FSW.py script.
# ~~~~~~~~~~~~~
#
# The initial conditions for the scenario are the same as found within scenario_FeedbackRW.py. Within BSK_Scenario.py
# log_outputs(), the user must log the relevant messages to observe how the spacecraft attitude changed throughout
# its orbit:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         samplingTime = self.masterSim.DynModels.processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputAttName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputTransName, samplingTime)
#
#         # FSW process outputs
#         samplingTime = self.masterSim.FSWModels.processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.hillPointData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
# The data is then pulled using:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputAttName + ".sigma_BN", range(3))
#         r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".v_BN_N", range(3))
#
#         # FSW process outputs
#         sigma_RN = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".sigma_RN", range(3))
#         omega_RN_N = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".omega_RN_N", range(3))
#         sigma_BR = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
#         omega_BR_B = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         Lr = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))
#
# ~~~~~~~~~~~~~
# and then plot the results using:
# ~~~~~~~~~~~~~{.py}
#         # Plot results
#         timeLineSet = sigma_BR[:, 0] * macros.NANO2MIN
#         BSK_plt.plot_attitude_error(timeLineSet, sigma_BR)
#         BSK_plt.plot_control_torque(timeLineSet, Lr)
#         BSK_plt.plot_rate_error(timeLineSet, omega_BR_B)
#         BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)
#         BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)
# ~~~~~~~~~~~~~
#
#
#
#
#
#
#
#
#
#
# Custom Dynamics Configurations Instructions
# -----
# The modules required for this scenario are identical to those used in scenario_AttSteering.py.
#
#
#
#
#
#
#
#
#
# Custom FSW Configurations Instructions
# -----
# All modules required to configure the "hillPoint" FSW mode have already been included within BSK_FSW.py in the
# previous scenarios (`hillPointConfig()`, `mrpFeedbackRWConfig()`, `attTrackingErrorConfig(),`rwMotorTorqueConfig()`):
#
# These modules provide the initial setup for an attitude guidance system that makes use of an hill pointing model, a module
# that tracks the error of the spacecraft's MRP parameters against the vector pointing towards the central, planetary
# body, and uses a module that takes that information to provide a torque to correct for the error.
#
# Within `InitAllFSWObjects()` no new setter functions are required beyond what was provided in the past two scenarios.
#
# The only additions required in BSK_FSW.py are to create a new task specific for hill pointing:
# ~~~~~~~~~~~~~{.py}
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
# ~~~~~~~~~~~~~
# and then to add the tracking model to the hill point task through:
# The modules need to be attached to the the tasks through:
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
#         SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
# ~~~~~~~~~~~~~
# Finally, the `hillPoint` mode needs to be defined by enabling the two needed tasks:
# ~~~~~~~~~~~~~{.py}
#              SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'hillPoint'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('hillPointTask')",
#                                 "self.enableTask('mrpFeedbackTask')"])
#
# ~~~~~~~~~~~~~
## @}

# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')
import scenarioAttitudeGuidance as scene_plt


# Create your own scenario child class
class scenario_HillPointing(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_HillPointing, self).__init__(masterSim)
        self.name = 'scenario_AttGuidance'

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'hillPoint'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.1
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    def log_outputs(self):
        print '%s: log_outputs' % self.name
        # Dynamics process outputs
        samplingTime = self.masterSim.DynModels.processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputAttName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputTransName, samplingTime)

        # FSW process outputs
        samplingTime = self.masterSim.FSWModels.processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.hillPointData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName, samplingTime)

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        # Dynamics process outputs
        sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputAttName + ".sigma_BN", range(3))
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".r_BN_N", range(3))
        v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".v_BN_N", range(3))

        # FSW process outputs
        sigma_RN = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".sigma_RN", range(3))
        omega_RN_N = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".omega_RN_N", range(3))
        sigma_BR = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        Lr = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))

        # Plot results
        timeLineSet = sigma_BR[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeLineSet, sigma_BR)
        BSK_plt.plot_control_torque(timeLineSet, Lr)
        BSK_plt.plot_rate_error(timeLineSet, omega_BR_B)
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)
        BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "orientation", "attitudeGuidance"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    # Instantiate base simulation
    TheBSKSim = BSKSim()

    # Configure a scenario in the base simulation
    TheScenario = scenario_HillPointing(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    TheBSKSim.ConfigureStopTime(simulationTime)

    TheBSKSim.ExecuteSimulation()


    # Pull the results of the base simulation running the chosen scenario
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
