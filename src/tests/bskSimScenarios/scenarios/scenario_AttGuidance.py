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

## \defgroup Tutorials_6_2
## @{
# Demonstrates how to use the hill pointing model within the BSK_Sim architecture.
#
# BSK Simulation: Attitude Guidance {#scenario_AttGuidance}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting Earth. The goal of the scenario is to
# make use of the hill pointing module with
# the MRP_Feedback module and a reaction wheel pyramid
# to control the attitude all within the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttGuidance.py
#
# The simulation mimics the basic simulation simulation in the earlier tutorial in
# [scenarioAttitudeGuidance.py](@ref scenarioAttitudeGuidance).
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_AttGuidance.svg "Illustration")
# The initial setup for the simulation closely models that of scenario_FeedbackRW.py.
#
# To begin, the scenario must be initialized using:
# ~~~~~~~~~~~~~{.py}
#   class scenario_AttGuidance(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_AttGuidance, self).__init__(masterSim)
#          self.name = 'scenario_AttGuidance'
# ~~~~~~~~~~~~~
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "hillPoint"
# ~~~~~~~~~~~~~
# which triggers the `initiateHillPoint` event within the BSK_FSW.py script.
#
# The initial conditions for the scenario are the same as found within scenario_FeedbackRW.py. Within BSK_Scenario.py
# log_outputs(), the user must log the relevant messages to observe the spacecraft attitude's error throughout
# its orbit:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         samplingTime = self.masterSim.get_DynModel().processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputAttName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputTransName, samplingTime)
#
#         # FSW process outputs
#         samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().hillPointData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpFeedbackControlData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
# The data is then pulled using:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", range(3))
#         r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))
#
#         # FSW process outputs
#         sigma_RN = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.inputRefName + ".sigma_RN", range(3))
#         omega_RN_N = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.inputRefName + ".omega_RN_N", range(3))
#         sigma_BR = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
#         omega_BR_B = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         Lr = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))
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
# The modules required for this scenario are identical to those used in [scenario_FeedbackRW.py](@ref scenario_FeedbackRW).
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
# Three of the four modules required to configure the "hillPoint" FSW mode have already been included within the BSK_FSW.py framework
# (`mrpFeedbackRWConfig()`, `attTrackingErrorConfig()`,`rwMotorTorqueConfig()`). The only remaining
# module is the hill pointing module itself which is set within `__init__()`:
# ~~~~~~~~~~~~~{.py}
#         self.hillPointData = hillPoint.hillPointConfig()
#         self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
#         self.hillPointWrap.ModelTag = "hillPoint"
# ~~~~~~~~~~~~~
# These modules provide the initial setup for an attitude guidance system that makes use of an hill pointing model, a module
# that tracks the error of the spacecraft's MRP parameters against the vector pointing towards the central, planetary
# body, and uses a module that takes that information to provide a torque to correct for the error.
#
# Within `InitAllFSWObjects()` only one new setter functions is required beyond what was provided in the past two scenarios.
# ~~~~~~~~~~~~~{.py}
#     def SetHillPointGuidance(self, SimBase):
#         self.hillPointData.outputDataName = "att_reference"
#         self.hillPointData.inputNavDataName = SimBase.get_DynModel().simpleNavObject.outputTransName
#         self.hillPointData.inputCelMessName = SimBase.get_DynModel().gravFactory.gravBodies['earth'].bodyInMsgName[:-12]
# ~~~~~~~~~~~~~
# Once the module is configured, the user can create a new pointing task for hill pointing:
# ~~~~~~~~~~~~~{.py}
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
# ~~~~~~~~~~~~~
# and then to add the tracking model to the hill point task through:
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
#         SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
# ~~~~~~~~~~~~~
# Finally, a `initiateHillPoint` event is defined which combines the new hill pointing task with the pre-existing
# `mrpFeedbackTask`:
# ~~~~~~~~~~~~~{.py}
#              SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'hillPoint'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('hillPointTask')",
#                                 "self.enableTask('mrpFeedbackTask')"])
# ~~~~~~~~~~~~~
# This event is triggered when a user calls `self.masterSim.modeRequest = 'hillPoint'` in any current or future
# BSK_scenario.py file.
#
# Numerical Simulation Results
# ------------
# If this simulation is run, then the following plots should be shown.
# ![](Images/Scenarios/scenario_AttGuidance_attitudeErrorNorm.svg)
# ![](Images/Scenarios/scenario_AttGuidance_rwMotorTorque.svg)
# ![](Images/Scenarios/scenario_AttGuidance_rateError.svg)
# ![](Images/Scenarios/scenario_AttGuidance_orientation.svg)
# ![](Images/Scenarios/scenario_AttGuidance_attitudeGuidance.svg)
#
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
import BSK_Dynamics, BSK_Fsw

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

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
        mu = self.masterSim.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    def log_outputs(self):
        print '%s: log_outputs' % self.name
        # Dynamics process outputs
        samplingTime = self.masterSim.get_DynModel().processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputAttName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputTransName, samplingTime)

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().hillPointData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpFeedbackRWsData.outputDataName, samplingTime)

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        # Dynamics process outputs
        sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", range(3))
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
        v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))

        # FSW process outputs
        sigma_RN = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.inputRefName + ".sigma_RN", range(3))
        omega_RN_N = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.inputRefName + ".omega_RN_N", range(3))
        sigma_BR = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        Lr = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().mrpFeedbackRWsData.outputDataName + ".torqueRequestBody", range(3))

        # Plot results
        BSK_plt.clear_all_plots()
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
    TheBSKSim.set_DynModel(BSK_Dynamics)
    TheBSKSim.set_FswModel(BSK_Fsw)
    TheBSKSim.initInterfaces()

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
