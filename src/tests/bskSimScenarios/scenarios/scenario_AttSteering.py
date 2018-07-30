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
# Demonstrates how to use the MRP_Steering() module to stabilize the attitude relative to the Hill Frame using the
# BSK_sim architecture.
#
# BSK Simulation: Attitude Steering {#scenario_AttSteering}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting earth, using the MRP_Steering module with a rate sub-servo system
# to control the attitude all within the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttSterring.py
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_AttSteering.svg "Illustration")
# Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
# modules. The initial setup for the simulation closely models that of scenario_BasicOrbit.py.
#
# To begin, one must first create a class that will
# inherent from the masterSim class and provide a name to the sim.
# This is accomplished through:
# ~~~~~~~~~~~~~{.py}
#   class scenario_AttSteering(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_AttSteering, self).__init__(masterSim)
#          self.name = 'scenario_AttSteering'
# ~~~~~~~~~~~~~
#
# Following the inheritance, there are three functions within the scenario class that need to be configured by the user:
# configure_initial_conditions(), log_outputs(), and pull_outputs().
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "steeringRW"
# ~~~~~~~~~~~~~
#
# which triggers the steeringRW event within the BSK_FSW.py script.
# ~~~~~~~~~~~~~
#
# The initial conditions for the scenario are the same as found within scenario_FeedbackRW.py.
#
# Within BSK_Scenario.py log_outputs(), the user must log the relevant messages to observe how the spacecraft corrected
# for its initial tumbling through:
# ~~~~~~~~~~~~~{.py}
#         # FSW process outputs
#         samplingTime = self.masterSim.FSWModels.processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.rwMotorTorqueData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
# The data is then pulled using:
# ~~~~~~~~~~~~~{.py}
#            num_RW = 4 # number of wheels used in the scenario
#
#         # Dynamics process outputs: pull log messages below if any
#         RW_speeds = self.masterSim.pullMessageLogData( # dataOmegaRW
#             self.masterSim.DynModels.rwStateEffector.OutputDataString + ".wheelSpeeds", range(num_RW))
#         # FSW process outputs
#         dataUsReq = self.masterSim.pullMessageLogData(
#             self.masterSim.FSWModels.rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
#         sigma_BR = self.masterSim.pullMessageLogData(
#             self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
#         omega_BR_B = self.masterSim.pullMessageLogData(
#             self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         omega_BR_ast = self.masterSim.pullMessageLogData(
#             self.masterSim.FSWModels.mrpSteeringData.outputDataName + ".omega_BastR_B", range(3))
#
# ~~~~~~~~~~~~~
# and then plot the results using:
# ~~~~~~~~~~~~~{.py}
#         # Plot results
#         timeData = dataUsReq[:, 0] * macros.NANO2MIN
#         BSK_plt.plot_attitude_error(timeData, sigma_BR)
#         BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
#         BSK_plt.plot_rate_error(timeData, omega_BR_B)
#         BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
# ~~~~~~~~~~~~~
#
# Custom Dynamics Configurations Instructions
# -----
# The dynamics setup is the same as in `scenario_FeedbackRW.py`.
#
# Custom FSW Configurations Instructions
# -----
# To configure the desired "steeringRW" FSW mode the user must declare the following modules
# within BSK_FSW.py:
# ~~~~~~~~~~~~~{.py}
#         self.hillPointData = hillPoint.hillPointConfig()
#         self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
#         self.hillPointWrap.ModelTag = "hillPoint"
#
#         self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
#         self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
#         self.trackingErrorWrap.ModelTag = "trackingError"
#
#         self.mrpSteeringData = MRP_Steering.MRP_SteeringConfig()
#         self.mrpSteeringWrap = SimBase.setModelDataWrap(self.mrpSteeringData)
#         self.mrpSteeringWrap.ModelTag = "MRP_Steering"
#
#         self.rateServoData = rateServoFullNonlinear.rateServoFullNonlinearConfig()
#         self.rateServoWrap = SimBase.setModelDataWrap(self.rateServoData)
#         self.rateServoWrap.ModelTag = "rate_servo"
#
#         self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
#         self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
#         self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
# ~~~~~~~~~~~~~
# each of which prepare various configuration messages to be attached to the various FSW task. The following shows
# how to attach these models to the various tasks.
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
#         SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteeringWrap, self.mrpSteeringData, 10)
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServoWrap, self.rateServoData, 9)
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)
# ~~~~~~~~~~~~~
# Below is the specific event called when the user sets the modeRequest variable in BSK_scenario.py.
# ~~~~~~~~~~~~~{.py}
#          SimBase.createNewEvent("initiateSteeringRW", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'steeringRW'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('hillPointTask')",
#                                 "self.enableTask('mrpSteeringRWsTask')"])
# ~~~~~~~~~~~~~
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

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')
import scenarioAttitudeSteering as scene_plt


# Create your own scenario child class
class scenario_AttitudeSteeringRW(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_AttitudeSteeringRW, self).__init__(masterSim)
        self.name = 'scenario_AttitudeSteeringRW'
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'steeringRW'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # [m]
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # [m]
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # [m/s]
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.5], [0.6], [-0.3]]
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [-0.01]]

    def log_outputs(self):
        print '%s: log_outputs' % self.name
        samplingTime = self.masterSim.DynModels.processTasksTimeStep
        # Dynamics process outputs:
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.rwStateEffector.OutputDataString, samplingTime)
        # FSW process outputs
        samplingTime = self.masterSim.FSWModels.processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpSteeringData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.rwMotorTorqueData.outputDataName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        RW_speeds = self.masterSim.pullMessageLogData( # dataOmegaRW
            self.masterSim.DynModels.rwStateEffector.OutputDataString + ".wheelSpeeds", range(num_RW))
        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        omega_BR_ast = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.mrpSteeringData.outputDataName + ".omega_BastR_B", range(3))

        # Plot results
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        scene_plt.plot_attitude_error(timeData, sigma_BR)
        scene_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        scene_plt.plot_rate_error(timeData, omega_BR_B, omega_BR_ast)
        scene_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    # Instantiate base simulation
    TheBSKSim = BSKSim()

    # Configure a scenario in the base simulation
    TheScenario = scenario_AttitudeSteeringRW(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print 'Starting Execution'
    TheBSKSim.ExecuteSimulation()
    print 'Finished Execution. Post-processing results'

    # Pull the results of the base simulation running the chosen scenario
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
