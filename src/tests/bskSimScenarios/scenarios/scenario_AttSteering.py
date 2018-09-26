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
## \defgroup Tutorials_6_4
## @{
# Demonstrates how to use the MRP_Steering() module to stabilize the attitude relative to the Hill Frame using the
# BSK_sim architecture.
#
# BSK Simulation: Attitude Steering {#scenario_AttSteering}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting Earth. The goal of this tutorial is to demonstrate
# how to configure and use the MRP_Steering module with a rate sub-servo system
# the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttSteering.py
#
# The simulation mimics the basic simulation simulation in the earlier tutorial in
# [scenarioAttSteering.py](@ref scenarioAttitudeSteering).
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_AttSteering.svg "Illustration")
#
# The scenario is initialized through:
# ~~~~~~~~~~~~~{.py}
#   class scenario_AttSteering(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_AttSteering, self).__init__(masterSim)
#          self.name = 'scenario_AttSteering'
# ~~~~~~~~~~~~~
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "steeringRW"
# ~~~~~~~~~~~~~
#
# which triggers the `initiateSteeringRW` event within the BSK_FSW.py script.
#
# The initial conditions for the scenario are the same as found within [scenario_FeedbackRW.py](@ref scenario_FeedbackRW)
#
# Within BSK_Scenario.py log_outputs(), the user must log the relevant messages to observe how the spacecraft corrected
# for its initial tumbling through:
# ~~~~~~~~~~~~~{.py}
#         # FSW process outputs
#         samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpSteeringData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
# The data is then pulled using:
# ~~~~~~~~~~~~~{.py}
#            num_RW = 4 # number of wheels used in the scenario
#
#         # Dynamics process outputs: pull log messages below if any
#         RW_speeds = self.masterSim.pullMessageLogData( # dataOmegaRW
#             self.masterSim.get_DynModel().rwStateEffector.OutputDataString + ".wheelSpeeds", range(num_RW))
#         # FSW process outputs
#         dataUsReq = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
#         sigma_BR = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
#         omega_BR_B = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         omega_BR_ast = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().mrpSteeringData.outputDataName + ".omega_BastR_B", range(3))
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
#
#
# Custom Dynamics Configurations Instructions
# -----
# The dynamics setup is the same as in [scenario_FeedbackRW.py](@ref scenario_FeedbackRW).
#
# Custom FSW Configurations Instructions
# -----
# To configure the desired "steeringRW" FSW mode the user must add the following modules to BSK_FSW.py
# within BSK_FSW.py:
# ~~~~~~~~~~~~~{.py}
#         self.mrpSteeringData = MRP_Steering.MRP_SteeringConfig()
#         self.mrpSteeringWrap = SimBase.setModelDataWrap(self.mrpSteeringData)
#         self.mrpSteeringWrap.ModelTag = "MRP_Steering"
#
#         self.rateServoData = rateServoFullNonlinear.rateServoFullNonlinearConfig()
#         self.rateServoWrap = SimBase.setModelDataWrap(self.rateServoData)
#         self.rateServoWrap.ModelTag = "rate_servo"
#
# ~~~~~~~~~~~~~
# each of which prepare various configuration messages to be attached to the various FSW task. The following code shows
# how to create a new control task, `mrpSteeringRWsTask`, and how to attach the configuration data to the task.
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteeringWrap, self.mrpSteeringData, 10)
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServoWrap, self.rateServoData, 9)
#         SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)
# ~~~~~~~~~~~~~
# The advantage of the BSK_Sim architecture becomes apparent here again, as the `trackingErrorData` and `rwMotorTorqueData`
# data were already defined from an earlier scenario as well as the entire `hillPointingTask`.
# The user can simply add them to their desired task without
# having to manually reconfigure the messages. These tasks are then enabled when the user sets the modeRequest variable
# to `steeringRW` in BSK_scenario.py.
# ~~~~~~~~~~~~~{.py}
#          SimBase.createNewEvent("initiateSteeringRW", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'steeringRW'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('hillPointTask')",
#                                 "self.enableTask('mrpSteeringRWsTask')"])
# ~~~~~~~~~~~~~
#
#
# Numerical Simulation Results
# ------------
# If this simulation is run, then the following plots should be shown.
# ![Attitude Errors](Images/Scenarios/scenario_AttSteering_attitudeErrorNorm.svg "Attitude Tracking history")
# ![RW Motor Torques](Images/Scenarios/scenario_AttSteering_rwMotorTorque.svg "RW motor torque history")
# ![Angular Velocities](Images/Scenarios/scenario_AttSteering_rateError.svg "Body Rate history")
# ![RW Spin Rates](Images/Scenarios/scenario_AttSteering_rwSpeed.svg "RW Speed history")
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

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

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
        mu = self.masterSim.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # [m]
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # [m/s]
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.5], [0.6], [-0.3]]
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [-0.01]]

    def log_outputs(self):
        print '%s: log_outputs' % self.name
        samplingTime = self.masterSim.get_DynModel().processTasksTimeStep
        # Dynamics process outputs:
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().rwStateEffector.OutputDataString, samplingTime)
        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpSteeringData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        RW_speeds = self.masterSim.pullMessageLogData( # dataOmegaRW
            self.masterSim.get_DynModel().rwStateEffector.OutputDataString + ".wheelSpeeds", range(num_RW))

        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        omega_BR_ast = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().mrpSteeringData.outputDataName + ".omega_BastR_B", range(3))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
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
    TheBSKSim.set_DynModel(BSK_Dynamics)
    TheBSKSim.set_FswModel(BSK_Fsw)
    TheBSKSim.initInterfaces()

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
