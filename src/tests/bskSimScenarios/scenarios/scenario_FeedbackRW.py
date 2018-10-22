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
## \defgroup Tutorials_6_1
## @{
# Demonstrates how to stabilize the tumble of a 6-DOF spacecraft with reaction wheels in the BSK_Sim architecture.
#
# BSK Simulation: Feedback RW {#scenario_FeedbackRW}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting Earth. The goal of the scenario is to
# 1) add reaction wheels to BSK_Dynamics.py, and 2) establish a inertial pointing FSW mode in BSK_FSW.py.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_FeedbackRW.py
#
# The simulation mimics the basic simulation simulation in the earlier tutorial in
# [scenarioAttitudeFeedbackRW.py](@ref scenarioAttitudeFeedbackRW).
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_FeedbackRW.svg "Illustration")
# Two simulation processes are created: one
# which contains dynamics modules, and one that contains the FSW
# modules. The initial setup for the simulation closely models that of scenario_BasicOrbit.py.
#
# The scenario must inherit from the BSK_master class using:
# ~~~~~~~~~~~~~{.py}
#   class scenario_FeedbackRW(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_FeedbackRW, self).__init__(masterSim)
#          self.name = 'scenario_FeedbackRW'
# ~~~~~~~~~~~~~
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "inertial3D"
# ~~~~~~~~~~~~~
# which triggers the `initiateInertial3D` event within the BSK_FSW.py script.
#
# The initial conditions for the scenario are the same as found within scenario_BasicOrbit.py except the tumble of the
# spacecraft must be simulated by adding:
# ~~~~~~~~~~~~~{.py}
#         self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
#         self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
# ~~~~~~~~~~~~~
# Within BSK_Scenario.py log_outputs(), the user must log additional messages to observe how the spacecraft corrected
# for its initial tumbling through:
# ~~~~~~~~~~~~~{.py}
#         # FSW process outputs
#         samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
# The data is then pulled using:
# ~~~~~~~~~~~~~{.py}
#         dataUsReq = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
#         sigma_BR = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
#         omega_BR_B = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         RW_speeds = self.masterSim.pullMessageLogData(
#             self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", range(num_RW))
#             self.masterSim.get_FswModel().mrpSteeringData.outputDataName + ".omega_BastR_B", range(3))
#
# ~~~~~~~~~~~~~
# and then plot using:
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
#
#
#
#
#
#
#
# Custom Dynamics Configurations Instructions
# -----
# In addition to the modules used in scenario_BasicOrbit.py, the user must configure the RW module in BSK_Dynamics.py
# to stabilize the tumbling. This is accomplished by first creating the RW state effector:
# ~~~~~~~~~~~~~{.py}
#         # Instantiate Dyn modules as objects
#           self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
# ~~~~~~~~~~~~~
# The RW object is then configured through `InitAllDynObjects(SimBase)` which includes the `SetReactionWheelDynEffector()`
# function which configures the RW pyramid's properties and messages.
# ~~~~~~~~~~~~~{.py}
#     # Global call to initialize every module
#     def InitAllDynObjects(self):
#         self.SetReactionWheelDynEffector()
# ~~~~~~~~~~~~~
#
# The setter function itself includes:
#
# ~~~~~~~~~~~~~{.py}
#     def SetReactionWheelDynEffector(self):
#         # Make a fresh RW factory instance, this is critical to run multiple times
#         rwFactory = simIncludeRW.rwFactory()
#
#         # specify RW momentum capacity
#         maxRWMomentum = 50. # Nms
#
#         # Define orthogonal RW pyramid
#         # -- Pointing directions
#         rwElAngle = np.array([40.0, 40.0, 40.0, 40.0])*mc.D2R
#         rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
#         rwPosVector = [[0.8, 0.8, 1.79070],
#                        [0.8, -0.8, 1.79070],
#                        [-0.8, -0.8, 1.79070],
#                        [-0.8, 0.8, 1.79070]
#                        ]
#
#         for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
#             gsHat = (rbk.Mi(-azAngle,3).dot(rbk.Mi(elAngle,2))).dot(np.array([1,0,0]))
#             rwFactory.create('Honeywell_HR16',
#                              gsHat,
#                              maxMomentum=maxRWMomentum,
#                              rWB_B=posVector)
#
#         rwFactory.addToSpacecraft("RWStateEffector", self.rwStateEffector, self.scObject)
# ~~~~~~~~~~~~~
# which generates a RW pyramid using the RW factory and then adds it to the spacecraft. Now all future BSK_Scenarios
# have access to a pre-configured RW pyramid that does not need to be defined for each new simulation.
#
# Following the configuration of all
# dynamics objects' messages and properties, the objects must be attached to the DynamicsTask. In addition to the tasks
# assigned in [scenario_BasicOrbit.py](@ref scenario_BasicOribt), the user must also add:
# ~~~~~~~~~~~~~{.py}
#         # Assign initialized modules to tasks
#         SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 301)
# ~~~~~~~~~~~~~
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
# To configure the desired "inertial3D" FSW mode the user must declare the following modules
# within the `__init__()` function in BSK_FSW.py:
# ~~~~~~~~~~~~~{.py}
#         self.inertial3DData = inertial3D.inertial3DConfig()
#         self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
#         self.inertial3DWrap.ModelTag = "inertial3D"
#
#         self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
#         self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
#         self.trackingErrorWrap.ModelTag = "trackingError"
#
#         self.mrpFeedbackRWsData = MRP_Feedback.MRP_FeedbackConfig()
#         self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
#         self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"
#
#         self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
#         self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
#         self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
# ~~~~~~~~~~~~~
# These provide the initial setup for an attitude guidance system that makes use of an inertial pointing model, a module
# that tracks the error of the spacecraft's MRP parameters against the pointing model, and a module that takes that
# information to provide a torque to correct for the error.
#
# Following the initial declaration of these configuration modules, BSK_FSW.py calls a `InitAllFSWObjects()` command,
# which, like BSK_Dynamics's InitAllDynObjects(), calls additional setter functions that configure each of the FSW modules
# with the appropriate information and message names.
#
# Within `InitAllFSWObjects()` these modules are configured by calling the setter functions:
# ~~~~~~~~~~~~~{.py}
#     # Global call to initialize every module
#     def InitAllFSWObjects(self, SimBase):
#         self.SetInertial3DPointGuidance()
#         self.SetAttitudeTrackingError(SimBase)
#         self.SetMRPFeedbackControl(SimBase)
#         self.SetVehicleConfiguration(SimBase)
#         self.SetRWConfigMsg(SimBase)
#         self.SetMRPFeedbackRWA()
#         self.SetRWMotorTorque(SimBase)
# ~~~~~~~~~~~~~
# Which configure the FSW modules as seen below:
# ~~~~~~~~~~~~~{.py}
#
#     def SetInertial3DPointGuidance(self):
#         self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
#         self.inertial3DData.outputDataName = "referenceOut"
#
#     def SetAttitudeTrackingError(self, SimBase):
#         self.trackingErrorData.inputNavName = SimBase.get_DynModel().simpleNavObject.outputAttName
#         # Note: SimBase.get_DynModel().simpleNavObject.outputAttName = "simple_att_nav_output"
#         self.trackingErrorData.inputRefName = "referenceOut"
#         self.trackingErrorData.outputDataName = "guidanceOut"
#
#      def SetMRPFeedbackControl(self, SimBase):
#         self.mrpFeedbackControlData.inputGuidName = "guidanceOut"
#         self.mrpFeedbackControlData.vehConfigInMsgName = "adcs_config_data"
#         self.mrpFeedbackControlData.outputDataName = SimBase.get_DynModel().extForceTorqueObject.cmdTorqueInMsgName
#         # Note: SimBase.get_DynModel().extForceTorqueObject.cmdTorqueInMsgName = "extTorquePntB_B_cmds"
#
#         self.mrpFeedbackControlData.K = 3.5
#         self.mrpFeedbackControlData.Ki = -1.0 # Note: make value negative to turn off integral feedback
#         self.mrpFeedbackControlData.P = 30.0
#         self.mrpFeedbackControlData.integralLimit = 2. / self.mrpFeedbackControlData.Ki * 0.1
#         self.mrpFeedbackControlData.domega0 = [0.0, 0.0, 0.0]
#
#     def SetMRPFeedbackRWA(self):
#         self.mrpFeedbackRWsData.K = 3.5
#         self.mrpFeedbackRWsData.Ki = -1  # Note: make value negative to turn off integral feedback
#         self.mrpFeedbackRWsData.P = 30.0
#         self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1
#         self.mrpFeedbackRWsData.domega0 = [0.0, 0.0, 0.0]
#
#         self.mrpFeedbackRWsData.vehConfigInMsgName = "adcs_config_data"
#         self.mrpFeedbackRWsData.inputRWSpeedsName = "reactionwheel_output_states"
#         self.mrpFeedbackRWsData.rwParamsInMsgName = "rwa_config_data"
#         self.mrpFeedbackRWsData.inputGuidName = "guidanceOut"
#         self.mrpFeedbackRWsData.outputDataName = "controlTorqueRaw"
#
#     def SetRWMotorTorque(self, SimBase):
#         controlAxes_B = [
#         1.0, 0.0, 0.0
#         , 0.0, 1.0, 0.0
#         , 0.0, 0.0, 1.0
#         ]
#         self.rwMotorTorqueData.controlAxes_B = controlAxes_B
#         self.rwMotorTorqueData.inputVehControlName = "controlTorqueRaw"
#         self.rwMotorTorqueData.outputDataName = SimBase.get_DynModel().rwStateEffector.InputCmds  # "reactionwheel_cmds"
#         self.rwMotorTorqueData.rwParamsInMsgName = "rwa_config_data"
# ~~~~~~~~~~~~~
# Note how the messages occassionaly pull output data from the `SimBase.get_DynModel()` to link messages from BSK_Dynamics.py.
#
# In addition to the modules used for attitude guidance, there are also two setter functions that send vehicle and RW
# configuration messages that are linked into the attitude guidance modules:
# ~~~~~~~~~~~~~{.py}
#     def SetVehicleConfiguration(self, SimBase):
#         vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
#         # use the same inertia in the FSW algorithm as in the simulation
#         vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
#         unitTestSupport.setMessage(SimBase.TotalSim,
#                                    SimBase.FSWProcessName,
#                                     "adcs_config_data",
#                                     vehicleConfigOut)
#
#     def SetRWConfigMsg(self, SimBase):
#         # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
#         rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
#         rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
#         wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)
#
#         fswSetupRW.clearSetup()
#         for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
#             gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
#             fswSetupRW.create(gsHat,  # spin axis
#                               wheelJs,  # kg*m^2
#                               0.2)  # Nm        uMax
#
#         fswSetupRW.writeConfigMessage("rwa_config_data", SimBase.TotalSim, SimBase.FSWProcessName)
# ~~~~~~~~~~~~~
# After each configuration module has been properly initialized with various message names, FSW tasks are generated.
# The two tasks required for the "inertial3D" mode are `inertial3DPointTask` and `mrpFeedbackRWsTask` and they are
# generated through:
# ~~~~~~~~~~~~~{.py}
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)
# ~~~~~~~~~~~~~
# Note how the tasks are divided between the pointing model and control loop. These modular tasks allow
# for simple FSW reconfigurations should the user want to use a different pointing model, but to use the same feedback
# control loop. This will be seen and discussed in later scenarios.
#
# Each task then has various FSW models added to it:
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
#         SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
#         SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
#         SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)
# ~~~~~~~~~~~~~
# Finally, the `inertial3D` mode call in scenario_FeedbackRW.py needs to be triggered by:
# ~~~~~~~~~~~~~{.py}
#                  SimBase.createNewEvent("initiateInertial3D", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'inertial3D'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('inertial3DPointTask')",
#                                 "self.enableTask('mrpFeedbackRWsTask')"])
# ~~~~~~~~~~~~~
# which disables any existing tasks and enables the inertial pointing task and RW feedback task.
# This concludes how to construct a preconfigured FSW mode that will be available for any future scenario
# that uses the BSK_Sim architecture.
#
# Numerical Simulation Results
# ------------
# If this simulation is run, then the following plots should be shown.
# ![Attitude Errors](Images/Scenarios/scenario_FeedbackRW_attitudeErrorNorm.svg "Attitude Tracking history")
# ![RW Motor Torques](Images/Scenarios/scenario_FeedbackRW_rwMotorTorque.svg "RW motor torque history")
# ![Angular Velocities](Images/Scenarios/scenario_FeedbackRW_rateError.svg "Body Rate history")
# ![RW Spin Rates](Images/Scenarios/scenario_FeedbackRW_rwSpeed.svg "RW Speed history")
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
class scenario_AttitudeFeedbackRW(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_AttitudeFeedbackRW, self).__init__(masterSim)
        self.name = 'scenario_AttitudeFeedbackRW'
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'inertial3D'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.01
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

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any

        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        RW_speeds = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", range(num_RW))

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
    TheScenario = scenario_AttitudeFeedbackRW(TheBSKSim)
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
