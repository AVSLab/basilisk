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
# Demonstrates how to create a 6-DOF spacecraft which is orbiting a planet. The purpose is to illustrate how to create
# a spacecraft, attach a gravity model, and run a basic Basilisk simulation using the BSK_Sim architecture.
#
# BSK Simulation: Attitude Steering {#scenario_AttSteering}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting earth, using the MRP_Steering module with a rate sub-servo system
# to conrtrol the attitude all within the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttSterring.py
#
# The simulation layout is shown in the following illustration.  Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
# modules. The initial setup for the simulation closely models that of scenario_BasicOrbit.py.
#
# To begin, one must first create a class that will
# inherient from the masterSim class within the __init__() proceedure and providing a name to the sim.
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
# which triggers the steeringRW event within the BSK_FSW.py script. The corresponding code within the BSK_FSW.py that
# allows for this event to be triggered includes:
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
# each of which prepare various configuraiton messages to be attached to the various FSW task. The following shows
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
# The initial conditions for the scenario are the same as found within scenario_BasicOrbit.py
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
#           # Plot results
#         timeData = dataUsReq[:, 0] * macros.NANO2MIN
#         scene_plt.plot_attitude_error(timeData, sigma_BR)
#         scene_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
#         scene_plt.plot_rate_error(timeData, omega_BR_B, omega_BR_ast)
#         scene_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
# ~~~~~~~~~~~~~
#






#
#
# Custom Configurations Instructions
# -----
# The benefit of the BSK_Simulation architecture is its user simplicity. Things like reaction wheel configurations and
# coarse sun sensor constellations are all preconfigured; however, for users who would like to customize their own
# dynamics module configurations and FSW modes, we recommend copying the three primary BSK_Sim.py files
# (BSK_Scenario.py, BSK_Dynamics.py, and BSK_FSW.py) and modifying them to their liking. Below documents the general
# proceedure for configuring a user customized BSK_Dynamics and BSK_FSW files.
#
# Custom Dynamics Configurations Instructions
# -----
# In BSK_Dynamics.py, it is revealed that BSK_Dynamics.py is a class whose __init__() first generates a task onto which
# future dynamics modules will be added.
# ~~~~~~~~~~~~~{.py}
#         # Create task
#         SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))
# ~~~~~~~~~~~~~
# Following the task generation, all desired dynamics module object are generated:
# ~~~~~~~~~~~~~{.py}
#         # Instantiate Dyn modules as objects
#         self.scObject = spacecraftPlus.SpacecraftPlus()
#         self.ephemerisSPICEObject = spice_interface.SpicePlanetStateSimMsg()
#         self.ephemerisSunSPICEObject = spice_interface.SpicePlanetStateSimMsg()
#         self.earthGravBody = gravityEffector.GravBodyData()
#         self.gravFactory = simIncludeGravBody.gravBodyFactory()
#         self.extForceTorqueObject = extForceTorque.ExtForceTorque()
#         self.simpleNavObject = simple_nav.SimpleNav()
#         self.eclipseObject = eclipse.Eclipse()
#         self.CSSObject = coarse_sun_sensor.CoarseSunSensor()
#         self.CSSConstellationObject = coarse_sun_sensor.CSSConstellation()
#         self.imuObject = imu_sensor.ImuSensor()
#         self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
# ~~~~~~~~~~~~~
# These objects are then configured through InitAllDynObjects(SimBase) which iterates through a number of 'setting'
# functions, that configure all of the dynamics objects properties and messages. Following the configuration of all
# dynamics objects' messages and properties, BSK_Dynamics.py initializes all one-time messages like SPICE messages.
# Finally, all desired objects are attached to the DynamicsTask through:
# ~~~~~~~~~~~~~{.py}
#         # Assign initialized modules to tasks
#         SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
#         SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
#         SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
#         SimBase.AddModelToTask(self.taskName, self.CSSObject, None, 202)
#         SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, None, 203)
#         SimBase.AddModelToTask(self.taskName, self.eclipseObject, None, 204)
#         SimBase.AddModelToTask(self.taskName, self.imuObject, None, 205)
#         SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 301)
#         SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, None, 300)
# ~~~~~~~~~~~~~
#
# Custom FSW Configurations Instructions
# -----
# Similar to BSK_Dynamics.py, BSK_FSW.py's __init__() proceedure beings by defining all possible configuration messages
# required for future functionality. A sample of such process is seen here:
# ~~~~~~~~~~~~~{.py}
#         # Create module data and module wraps
#         self.inertial3DData = inertial3D.inertial3DConfig()
#         self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
#         self.inertial3DWrap.ModelTag = "inertial3D"
#
#         self.hillPointData = hillPoint.hillPointConfig()
#         self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
#         self.hillPointWrap.ModelTag = "hillPoint"
#
#         self.sunSafePointData = sunSafePoint.sunSafePointConfig()
#         self.sunSafePointWrap = SimBase.setModelDataWrap(self.sunSafePointData)
#         self.sunSafePointWrap.ModelTag = "sunSafePoint"
#
#         self.velocityPointData = velocityPoint.velocityPointConfig()
#         self.velocityPointWrap = SimBase.setModelDataWrap(self.velocityPointData)
#         self.velocityPointWrap.ModelTag  = "velocityPoint"
# ~~~~~~~~~~~~~
# Following the initial declaration of these configuration modules, BSK_FSW.py calls a InitAllFSWObjects() command,
# which, like BSK_Dynamics's InitAllDynObjects() configures each of these modules with the appropriate information and
# message names.
#
# After each configuration module has been properly intialized with various message links, various tasks are generated
# within the module.
# ~~~~~~~~~~~~~{.py}
#         # Create tasks
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 20)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpSteeringRWsTask", self.processTasksTimeStep), 10)
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)
# ~~~~~~~~~~~~~
# Each task then has various FSW models uniquely attached to it:
# ~~~~~~~~~~~~~{.py}
#         # Assign initialized modules to tasks
#         SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
#         SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
#         SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
#         SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
# ~~~~~~~~~~~~~
# Finally, the FSW.py file disables all existing tasks, and creates events, which are triggered by the modeRequest call
# within the BSK_Scenario.py file. These events enable various tasks, and their associated FSW models to offer a
# particular FSW mode for the spacecraft.
# ~~~~~~~~~~~~~{.py}
#         # Create events to be called for triggering GN&C maneuvers
#         SimBase.fswProc.disableAllTasks()
#         SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'inertial3D'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('inertial3DPointTask')",
#                                 "self.enableTask('mrpFeedbackTask')"])
#
#         SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'hillPoint'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('hillPointTask')",
#                                 "self.enableTask('mrpFeedbackTask')"])
# ~~~~~~~~~~~~~
#








# ~~~~~~~~~~~~~{.py}
#           # Plot results
#         timeData = dataUsReq[:, 0] * macros.NANO2MIN
#         scene_plt.plot_attitude_error(timeData, sigma_BR)
#         scene_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
#         scene_plt.plot_rate_error(timeData, omega_BR_B, omega_BR_ast)
#         scene_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
# ~~~~~~~~~~~~~
#  ~~~~~~~~~~~~~{.py}
# #         # Create events to be called for triggering GN&C maneuvers
# #         SimBase.fswProc.disableAllTasks()
# #         SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
# #                                ["self.modeRequest == 'inertial3D'"],
# #                                ["self.fswProc.disableAllTasks()",
# #                                 "self.enableTask('inertial3DPointTask')",
# #                                 "self.enableTask('mrpFeedbackTask')"])
# #
# #         SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
# #                                ["self.modeRequest == 'hillPoint'"],
# #                                ["self.fswProc.disableAllTasks()",
# #                                 "self.enableTask('hillPointTask')",
# #                                 "self.enableTask('mrpFeedbackTask')"])
# # ~~~~~~~~~~~~~
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
