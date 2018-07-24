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
## \defgroup Tutorials_6_0
## @{
# Demonstrates how to create a 6-DOF spacecraft which is orbiting a planet. The purpose is to illustrate how to create
# a spacecraft, attach a gravity model, and run a basic Basilisk simulation using the BSK_Sim architecture.
#
# BSK Simulation: Basic Orbit {#scenario_BasicOrbit}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth using the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_BasicOrbit.py
#
# The simulation layout is shown in the following illustration.  Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
# modules. The benefit of the new BSK_Sim architecture is it allows the user to have a pre-built spacecraft
# configurations neatly organized within three simple, similarly-structured files: BSK_scenario.py, BSK_FSW.py, and
# BSK_Dynamics.py.
#
# More explicitly, the purpose of the BSK_Scenario.py within the BSK_Simulation architecture is to provide the user a
# simple, front-end interface to configure a scenario without having to individually initialize and integrate each
# dynamics and FSW module into their simulation. Instead BSK_Dynamics.py will preconfigure the desired dynamics modules,
# attach them to the spacecraft, and link the appropriate messages to the desired FSW modules automatically.
# Similarly, BSK_FSW.py accomplishes a similar goal, through rather than preconfigure all dynamics modules onto the spacecraft,
# BSK_FSW.py instead creates preconfigured events for various FSW operating modes such as hill pointing, sun safe
# pointing, velocity pointing, and more. These preconfigured events automatically enable various tasks, which in turn,
# assign various FSW models to the tasks. Together, these sequence of events initialize the required FSW modules, link
# the messages sent via the dynamics modules, and provide pre-written FSW functionality directly through a simple
# modeRequest variable within BSK_Scenario.py.
#
# To write a custom BSK_scenario.py file first create a class that will
# inherient from the masterSim class within the __init__() proceedure and providing a name to the sim.
# This is accomplished through:
# ~~~~~~~~~~~~~{.py}
#   class scenario_BasicOrbit(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_BasicOrbit, self).__init__(masterSim)
#          self.name = 'scenario_BasicOrbit'
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
# this is the parameter that triggers the afformentioned BSK_FSW event. Additional FSW modes include
# sunSafePoint, inertial3D, velocityPoint, hillPoint, and more. Additionally, the user needs to supply initial conditions
# for the spacecraft. The following code produces a basic orbit:
# ~~~~~~~~~~~~~{.py}
#         # Configure Dynamics initial conditions
#         oe = orbitalMotion.ClassicElements()
#         oe.a = 10000000.0  # meters
#         oe.e = 0.1
#         oe.i = 33.3 * macros.D2R
#         oe.Omega = 48.2 * macros.D2R
#         oe.omega = 347.8 * macros.D2R
#         oe.f = 85.3 * macros.D2R
#         mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
#         rN, vN = orbitalMotion.elem2rv(mu, oe)
#         orbitalMotion.rv2elem(mu, rN, vN)
#         self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#         self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#         self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
#         self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
# ~~~~~~~~~~~~~
# The `self.masterSim.DynModels` brings the user to a list of active dynamic modules used within BSK_Dynamics.py.
#
# Within the log_outputs() function, the user can supply a list of messages they are interested in logging. For a
# basic orbit, we need position and attitude.
# ~~~~~~~~~~~~~{.py}
#       samplingTime = self.masterSim.DynModels.processTasksTimeStep
#       self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputAttName, samplingTime)
#       self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputTransName, samplingTime)
# ~~~~~~~~~~~~~
#
# Finally within the pull_outputs(), the user can pull specific variables from the messages:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputAttName + ".sigma_BN", range(3))
#         r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".v_BN_N", range(3))
# ~~~~~~~~~~~~~
# and proceed to graph them using predefined plotting routines in BSK_Plotting.py
# ~~~~~~~~~~~~~{.py}
#         BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)
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


# Create your own scenario child class
class scenario_BasicOrbit(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_BasicOrbit, self).__init__(masterSim)
        self.name = 'scenario_BasicOrbit'

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'hillPoint'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
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

        # Plot results
        timeLineSet = sigma_BR[:, 0] * macros.NANO2MIN
        BSK_plt.plot_orbit(r_BN_N)
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)
        BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "orientation", "attitudeGuidance"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    # Instantiate base simulation
    TheBSKSim = BSKSim()

    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbit(TheBSKSim)
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
