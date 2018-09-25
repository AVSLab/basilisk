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
# Demonstrates how to create a 3-DOF spacecraft which is orbiting Earth using the BSK_Sim architecture.
#
# BSK Simulation: Basic Orbit {#scenario_BasicOrbit}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting the Earth. The goal of the scenario is to
# 1) highlight the structure of the BSK_Sim architecture, 2) demonstrate how to create a custom BSK_scenario, and 3)
# how to customize the BSK_Dynamics.py and BSK_FSW.py files.
#
# To run the default scenario, call the bskSim python simulation script withing `src\tests\bskSimScenarios\scenarios`
#  from a Terminal window through:
#
#       python scenario_BasicOrbit.py
#
# The simulation mimics the basic simulation simulation in the earlier tutorial in
# [scenarioBasicOrbit.py](@ref scenarioBasicOrbit).  But rather than explicitly defining all simulation properties
# within the python simulation file, the bskSim spacecraft simulation class is used to encapsulate a lot of the
# setup and configuring.
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_basicOrbit_v1.1.svg "Illustration")
# Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW)
# modules. The benefit of the new BSK_Sim architecture is how it allows the user to have a pre-written spacecraft
# configurations and FSW modes neatly organized within three modular files: a BSK_scenario file, a FSW file, and
# a Dynamics file.
#
# More explicitly, the purpose of the scenario file (in this case scenario_BasicOrbit.py) within the BSK_Simulation architecture is to provide the user a
# simple, front-end interface to configure a scenario without having to individually initialize and integrate each
# dynamics and FSW module into their simulation. Instead the Dynamics file (for instance BSK_Dynamics.py or BSK_FormationDynamics.py)
# has preconfigured many dynamics modules, attached them to the spacecraft, and linked their messages to the appropriate FSW modules.
# Similarly, the FSW file (in this case BSK_FSW.py) creates preconfigured FSW modes such as hill pointing, sun safe
# pointing, velocity pointing, and more. Each preconfigured mode triggers a specific event which enables various FSW tasks
# like assigning enabling a specific pointing model or control loop. The proceeding sequence of tasks then initialize the
# appropriate FSW modules, link their messages, and provide pre-written FSW functionality through a simple
# modeRequest variable within the BSK_scenario file.
#
# Configuring the scenario file
# -----
# To write a custom scenario file, first create a class that will
# inherent from the masterSim class using:
# ~~~~~~~~~~~~~{.py}
#   class scenario_BasicOrbit(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_BasicOrbit, self).__init__(masterSim)
#          self.name = 'scenario_BasicOrbit'
# ~~~~~~~~~~~~~
#
# Following the inheritance, there are three functions within the scenario class that need to be defined by the user:
# `configure_initial_conditions()`, `log_outputs()`, and `pull_outputs()`.
#
# Within `configure_initial_conditions()`, the user needs to define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "standby"
# ~~~~~~~~~~~~~
# this is the parameter that triggers the aforementioned FSW event. Additional FSW modes (to be discussed in later
# tutorials) include sunSafePoint, inertial3D, velocityPoint, hillPoint, and more.
#
# Additionally, the user needs to supply initial conditions
# for the spacecraft and its orbit. The following code uses the orbitalMotion module to
# construct the appropriate position and velocity vectors for a stable orbit, and then assigns them to the
# spacecraft:
# ~~~~~~~~~~~~~{.py}
#         # Configure Dynamics initial conditions
#         oe = orbitalMotion.ClassicElements()
#         oe.a = 10000000.0  # meters
#         oe.e = 0.1
#         oe.i = 33.3 * macros.D2R
#         oe.Omega = 48.2 * macros.D2R
#         oe.omega = 347.8 * macros.D2R
#         oe.f = 85.3 * macros.D2R
#         mu = self.masterSim.get_DynModel().gravFactory.gravBodies['earth'].mu
#         rN, vN = orbitalMotion.elem2rv(mu, oe)
#         orbitalMotion.rv2elem(mu, rN, vN)
#         self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#         self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
# ~~~~~~~~~~~~~
# The `self.masterSim.get_DynModel()` is referencing a list of available dynamic modules preconfigured in the Dynamics file.
#
# Within `log_outputs()`, the user can supply a list of messages they are interested in logging. Position and velocity
# from the navigation message are relevant to verify proper orbit functionality.
# ~~~~~~~~~~~~~{.py}
#       samplingTime = self.masterSim.get_DynModel().processTasksTimeStep
#       self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputTransName, samplingTime)
# ~~~~~~~~~~~~~
#
# Finally within the pull_outputs(), the user can pull specific variables from the logged messages:
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))
# ~~~~~~~~~~~~~
# and proceed to graph them using predefined plotting routines in BSK_Plotting.py
# ~~~~~~~~~~~~~{.py}
#         BSK_plt.plot_orbit(r_BN_N)
# ~~~~~~~~~~~~~
#
#
#
# Custom Configurations Instructions
# -----
# The benefit of the BSK_Simulation architecture is its user simplicity. Things like spacecraft hub configurations,
# reaction wheel pyramids, and
# coarse sun sensor constellations are all preconfigured; however, for users who would like to customize their own
# dynamics modules and FSW modes, it is recommended to copy the three primary BSK_Sim files
# (BSK_Scenario.py, BSK_Dynamics.py, and BSK_FSW.py) and modify them directly. Instructions for configuring
# user-customized Dynamics and FSW files are detailed below.
#
# **Custom Dynamics Configurations Instructions**
#
# In BSK_Dynamics.py, the script first generates a dynamics task onto which
# future dynamics modules will be added.
# ~~~~~~~~~~~~~{.py}
#         # Create task
#         SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep)
# ~~~~~~~~~~~~~
# Following the task generation, all desired dynamics module objects are generated:
# ~~~~~~~~~~~~~{.py}
#         # Instantiate Dyn modules as objects
#         self.scObject = spacecraftPlus.SpacecraftPlus()
#         self.gravFactory = simIncludeGravBody.gravBodyFactory()
#         self.simpleNavObject = simple_nav.SimpleNav()
# ~~~~~~~~~~~~~
# These objects are then configured through `InitAllDynObjects(SimBase)` which iterates through a number of setter
# functions that configure all of the dynamics objects properties and messages.
# ~~~~~~~~~~~~~{.py}
#     # Global call to initialize every module
#     def InitAllDynObjects(self):
#         self.SetSpacecraftHub()
#         self.SetGravityBodies()
#         self.SetSimpleNavObject()
# ~~~~~~~~~~~~~
#
# The relevant setting functions for scenario_BasicOrbit.py are:
#
# ~~~~~~~~~~~~~{.py}
#     def SetSpacecraftHub(self):
#         self.scObject.ModelTag = "spacecraftBody"
#         # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
#         self.I_sc = [900., 0., 0.,
#                      0., 800., 0.,
#                      0., 0., 600.]
#         self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
#         self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
#         self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)
#         self.scObject.scStateOutMsgName = "inertial_state_output"
#
#     def SetGravityBodies(self):
#         timeInitString = "2012 MAY 1 00:28:30.0"
#         gravBodies = self.gravFactory.createBodies(['earth', 'sun', 'moon'])
#         gravBodies['earth'].isCentralBody = True
#
#         self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravFactory.gravBodies.values())
#         self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', timeInitString)
#         self.gravFactory.spiceObject.zeroBase = 'Earth'
#
#         pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
#         pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
#         pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
#         pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel
#
#     def SetSimpleNavObject(self):
#         self.simpleNavObject.ModelTag = "SimpleNavigation"
# ~~~~~~~~~~~~~
# These setter functions are examples of how the BSK_Sim architecture has preconfigured dynamics modules within the BSK_Dynamics.py.
# Now, for every future scenario file, a spacecraft object, gravity effector, and simple navigation sensor will be available
# for use.
#
# Finally, all now-configured objects are attached to the DynamicsTask through:
# ~~~~~~~~~~~~~{.py}
#         # Assign initialized modules to tasks
#         SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
#         SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
#         SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
# ~~~~~~~~~~~~~
# The number at the end of `AddModelToTask` corresponds with the priority of the model. The higher the number, the earlier
# the model gets evaluated during each time step.
#
# **Custom FSW Configurations Instructions**
#
# BSK_FSW.py's __init__() procedure defines all possible configuration messages to be used by future FSW algorithms.
# Because this scenario is simulating a 3-DOF spacecraft, there are no FSW algorithms needed to control attitude.
#
# As such, a `initializeStandby` event is created within BSK_FSW.py to ensure all FSW tasks are disabled. This event is
# triggered by the modeRequest called in scenario_BasicOrbit.py and executes the following code in BSK_FSW.py:
# ~~~~~~~~~~~~~{.py}
#         # Create events to be called for triggering GN&C maneuvers
#         SimBase.fswProc.disableAllTasks()
#         SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'standby'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 ])
# ~~~~~~~~~~~~~
#
# Numerical Simulation Results
# ------------
# If this simulation is run, then the following plots should be shown.
# ![Inertial Orbit Illustration](Images/Scenarios/scenario_BasicOrbit_orbit.svg "Position history")
# ![Attitude History](Images/Scenarios/scenario_BasicOrbit_orientation.svg "Attitude history")
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

sys.path.append(path + '/../../scenarios')


# Create your own scenario child class
class scenario_BasicOrbit(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_BasicOrbit, self).__init__(masterSim)
        self.name = 'scenario_BasicOrbit'

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'standby'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
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

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        # Dynamics process outputs
        sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", range(3))
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
        v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))

        # Plot results
        BSK_plt.clear_all_plots()
        timeLineSet = r_BN_N[:, 0] * macros.NANO2MIN
        BSK_plt.plot_orbit(r_BN_N)
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "orientation"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    # Instantiate base simulation
    TheBSKSim = BSKSim()
    TheBSKSim.set_DynModel(BSK_Dynamics)
    TheBSKSim.set_FswModel(BSK_Fsw)
    TheBSKSim.initInterfaces()

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
