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

## \page scenario_BasicOrbitFormationGroup
## @{
# Demonstrates how to create two 3-DOF spacecraft orbiting Earth using the BSK_Sim architecture.
#
# BSK Simulation: Basic Orbit Formation {#scenario_BasicOrbitFormation}
# ====
#
# Scenario Description
# -----
# This script sets up two 3-DOF spacecraft orbiting the Earth. The goal of the scenario is to
# 1) highlight how the BSK_Sim structure of a formation flying tutorial is different from the basic orbit scenario,
# 2) demonstrate how to create a formation flying scenario, and
# 3) how to customize the BSK_FormationDynamics.py and BSK_FormationFSW.py files.
#
# To run the default scenario, call the bskSim python simulation script within `src\tests\bskSimScenarios\scenarios`
# from a Terminal window through:
#
#       python scenario_BasicOrbitFormation.py
#
# The simulation mimics the basic simulation in the earlier tutorial in
# [scenario_BasicOrbit.py](@ref scenario_BasicOrbit).
#
# The flight software mode is set to inertial3D. The goal of this mode is to align the body axes of the spacecraft
# with an inertial 3D point guidance coordinate system defined in BSK_FormationFSW.py. However the flight software mode
# can also be set to "standby".
#
# The simulation layout is shown in the following illustrations.
# ![Simulation Flow Diagram](Images/doc/test_scenario_BasicOrbitFormation.svg "Simulation Flow Diagram")
# ![Simulation Container Layout](Images/doc/Simulation_container_layout_BasicOrbitFormation.svg "Simulation Container Layout")
#
# Configuring the scenario file
# -----
# The simulation layout is very similar to the one used for the scenario_BasicOrbit file.
# Two simulation processes are created: one which contains dynamics modules, and one that contains
# the Flight Software (FSW) modules. First of all, it can be observed that the Dynamics- and FSW files used are
# the BSK_FormationDynamics and BSK_FormationFSW files. These two files have been created for this specific formation flying
# implementation into Basilisk.
#
# ~~~~~~~~~~~~~{.py}
#         TheBSKSim.set_DynModel(BSK_FormationDynamics)
#         TheBSKSim.set_FswModel(BSK_FormationFsw)
# ~~~~~~~~~~~~~
#
# After initializing the interfaces and making sure that the scenario_BasicOrbitFormation class inherits from the BSKSim class,
# it is time to configure the initial conditions using the configure_initial_conditions method. It can be observed that two sets of
# orbital elements are created. Each set corresponding to one spacecraft. After that the initial conditions are set for each spacecraft.
#
# ~~~~~~~~~~~~~{.py}
#         # Configure Dynamics initial conditions
#         self.oe = orbitalMotion.ClassicElements()
#         self.oe.a = 10000000.0  # meters
#         self.oe.e = 0.01
#         self.oe.i = 33.3 * macros.D2R
#         self.oe.Omega = 48.2 * macros.D2R
#         self.oe.omega = 347.8 * macros.D2R
#         self.oe.f = 85.3 * macros.D2R
#         rN, vN = orbitalMotion.elem2rv(self.mu, self.oe)
#         orbitalMotion.rv2elem(self.mu, rN, vN)
#         self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#         self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#         self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
#         self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
#
#         # Configure Dynamics initial conditions
#         self.oe2 = orbitalMotion.ClassicElements()
#         self.oe2.a = 12000000.0  # meters
#         self.oe2.e = 0.1
#         self.oe2.i = 33.3 * macros.D2R
#         self.oe2.Omega = 48.2 * macros.D2R
#         self.oe2.omega = 347.8 * macros.D2R
#         self.oe2.f = 85.3 * macros.D2R
#         rN2, vN2 = orbitalMotion.elem2rv(self.mu, self.oe2)
#         orbitalMotion.rv2elem(self.mu, rN2, vN2)
#         self.masterSim.get_DynModel().scObject2.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN2)  # m   - r_CN_N
#         self.masterSim.get_DynModel().scObject2.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN2)  # m/s - v_CN_N
#         self.masterSim.get_DynModel().scObject2.hub.sigma_BNInit = [[-0.3], [0.0], [0.5]]  # sigma_BN_B
#         self.masterSim.get_DynModel().scObject2.hub.omega_BN_BInit = [[0.003], [-0.02], [0.01]]  # rad/s - omega_BN_B
# ~~~~~~~~~~~~~
#
# After that the function that logs the outputs can be observed. Again this looks very similar to the log_outputs method
# in the scenario_BasicOrbit file, however one discrepancy can be noticed. Looking at the code below it can be observed that
# two instances of the simpleNavObject are logged (simpleNavObject and simpleNavObject2 respectively). Each object corresponds
# two one of the spacecraft. The same is true for the FSW objects. More on this will be discussed later.
#
# ~~~~~~~~~~~~~{.py}
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputTransName, samplingTime)
#         self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject2.outputTransName, samplingTime)
# ~~~~~~~~~~~~~
#
# The same is true for the pull_outputs method. Also in this function, it can be observed that the outputs of two instances
# of a specific object are pulled.
#
# ~~~~~~~~~~~~~{.py}
#         r_BN_N_chief = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         r_BN_N_deputy = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject2.outputTransName + ".r_BN_N", range(3))
# ~~~~~~~~~~~~~
#
# BSK_FormationDynamics file description
# -----
# Looking at the BSK_FormationDynamics file, it can be observed that the dynamics process consists of two tasks named DynamicsTask
# and DynamicsTask2 respectively. These tasks are added to the dynamics process and to each task, an instance of a specific object
# is added. This can be observed in the code below.
#
# ~~~~~~~~~~~~~{.py}
#         # Assign initialized modules to tasks
#         SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
#         SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
#         SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 301)
#
#         SimBase.AddModelToTask(self.taskName2, self.scObject2, None, 201)
#         SimBase.AddModelToTask(self.taskName2, self.simpleNavObject2, None, 109)
#         SimBase.AddModelToTask(self.taskName2, self.rwStateEffector2, None, 301)
# ~~~~~~~~~~~~~
#
# The gravity body (Earth in this case) is created using the gravBodyFactory and is attached as a separate ofbject to
# each spacecraft as can be seen below.
#
# ~~~~~~~~~~~~~{.py}
#         # Create gravity body
#         self.gravFactory = simIncludeGravBody.gravBodyFactory()
#         planet = self.gravFactory.createEarth()
#         planet.isCentralBody = True          # ensure this is the central gravitational body
#         self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravFactory.gravBodies.values())
#         self.scObject2.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravFactory.gravBodies.values())
# ~~~~~~~~~~~~~
#
# After that each object is added to the corresponding task. Something that is very important is the message names.
# In case multiple spacecraft are implemented in Basilisk it is necessary to manually connect an output message of
# one module to the input of a different module. This can be seen in the module-initialization methods in the BSK_FormationDynamics.py file.
#
# BSK_FormationFsw file description
# -----
# The setup of the FSW file (BSK_FormationFSW.py) in case of formation flying is very similar to the setup of the dynamics file.
# Also in this case, an instance of each task is initialized that corresponds to one of the two spacecraft. Furthermore, it is
# necessary to manually set the input- and output message names for the FSW modules. In order to make this tutorial work properly its
# is very important to set the self.mrpFeedbackRWsData.Ki and self.mrpFeedbackRWsData2.Ki variables in BSK_FormationFsw to -1. Otherwise
# the orientation and rates of both spacecraft will not converge!
#
#
# Numerical Simulation Results
# ------------
# If this simulation is run the following plots should be shown.
# ![Chief Attitude Error](Images/Scenarios/scenario_BasicOrbitFormation_attitude_error_chief.svg "Attitude Error")
# ![Chief Rate Error](Images/Scenarios/scenario_BasicOrbitFormation_rate_error_chief.svg "Rate Error")
# ![Deputy Attitude Error](Images/Scenarios/scenario_BasicOrbitFormation_attitude_error_deputy.svg "Attitude Error")
# ![Deputy Rate Error](Images/Scenarios/scenario_BasicOrbitFormation_rate_error_deputy.svg "Rate Error")
# ![Inertial Orbits Illustration Deputy](Images/Scenarios/scenario_BasicOrbitFormation_orbits.svg "Position History")
#
# In order to create the orbits plot, two functions have been added to the BSK_Plotting file. One function that is able
# to plot a celestial body and a second function that is able to plot an orbit around the celestial body. In case of the
# formation flying tutorial two orbits are plotted around the celestial body as can be seen below.
#
# ~~~~~~~~~~~~~{.py}
#         BSK_plt.plot_planet(self.oe, self.masterSim.get_DynModel().gravFactory.gravBodies['earth'])
#         BSK_plt.plot_peri_and_orbit(self.oe, self.mu, r_BN_N_chief, v_BN_N_chief)
#         BSK_plt.plot_peri_and_orbit(self.oe2, self.mu, r_BN_N_deputy, v_BN_N_deputy)
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
import BSK_FormationDynamics, BSK_FormationFsw

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')


# Create your own scenario child class
class scenario_BasicOrbitFormation(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_BasicOrbitFormation, self).__init__(masterSim)
        self.name = 'scenario_BasicOrbitFormation'

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Configure FSW mode
        self.masterSim.modeRequest = 'inertial3D'

        self.mu = self.masterSim.get_DynModel().gravFactory.gravBodies['earth'].mu

        # Configure Dynamics initial conditions
        self.oe = orbitalMotion.ClassicElements()
        self.oe.a = 10000000.0  # meters
        self.oe.e = 0.01
        self.oe.i = 33.3 * macros.D2R
        self.oe.Omega = 48.2 * macros.D2R
        self.oe.omega = 347.8 * macros.D2R
        self.oe.f = 85.3 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(self.mu, self.oe)
        orbitalMotion.rv2elem(self.mu, rN, vN)
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

        # Configure Dynamics initial conditions
        self.oe2 = orbitalMotion.ClassicElements()
        self.oe2.a = 12000000.0  # meters
        self.oe2.e = 0.1
        self.oe2.i = 33.3 * macros.D2R
        self.oe2.Omega = 48.2 * macros.D2R
        self.oe2.omega = 347.8 * macros.D2R
        self.oe2.f = 85.3 * macros.D2R
        rN2, vN2 = orbitalMotion.elem2rv(self.mu, self.oe2)
        orbitalMotion.rv2elem(self.mu, rN2, vN2)
        self.masterSim.get_DynModel().scObject2.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN2)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject2.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN2)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject2.hub.sigma_BNInit = [[-0.3], [0.0], [0.5]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject2.hub.omega_BN_BInit = [[0.003], [-0.02], [0.01]]  # rad/s - omega_BN_B

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        samplingTime = self.masterSim.get_DynModel().processTasksTimeStep

        # Dynamics process outputs
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject.outputTransName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().simpleNavObject2.outputTransName, samplingTime)

        # FSW process outputs
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData2.outputDataName, samplingTime)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().scObject.scStateOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().scObject2.scStateOutMsgName, samplingTime)

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        # Dynamics process outputs
        r_BN_N_chief = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", list(range(3)))
        r_BN_N_deputy = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject2.outputTransName + ".r_BN_N", list(range(3)))

        v_BN_N_chief = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", list(range(3)))
        v_BN_N_deputy = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().simpleNavObject2.outputTransName + ".v_BN_N", list(range(3)))

        # FSW process outputs
        omega_BR_B_chief = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        omega_BR_B_deputy = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData2.outputDataName + ".omega_BR_B", list(range(3)))

        sigma_BR_chief = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", list(range(3)))
        sigma_BR_deputy = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().trackingErrorData2.outputDataName + ".sigma_BR", list(range(3)))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = sigma_BR_deputy[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR_chief)
        BSK_plt.plot_rate_error(timeData, omega_BR_B_chief)
        BSK_plt.plot_attitude_error(timeData, sigma_BR_deputy)
        BSK_plt.plot_rate_error(timeData, omega_BR_B_deputy)
        BSK_plt.plot_planet(self.oe, self.masterSim.get_DynModel().gravFactory.gravBodies['earth'])
        BSK_plt.plot_peri_and_orbit(self.oe, self.mu, r_BN_N_chief, v_BN_N_chief)
        BSK_plt.plot_peri_and_orbit(self.oe2, self.mu, r_BN_N_deputy, v_BN_N_deputy)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitude_error_chief", "rate_error_chief", "attitude_error_deputy",
                           "rate_error_deputy", "orbits"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    # Instantiate base simulation
    TheBSKSim = BSKSim()
    TheBSKSim.set_DynModel(BSK_FormationDynamics)
    TheBSKSim.set_FswModel(BSK_FormationFsw)
    TheBSKSim.initInterfaces()
    
    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbitFormation(TheBSKSim)
    
    TheScenario.configure_initial_conditions()
    TheScenario.log_outputs()

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
