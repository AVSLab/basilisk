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

## \page scenario_RelativePointingFormationGroup
## @{
# Demonstrates how to make use of the spacecraftPointing module to create a tutorial that simulates a deputy spacecraft
# that points to a chief spacecraft
#
# BSK Simulation: Relative Pointing Formation {#scenario_RelativePointingFormation}
# ====
#
# Scenario Description
# -----
# This script sets up a deputy that points to a chief spacecraft. The goal of the scenario is to
# 1) How to make sure that a deputy spacecraft is able to read data from a chief spacecraft.
# 2) How to implement a module that combines data from two spacecraft into a scenario.
#
# To run the default scenario, call the bskSim python simulation script within `src\tests\bskSimScenarios\scenarios`
# from a Terminal window through:
#
#       python3 scenario_RelativePointingFormation.py
#
# The simulation mimics the basic simulation in the earlier tutorial in
# [scenario_BasicOrbitFormation.py](@ref scenario_BasicOrbitFormation).
#
# The flight software mode is set to spacecraftPointing. The goal of this mode is to align a vector given in the
# deputy's body-frame with a vector that points from the deputy to the chief spacecraft.
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenario_RelativePointingFormation.svg "Simulation Flow Diagram")
#
# In the simulation flow diagram it can be observed that the deputy spacecraft reads the data from the chief spacecraft's
# simpleNavObject. This data is consequently used to calculate the attitude of the reference frame with respect to
# the inertial reference frame. Together with the attitude of the body frame of the deputy spacecraft, this data is fed
# into the attitudeError module. In this module, the attitude error is calculated and fed into the MRP feedback and
# torque module to make sure that the deputy's attitude will match with the attitude of the reference frame.
#
# Configuring the scenario file
# -----
# The simulation layout is almost the same as the one used for the scenario_BasicOrbitFormation file.
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
# orbital elements are created. Each set corresponding to one spacecraft. From the true anomaly of both spacecraft,
# it can be deduced that this scenario makes use of a leader-follower formation. However, the orbital elements can be changed to
# whatever the user prefers. After the orbital elements are initialized the initial conditions are set for each spacecraft.
#
# ~~~~~~~~~~~~~{.py}
#         # Configure Dynamics initial conditions
#         oe = orbitalMotion.ClassicElements()
#         oe.a = 7000000.0  # meters
#         oe.e = 0.1
#         oe.i = 33.3 * macros.D2R
#         oe.Omega = 48.2 * macros.D2R
#         oe.omega = 347.8 * macros.D2R
#         oe.f = 0.1 * macros.D2R
#         rN, vN = orbitalMotion.elem2rv(mu, oe)
#         orbitalMotion.rv2elem(mu, rN, vN)
#         self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#         self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#         self.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]] # sigma_BN_B
#         self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
#
#         # Configure Dynamics initial conditions
#         oe2 = orbitalMotion.ClassicElements()
#         oe2.a = 7000000.0  # meters
#         oe2.e = 0.1
#         oe2.i = 33.3 * macros.D2R
#         oe2.Omega = 48.2 * macros.D2R
#         oe2.omega = 347.8 * macros.D2R
#         oe2.f = 0.0 * macros.D2R
#         rN2, vN2 = orbitalMotion.elem2rv(mu, oe2)
#         orbitalMotion.rv2elem(mu, rN2, vN2)
#         self.get_DynModel().scObject2.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN2)  # m   - r_CN_N
#         self.get_DynModel().scObject2.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN2)  # m/s - v_CN_N
#         self.get_DynModel().scObject2.hub.sigma_BNInit = [[-0.3], [0.0], [0.5]]  # sigma_BN_B
#         self.get_DynModel().scObject2.hub.omega_BN_BInit = [[0.003], [-0.02], [0.01]]  # rad/s - omega_BN_B
# ~~~~~~~~~~~~~
#
# After that the function that logs the outputs can be observed. Again this looks very similar to the log_outputs function
# in the scenario_BasicOrbit file, however one discrepancy can be noticed. Looking at the code below it can be observed that
# two instances of the simpleNavObject are logged (simpleNavObject and simpleNavObject2 respectively). Each object corresponds
# two one of the spacecraft. The output of the new module is also logged, as can be seen in the beforelast logging statement below.
# In this statement the user asks to log the output message of the spacecraftPointing module.
#
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputTransName, samplingTime)
#         self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject2.outputTransName, samplingTime)
#
#         self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputAttName, samplingTime)
#         self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject2.outputAttName, samplingTime)
#
#         # # FSW process outputs
#         self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)
#         self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData2.outputDataName, samplingTime)
#
#         self.TotalSim.logThisMessage(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName, samplingTime)
#
#         self.TotalSim.logThisMessage(self.get_FswModel().mrpFeedbackControlData.outputDataName, samplingTime)
# ~~~~~~~~~~~~~
#
# The same yields for the pull_outputs function. Also in this function, it can be observed that the outputs of two instances
# of a specific object can be pulled.
#
# ~~~~~~~~~~~~~{.py}
#         # Dynamics process outputs
#         r_BN_N_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", range(3))
#         r_BN_N_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputTransName + ".r_BN_N", range(3))
#
#         v_BN_N_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", range(3))
#         v_BN_N_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputTransName + ".v_BN_N", range(3))
#
#         sigma_BN_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", range(3))
#         sigma_BN_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputAttName + ".sigma_BN", range(3))
#
#         # FSW process outputs
#         omega_BR_B_chief = self.pullMessageLogData(self.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", range(3))
#         omega_BR_B_deputy = self.pullMessageLogData(self.get_FswModel().trackingErrorData2.outputDataName + ".omega_BR_B", range(3))
#
#         sigma_BR_deputy = self.pullMessageLogData(self.get_FswModel().trackingErrorData2.outputDataName + ".sigma_BR", range(3))
#
#         sigma_RN = self.pullMessageLogData(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName + ".sigma_RN", range(3))
#         omega_RN_N = self.pullMessageLogData(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName + ".omega_RN_N", range(3)))
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
# necessary to manually set the input- and output message names for the FSW modules.
#
#
# Numerical Simulation Results
# ------------
# If this simulation is run for 200 minutes the following plots should be shown.
# ![Attitude Error Norm](Images/Scenarios/scenario_RelativePointingFormation_attitude_error.svg "Attitude Error Norm")
# ![Relative Orbit Components](Images/Scenarios/scenario_RelativePointingFormation_relative_orbit.svg "Relative Orbits Components")
# ![MRP Reference Frame with respect to Inertial Frame](Images/Scenarios/scenario_RelativePointingFormation_sigma_RN.svg
#  "MRP Reference Frame with respect to Inertial Frame")
# ![MRP Body Frame with respect to Inertial Frame](Images/Scenarios/scenario_RelativePointingFormation_sigma_BN_deputy.svg
#  "MRP Body Frame with respect to Inertial Frame")
# ![MRP Body Frame with respect to Reference Frame](Images/Scenarios/scenario_RelativePointingFormation_sigma_BR_deputy.svg
#  "MRP Body Frame with respect to Reference Frame")
#
# In order to create the relative orbits plot, a function has been added to the BSK_Plotting file that takes the position
# data of both spacecraft and plots the relative orbit.
# ~~~~~~~~~~~~~{.py}
#         BSK_plt.plot_rel_orbit(timeData, r_BN_N_chief, r_BN_N_deputy)
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
import matplotlib.pyplot as plt

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')

# Create your own scenario child class
class scenario_RelativePointingFormation(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_RelativePointingFormation, self).__init__()
        self.name = 'scenario_RelativePointingFormation'

        self.set_DynModel(BSK_FormationDynamics)
        self.set_FswModel(BSK_FormationFsw)
        self.initInterfaces()

        self.configure_initial_conditions()
        self.log_outputs()

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Configure FSW mode
        self.modeRequest = 'spacecraftPointing'

        mu = self.get_DynModel().gravFactory.gravBodies['earth'].mu

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
        oe.e = 0.1
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 0.1 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]] # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

        # Configure Dynamics initial conditions
        oe2 = orbitalMotion.ClassicElements()
        oe2.a = 7000000.0  # meters
        oe2.e = 0.1
        oe2.i = 33.3 * macros.D2R
        oe2.Omega = 48.2 * macros.D2R
        oe2.omega = 347.8 * macros.D2R
        oe2.f = 0.0 * macros.D2R
        rN2, vN2 = orbitalMotion.elem2rv(mu, oe2)
        orbitalMotion.rv2elem(mu, rN2, vN2)
        self.get_DynModel().scObject2.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN2)  # m   - r_CN_N
        self.get_DynModel().scObject2.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN2)  # m/s - v_CN_N
        self.get_DynModel().scObject2.hub.sigma_BNInit = [[-0.3], [0.0], [0.5]]  # sigma_BN_B
        self.get_DynModel().scObject2.hub.omega_BN_BInit = [[0.003], [-0.02], [0.01]]  # rad/s - omega_BN_B

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        samplingTime = self.get_DynModel().processTasksTimeStep

        # Dynamics process outputs
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputTransName, samplingTime)
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject2.outputTransName, samplingTime)

        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputAttName, samplingTime)
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject2.outputAttName, samplingTime)

        # # FSW process outputs
        self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData2.outputDataName, samplingTime)

        self.TotalSim.logThisMessage(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName, samplingTime)

        self.TotalSim.logThisMessage(self.get_FswModel().mrpFeedbackControlData.outputDataName, samplingTime)

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        # Dynamics process outputs
        r_BN_N_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", list(range(3)))
        r_BN_N_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputTransName + ".r_BN_N", list(range(3)))

        v_BN_N_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", list(range(3)))
        v_BN_N_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputTransName + ".v_BN_N", list(range(3)))

        sigma_BN_chief = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", list(range(3)))
        sigma_BN_deputy = self.pullMessageLogData(self.get_DynModel().simpleNavObject2.outputAttName + ".sigma_BN", list(range(3)))

        # FSW process outputs
        omega_BR_B_chief = self.pullMessageLogData(self.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        omega_BR_B_deputy = self.pullMessageLogData(self.get_FswModel().trackingErrorData2.outputDataName + ".omega_BR_B", list(range(3)))

        sigma_BR_deputy = self.pullMessageLogData(self.get_FswModel().trackingErrorData2.outputDataName + ".sigma_BR", list(range(3)))

        sigma_RN = self.pullMessageLogData(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName + ".sigma_RN", list(range(3)))
        omega_RN_N = self.pullMessageLogData(self.get_FswModel().spacecraftPointing.attReferenceOutMsgName + ".omega_RN_N", list(range(3)))


        # Plot results
        BSK_plt.clear_all_plots()
        timeData = sigma_BR_deputy[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR_deputy)
        BSK_plt.plot_rel_orbit(timeData, r_BN_N_chief, r_BN_N_deputy)
        BSK_plt.plot_sigma(sigma_RN)
        BSK_plt.plot_sigma(sigma_BN_deputy)
        BSK_plt.plot_sigma(sigma_BR_deputy)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitude_error", "relative_orbit", "sigma_RN",
                           "sigma_BN_deputy", "sigma_BR_deputy"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList

def runScenario(scenario):
    # Initialize simulation
    scenario.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.0)
    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()

def run(showPlots):

    TheScenario = scenario_RelativePointingFormation()
    runScenario(TheScenario)

    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
