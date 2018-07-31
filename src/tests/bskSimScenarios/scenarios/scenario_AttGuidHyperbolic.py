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
# Demonstrates how to use the velocity pointing model within the BSK_Sim architecture.
#
# BSK Simulation: Attitude Alignment for Hyperbolic Trajectory {#scenario_AttGuidHyperbolic}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting earth, using the MRP_Steering module with a rate sub-servo system
# to conrtrol the attitude all within the new BSK_Sim architecture.
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttGuidHyperbolic.py
#
# The simulation layout is shown in the following illustration.  Two simulation processes are created: one
# which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
# modules. The initial setup for the simulation closely models that of scenario_FeedbackRW.py.
#
# To begin, one must first create a class that will
# inherient from the masterSim class within the __init__() procedure and providing a name to the sim.
# This is accomplished through:
# ~~~~~~~~~~~~~{.py}
#   class scenario_AttGuidHyperbolic(BSKScenario):
#      def __init__(self, masterSim):
#          super(scenario_AttGuidHyperbolic, self).__init__(masterSim)
#          self.name = 'scenario_AttGuidHyperbolic'
# ~~~~~~~~~~~~~
#
# Following the inheritance, there are three functions within the scenario class that need to be configured by the user:
# configure_initial_conditions(), log_outputs(), and pull_outputs().
#
# Within configure_initial_conditions(), the user needs to first define the spacecraft FSW mode for the simulation
# through:
# ~~~~~~~~~~~~~{.py}
#   self.masterSim.modeRequest = "velocityPoint"
# ~~~~~~~~~~~~~
# which triggers the Hill Point event within the BSK_FSW.py script.
# ~~~~~~~~~~~~~
#
# The initial conditions for the scenario are set to establish a hyperbolic trajectory with initial tumbling:
# ~~~~~~~~~~~~~{.py}
#         # Configure Dynamics initial conditions
#         oe = orbitalMotion.ClassicElements()
#         oe.a = -150000.0 * 1000  # meters
#         oe.e = 1.5
#         oe.i = 33.3 * macros.D2R
#         oe.Omega = 48.2 * macros.D2R
#         oe.omega = 347.8 * macros.D2R
#         oe.f = 30 * macros.D2R
#         mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
#         rN, vN = orbitalMotion.elem2rv(mu, oe)
#         self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#         self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#         self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
#         self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
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
#
# Custom Dynamics Configurations Instructions
# -----
# The modules required for this scenario are identical to those used in scenario_AttGuidance.py.
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
# The only new module required to configure the "velocityPoint" FSW mode is `velocityPoint` itself:
# ~~~~~~~~~~~~~{.py}
#         self.velocityPointData = velocityPoint.velocityPointConfig()
#         self.velocityPointWrap = SimBase.setModelDataWrap(self.velocityPointData)
#         self.velocityPointWrap.ModelTag  = "velocityPoint"
# ~~~~~~~~~~~~~
# Unlike hill pointing, this modules provides a pointing model relative to the velocity vector.
#
# Within `InitAllFSWObjects()` a new setter functions `self.SetVelocityPointGuidance(SimBase)` is required, whose
# definition is:
# ~~~~~~~~~~~~~{.py}
#     def SetVelocityPointGuidance(self, SimBase):
#         self.velocityPointData.outputDataName = "referenceOut"
#         self.velocityPointData.inputNavDataName = SimBase.DynModels.simpleNavObject.outputTransName
#         self.velocityPointData.inputCelMessName = SimBase.DynModels.gravFactory.gravBodies['earth'].bodyInMsgName[:-12]
#         self.velocityPointData.mu = SimBase.DynModels.gravFactory.gravBodies['earth'].mu
# ~~~~~~~~~~~~~
# The only additions required in BSK_FSW.py are to create a new task specific for velocity pointing:
# ~~~~~~~~~~~~~{.py}
#         SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
# ~~~~~~~~~~~~~
# and then to add the tracking model to the velocity point task through:
# The modules need to be attached to the the tasks through:
# ~~~~~~~~~~~~~{.py}
#         SimBase.AddModelToTask("velocityPointTask", self.velocityPointWrap, self.velocityPointData, 10)
#         SimBase.AddModelToTask("velocityPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
#
# ~~~~~~~~~~~~~
# Finally, the `velocityPoint` mode needs to be defined by enabling the two needed tasks:
# ~~~~~~~~~~~~~{.py}
#              SimBase.createNewEvent("initiateVelocityPoint", self.processTasksTimeStep, True,
#                                ["self.modeRequest == 'velocityPoint'"],
#                                ["self.fswProc.disableAllTasks()",
#                                 "self.enableTask('velocityPointTask')",
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
import scenarioAttGuideHyperbolic as scene_plt


# Create your own scenario child class
class scenario_VelocityPointing(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_VelocityPointing, self).__init__(masterSim)
        self.name = 'scenario_VelocityPointing'

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'velocityPoint'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = -150000.0 * 1000  # meters
        oe.e = 1.5
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 30 * macros.D2R
        mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

        # Safe orbit elements for postprocessing
        self.oe = oe


    def log_outputs(self):
        print '%s: log_outputs' % self.name
        # Dynamics process outputs
        samplingTime = self.masterSim.DynModels.processTasksTimeStep
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputAttName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.simpleNavObject.outputTransName, samplingTime)

        # FSW process outputs
        samplingTime = self.masterSim.FSWModels.processTasksTimeStep
        #self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.velocityPointData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName, samplingTime)

    def pull_outputs(self, showPlots):
        print '%s: pull_outputs' % self.name
        # Dynamics process outputs
        #sigma_BN = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputAttName + ".sigma_BN", range(3))
        #omega_BN_B = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputAttName + ".omega_BN_B", range(3))
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".r_BN_N", range(3))
        v_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.simpleNavObject.outputTransName + ".v_BN_N", range(3))

        # FSW process outputs
        #sigma_RN = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".sigma_RN", range(3))
        #omega_RN_N = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.inputRefName + ".omega_RN_N", range(3))
        sigma_BR = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        Lr = self.masterSim.pullMessageLogData(self.masterSim.FSWModels.mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))

        # Plot results
        timeLineSet = sigma_BR[:, 0] * macros.NANO2MIN
        scene_plt.plot_track_error_norm(timeLineSet, sigma_BR)
        scene_plt.plot_control_torque(timeLineSet, Lr)
        scene_plt.plot_rate_error(timeLineSet, omega_BR_B)
        scene_plt.plot_orbit(self.oe,
                             self.masterSim.DynModels.earthGravBody.mu,
                             self.masterSim.DynModels.earthGravBody.radEquator,
                             r_BN_N, v_BN_N)
        #BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)
        #BSK_plt.plot_rotationalNav(sigma_BN, omega_BN_B)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "orbit"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList
def run(showPlots):

    # Instantiate base simulation
    TheBSKSim = BSKSim()

    # Configure a scenario in the base simulation
    TheScenario = scenario_VelocityPointing(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print 'BSKSim: Starting Execution'
    TheBSKSim.ExecuteSimulation()
    print 'BSKSim: Finished Execution. Post-processing results'

    # Pull the results of the base simulation running the chosen scenario
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
