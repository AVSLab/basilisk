''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

    def pull_outputs(self):
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
        BSK_plt.show_all_plots()

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
    figureList = {}
    if showPlots:
        figureList = TheScenario.pull_outputs()

    return figureList

if __name__ == "__main__":
    run(True)
