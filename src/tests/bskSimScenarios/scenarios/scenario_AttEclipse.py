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

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')
import scenarioAttitudeFeedbackRW as scene_plt

# Create your own scenario child class
class scenario_AttitudeEclipse(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_AttitudeEclipse, self).__init__(masterSim)
        self.name = 'scenario_AttitudeEclipse'
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'sunSafePoint'#'feedbackRW'

        # Configure Dynamics initial conditions

        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
        oe.e = 0.0
        oe.i = 0.0 #33.3 * macros.D2R
        oe.Omega = 0.0# 48.2 * macros.D2R
        oe.omega = 0.0# 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.masterSim.DynModels.earthGravBody.mu #self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu #
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

        '''
        #from eclipse unit test
        oe = orbitalMotion.ClassicElements()
        r_0 = 500 + orbitalMotion.REQ_EARTH  # km
        oe.a = r_0
        oe.e = 0.00000
        oe.i = 0.0#5.0 * macros.D2R
        oe.Omega = 0.0#48.2 * macros.D2R
        oe.omega = 0 * macros.D2R
        oe.f = 85.3 * macros.D2R    #173 * macros.D2R
        mu = self.masterSim.DynModels.earthGravBody.mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = rN #* 1000 #unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = vN #* 1000 #unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
        '''



    def log_outputs(self):
        print '%s: log_outputs' % self.name
        samplingTime = self.masterSim.DynModels.processTasksTimeStep

        # Dynamics process outputs: log messages below if desired.
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.scObject.scStateOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage("eclipse_data_0", samplingTime)

        # FSW process outputs
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.rwMotorTorqueData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.sunSafePointData.sunDirectionInMsgName, samplingTime)
        return

    def pull_outputs(self):
        print '%s: pull_outputs' % self.name
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.scObject.scStateOutMsgName + ".r_BN_N", range(3))
        shadowFactor = self.masterSim.pullMessageLogData("eclipse_data_0.shadowFactor")

        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        RW_speeds = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", range(num_RW))
        sunPoint = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.sunSafePointData.sunDirectionInMsgName + ".vehSunPntBdy", range(3))

        # Plot results
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        #scene_plt.plot_attitude_error(timeData, sigma_BR)
        #scene_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        #scene_plt.plot_rate_error(timeData, omega_BR_B)
        #scene_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)

        BSK_plt.plot_shadow_factor(timeData, shadowFactor)
        BSK_plt.plot_orbit(r_BN_N)
        BSK_plt.plot_sun_point(timeData, sunPoint)
        #BSK_plt.plot3components(r_BN_N)
        BSK_plt.show_all_plots()



def run(showPlots):
    # Instantiate base simulation

    '''
    CURRENT STATE, THE SPICE DATA MODULE DOES WELL BUT HAS NO ECLIPSE FUNCTIONALITY

    THE SPICE OBJECT ALSO RUNS, BUT THERE IS NO SPACECRAFT ORBIT ANY LONGER SO...THAT NEEDS TO BE FIXED
    '''
    TheBSKSim = BSKSim()

    # Configure an scenario in the base simulation
    TheScenario = scenario_AttitudeEclipse(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(60.0* 4.0)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print 'Starting Execution'
    TheBSKSim.ExecuteSimulation()
    print 'Finished Execution. Post-processing results'

    # Pull the results of the base simulation running the chosen scenario
    if showPlots:
        TheScenario.pull_outputs()


if __name__ == "__main__":
    run(True)
