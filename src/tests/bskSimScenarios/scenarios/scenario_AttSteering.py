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
import test_scenarioAttitudeSteering as scene_plt


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
        mu = self.masterSim.DynModels.earthGravBody.mu
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

    def pull_outputs(self):
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
        BSK_plt.show_all_plots()



if __name__ == "__main__":
    # Instantiate base simulation
    TheBSKSim = BSKSim()

    # Configure an scenario in the base simulation
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
    TheScenario.pull_outputs()
