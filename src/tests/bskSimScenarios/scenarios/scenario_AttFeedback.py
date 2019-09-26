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

from Basilisk.utilities import orbitalMotion, macros, unitTestSupport, vizSupport


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
class scenario_AttFeedback(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_AttFeedback, self).__init__()
        self.name = 'scenario_AttFeedbackMC'

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)
        self.initInterfaces()

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        # vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynamicsProcessName,
        #                                     gravBodies=self.DynModels.gravFactory,
        #                                     saveFile=filename,
        #                                     numRW=self.DynModels.rwFactory.getNumOfDevices())

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R

        mu = self.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep
        self.TotalSim.logThisMessage(self.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        return


    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any

        # FSW process outputs
        # dataUsReq = self.pullMessageLogData(
        #     self.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", list(range(num_RW)))
        sigma_BR = self.pullMessageLogData(
            self.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.pullMessageLogData(
            self.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        # RW_speeds = self.pullMessageLogData(
        #     self.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", list(range(num_RW)))

        # Plot results
        #BSK_plt.clear_all_plots()
        timeData = sigma_BR[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        #BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        #BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(TheScenario):
    print('Starting Execution')
    simulationTime = macros.min2nano(10.)

    TheScenario.InitializeSimulationAndDiscover()
    TheScenario.modeRequest = 'inertial3D'
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()
    return



def run():
    scenario = scenario_AttFeedback()
    runScenario(scenario)
    scenario.pull_outputs(True)

if __name__ == "__main__":
    run()
