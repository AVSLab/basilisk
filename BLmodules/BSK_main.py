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

from Basilisk.utilities import SimulationBaseClass, orbitalMotion, macros, unitTestSupport
from Basilisk.simulation import sim_model

import BSK_Dynamics
import BSK_Fsw
import BSK_Plotting as BSK_plt


class BSKSim(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        # Create simulation process names
        self.DynamicsProcessName = "DynamicsProcess"
        self.FSWProcessName = "FSWProcess"

        # Create processes
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)

        # Define process message interfaces.
        self.dyn2FSWInterface = sim_model.SysInterface()
        self.fsw2DynInterface = sim_model.SysInterface()

        # Crate Dynamics and FSW classes
        self.DynModels = BSK_Dynamics.BSKDynamicModels(self)
        self.FSWModels = BSK_Fsw.BSKFswModels(self)

        # Discover interfaces between processes
        self.dyn2FSWInterface.addNewInterface(self.DynamicsProcessName, self.FSWProcessName)
        self.fsw2DynInterface.addNewInterface(self.FSWProcessName, self.DynamicsProcessName)
        self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
        self.fswProc.addInterfaceRef(self.fsw2DynInterface)

    def configure_initial_conditions(self):
        self.modeRequest = 'inertial3D'

        oe = orbitalMotion.ClassicElements()
        oe.a = 10000.0 * 1000.0  # meters
        oe.e = 0.2
        oe.i = 0.0 * macros.D2R
        oe.Omega = 0.0 * macros.D2R
        oe.omega = 0.0 * macros.D2R
        oe.f = 280.0 * macros.D2R

        mu = self.DynModels.earthGravBody.mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    def log_outputs(self):
        samplingTime = self.DynModels.processTasksTimeStep
        self.TotalSim.logThisMessage(self.DynModels.scObject.scStateOutMsgName, samplingTime)
        self.TotalSim.logThisMessage(self.DynModels.simpleNavObject.outputAttName, samplingTime)

        samplingTime = self.FSWModels.processTasksTimeStep
        self.TotalSim.logThisMessage(self.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.FSWModels.mrpFeedbackData.outputDataName, samplingTime)

    def pull_outputs(self):
        def print_dyn_outputs(sigma_BN, omega_BN_B):
            print 'sigma_BN = %s \n' % sigma_BN[-3:, 1:]
            print 'omega_BN_B = %s \n' % omega_BN_B[-3:, 1:]
            print 't_sim_end = %s \n' % sigma_BN[-1:, 0]
        def plot_dyn_outputs(sigma_BN, omega_BN_B):
            print "Plotting results."
            BSK_plt.plot_rotationalNav(sigma_BN, omega_BN_B)

        sigma_BN = self.pullMessageLogData(self.DynModels.simpleNavObject.outputAttName + ".sigma_BN", range(3))
        omega_BN_B = self.pullMessageLogData(self.DynModels.simpleNavObject.outputAttName + ".omega_BN_B", range(3))
        print_dyn_outputs(sigma_BN, omega_BN_B)
        plot_dyn_outputs(sigma_BN, omega_BN_B)

        def print_fsw_outputs(sigma_RN, sigma_BR, Lr):
            print 'sigma_RN = %s' % sigma_RN[-3:, 1:]
            print 'sigma_BR = %s' % sigma_BR[-3:, 1:]
            print 'Lr = %s' % Lr[-3:, 1:]
            print 't_sim_end = %s' % sigma_RN[-1:, 0]
        def plot_fsw_outputs(sigma_RN, sigma_BR, Lr):
            BSK_plt.plot_trackingError(sigma_BR, omega_BR_B)
            BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)
            BSK_plt.plot_controlTorque(Lr)

        sigma_RN = self.pullMessageLogData(self.FSWModels.trackingErrorData.inputRefName + ".sigma_RN", range(3))
        omega_RN_N = self.pullMessageLogData(self.FSWModels.trackingErrorData.inputRefName + ".omega_RN_N", range(3))
        sigma_BR = self.pullMessageLogData(self.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.pullMessageLogData(self.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        Lr = self.pullMessageLogData(self.FSWModels.mrpFeedbackData.outputDataName + ".torqueRequestBody", range(3))
        print_fsw_outputs(sigma_RN, sigma_BR, Lr)
        plot_fsw_outputs(sigma_RN, sigma_BR, Lr)



if __name__ == "__main__":
    TheBSKSim = BSKSim()

    TheBSKSim.log_outputs()
    TheBSKSim.configure_initial_conditions()
    TheBSKSim.InitializeSimulation()
    TheBSKSim.dyn2FSWInterface.discoverAllMessages()
    TheBSKSim.fsw2DynInterface.discoverAllMessages()

    simulationTime = macros.min2nano(10.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print 'Starting Execution'
    TheBSKSim.ExecuteSimulation()
    print 'Finished Execution. Post-processing results'
    TheBSKSim.pull_outputs()
    BSK_plt.show_all_plots()



