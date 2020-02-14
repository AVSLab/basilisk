#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
r"""
Overview
--------

This script is called by OpNavScenarios/OpNavMC/MonteCarlo.py in order to make MC data.

"""
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


# Get current file path
import sys, os, inspect, time, signal, subprocess
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../..')
from BSK_OpNav import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np


# Import plotting file for your scenario
sys.path.append(path + '/../../plotting')
import OpNav_Plotting as BSK_plt

# Create your own scenario child class
class scenario_OpNav(BSKSim):
    """Main Simulation Class"""
    def __init__(self):
        super(scenario_OpNav, self).__init__(BSKSim)
        self.fswRate = 0.5
        self.dynRate = 0.5
        self.set_DynModel(BSK_OpNavDynamics)
        self.set_FswModel(BSK_OpNavFsw)
        self.initInterfaces()
        self.name = 'scenario_opnav'
        self.configure_initial_conditions()

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000 * 1E3  # meters
        oe.e = 0.6
        oe.i = 10 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 80. * macros.D2R  # 90 good
        mu = self.get_DynModel().marsGravBody.mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]
        rError= np.array([10000.,10000., -10000])
        vError= np.array([100, -10, 10])
        MRP= [0,0,0]
        self.get_FswModel().relativeODData.stateInit = (rN + rError).tolist() + (vN + vError).tolist()

        self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.get_FswModel().relativeODData.qNoise = qNoiseIn.reshape(36).tolist()
        self.get_FswModel().imageProcessing.noiseSF = 1
        self.get_FswModel().relativeODData.noiseSF = 5#7.5

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep
        # self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorCamData.outputDataName, samplingTime)
        # self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)

        self.TotalSim.logThisMessage(self.get_FswModel().relativeODData.filtDataOutMsgName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().pixelLineData.opNavOutMsgName, samplingTime)

        self.TotalSim.logThisMessage(self.get_DynModel().scObject.scStateOutMsgName,samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().imageProcessing.opnavCirclesOutMsgName, samplingTime)
        return


def run(TheScenario):

    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheScenario.get_FswModel().imageProcessing.saveImages = 0
    TheScenario.get_DynModel().vizInterface.opNavMode = 2


    # Configure FSW mode
    TheScenario.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheScenario.InitializeSimulationAndDiscover()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(3.)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()
    TheScenario.modeRequest = 'OpNavAttOD'
    # TheBSKSim.get_DynModel().SetLocalConfigData(TheBSKSim, 60, True)
    simulationTime = macros.min2nano(600.)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()

    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'de430.bsp')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'naif0012.tls')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath,
                                                      'de-403-masses.tpc')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'pck00010.tpc')

    return

if __name__ == "__main__":
    # Instantiate base simulation

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav()
    run(TheScenario)
