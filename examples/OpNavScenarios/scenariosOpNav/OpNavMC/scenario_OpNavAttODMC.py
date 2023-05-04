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
# Get current file path
import inspect
import os
import subprocess
import sys

# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../..')
from BSK_OpNav import BSKSim
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np

# Import plotting file for your scenario
sys.path.append(path + '/../../plottingOpNav')


# Create your own scenario child class
class scenario_OpNav(BSKSim):
    """Main Simulation Class"""
    def __init__(self):
        super(scenario_OpNav, self).__init__(BSKSim)
        self.fswRate = 0.5
        self.dynRate = 0.5
        self.set_DynModel(BSK_OpNavDynamics)
        self.set_FswModel(BSK_OpNavFsw)
        self.name = 'scenario_opnav'
        self.configure_initial_conditions()

        self.msgRecList = {}
        self.retainedMessageNameSc = "scMsg"
        self.retainedMessageNameFilt = "filtMsg"
        self.retainedMessageNameOpNav = "opnavMsg"

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000 * 1E3  # meters
        oe.e = 0.6
        oe.i = 10 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 80. * macros.D2R  # 90 good
        mu = self.get_DynModel().gravFactory.gravBodies['mars barycenter'].mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]
        rError= np.array([10000.,10000., -10000])
        vError= np.array([100, -10, 10])
        MRP= [0,0,0]
        self.get_FswModel().relativeODData.stateInit = (rN + rError).tolist() + (vN + vError).tolist()

        self.get_DynModel().scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.get_FswModel().relativeODData.qNoise = qNoiseIn.reshape(36).tolist()
        self.get_FswModel().imageProcessing.noiseSF = 1
        self.get_FswModel().relativeODData.noiseSF = 5#7.5

    def log_outputs(self):
        # Dynamics process outputs: log messages below if desired.
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()

        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep

        self.msgRecList[self.retainedMessageNameSc] = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageNameSc])

        self.msgRecList[self.retainedMessageNameFilt] = FswModel.relativeODData.filtDataOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageNameFilt])

        self.msgRecList[self.retainedMessageNameOpNav] = FswModel.opnavMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageNameOpNav])

        return


def run(TheScenario):

    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheScenario.get_FswModel().imageProcessing.saveImages = 0
    TheScenario.get_DynModel().vizInterface.opNavMode = 1

    mode = ["None", "-directComm", "-noDisplay"]
    vizard = subprocess.Popen(
        [TheScenario.vizPath, "--args", mode[TheScenario.get_DynModel().vizInterface.opNavMode], "tcp://localhost:5556"], stdout=subprocess.DEVNULL)
    print("Vizard spawned with PID = " + str(vizard.pid))

    # Configure FSW mode
    TheScenario.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheScenario.InitializeSimulation()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(3.)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()
    TheScenario.modeRequest = 'OpNavAttOD'
    # TheBSKSim.get_DynModel().SetLocalConfigData(TheBSKSim, 60, True)
    simulationTime = macros.min2nano(100.)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()

    vizard.kill()

    spice = TheScenario.get_DynModel().gravFactory.spiceObject
    spice.unloadSpiceKernel(spice.SPICEDataPath, 'de430.bsp')
    spice.unloadSpiceKernel(spice.SPICEDataPath, 'naif0012.tls')
    spice.unloadSpiceKernel(spice.SPICEDataPath, 'de-403-masses.tpc')
    spice.unloadSpiceKernel(spice.SPICEDataPath, 'pck00010.tpc')

    return

if __name__ == "__main__":
    # Instantiate base simulation

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav()
    run(TheScenario)
