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

This script is called by OpNavScenarios/CNN_ImageGen/OpNavMonteCarlo.py in order to generate images.

"""
# Get current file path
import inspect
import os
import subprocess
import sys

from Basilisk.utilities import RigidBodyKinematics as rbk
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../..')
from BSK_OpNav import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np

# Import plotting file for your scenario
sys.path.append(path + '/../../plottingOpNav')
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
        self.name = 'scenario_opnav'
        self.filterUse = "bias" #"relOD"
        self.configure_initial_conditions()

        # set recorded message information
        self.msgRecList = {}
        self.retainedMessageName1 = "scMsg"
        self.retainedMessageName2 = "circlesMsg"
        self.var1 = "r_BN_N"
        self.var2 = "sigma_BN"
        self.var3 = "valid"

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000*1E3 # meters
        oe.e = 0.
        oe.i = 20 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 100. * macros.D2R #90 good
        mu = self.get_DynModel().gravFactory.gravBodies['mars barycenter'].mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,0,0]
        if self.filterUse =="relOD":
            self.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        if self.filterUse == "bias":
            self.get_FswModel().pixelLineFilterData.stateInit = rN.tolist() + vN.tolist() + bias
        # self.get_DynModel().scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        # self.get_DynModel().scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        self.get_DynModel().cameraMod.fieldOfView = np.deg2rad(55)

    def log_outputs(self):
        # Dynamics process outputs: log messages below if desired.
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep

        self.msgRecList[self.retainedMessageName1] = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageName1])

        self.msgRecList[self.retainedMessageName2] = FswModel.opnavCirclesMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageName2])

        return

    def pull_outputs(self, showPlots):
        ## Spacecraft true states
        scStates = self.msgRecList[self.retainedMessageName1]
        position_N = unitTestSupport.addTimeColumn(scStates.times(), scStates.r_BN_N)
        sigma_BN = unitTestSupport.addTimeColumn(scStates.times(), scStates.sigma_BN)

        ## Image processing
        circleStates = self.scRecmsgRecList[self.retainedMessageName2]
        validCircle = unitTestSupport.addTimeColumn(circleStates.times(), circleStates.valid)

        sigma_CB = self.get_DynModel().cameraMRP_CB
        sizeMM = self.get_DynModel().cameraSize
        sizeOfCam = self.get_DynModel().cameraRez
        focal = self.get_DynModel().cameraFocal #in m

        pixelSize = []
        pixelSize.append(sizeMM[0] / sizeOfCam[0])
        pixelSize.append(sizeMM[1] / sizeOfCam[1])

        dcm_CB = rbk.MRP2C(sigma_CB)
        # Plot results
        BSK_plt.clear_all_plots()

        trueRhat_C = np.full([len(validCircle[:,0]), 4], np.nan)
        trueCircles = np.full([len(validCircle[:,0]), 4], np.nan)
        trueCircles[:,0] = validCircle[:,0]
        trueRhat_C[:,0] = validCircle[:,0]

        ModeIdx = 0
        Rmars = 3396.19*1E3
        for j in range(len(position_N[:, 0])):
            if position_N[j, 0] in validCircle[:, 0]:
                ModeIdx = j
                break
        for i in range(len(validCircle[:,0])):
            if validCircle[i,1] > 1E-5:
                trueRhat_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[ModeIdx+i , 1:4])) ,position_N[ModeIdx+i, 1:4])/np.linalg.norm(position_N[ModeIdx+i, 1:4])
                trueCircles[i,3] = focal*np.tan(np.arcsin(Rmars/np.linalg.norm(position_N[ModeIdx+i,1:4])))/pixelSize[0]
                trueRhat_C[i,1:] *= focal/trueRhat_C[i,3]
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0]/2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1]/2 - 0.5

        return


def run(TheScenario, runLog):
    TheBskScenario = BSKScenario
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    if not os.path.exists(runLog):
        os.makedirs(runLog)

    TheScenario.get_DynModel().cameraMod.fieldOfView = np.deg2rad(55)  # in degrees
    TheScenario.get_DynModel().cameraMod.cameraIsOn = 1
    TheScenario.get_DynModel().cameraMod.saveImages = 1
    TheScenario.get_DynModel().cameraMod.saveDir = runLog.split('/')[-2] +'/' +runLog.split('/')[-1] + '/'
    TheScenario.get_DynModel().vizInterface.opNavMode = 2

    mode = ["None", "-directComm", "-noDisplay"]
    # The following code spawns the Vizard application from python as a function of the mode selected above, and the platform.
    TheScenario.vizard = subprocess.Popen(
        [TheScenario.vizPath, "--args", mode[TheScenario.get_DynModel().vizInterface.opNavMode], "tcp://localhost:5556"], stdout=subprocess.DEVNULL)
    print("Vizard spawned with PID = " + str(TheScenario.vizard.pid))

    # Configure FSW mode
    TheScenario.modeRequest = 'imageGen'
    # Initialize simulation
    TheScenario.InitializeSimulation()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(100.)
    TheScenario.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    TheScenario.ExecuteSimulation()

    TheScenario.vizard.kill()

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
    run(TheScenario, os.path.abspath(os.path.dirname(__file__)) + "/cnn_MC_data")
