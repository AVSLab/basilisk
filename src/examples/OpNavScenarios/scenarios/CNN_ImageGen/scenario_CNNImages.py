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
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


# Get current file path
import sys, os, inspect, time, subprocess, signal
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../..')
from BSK_OpNav import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np
from sys import platform

# Import plotting file for your scenario
sys.path.append(path + '/../../plotting')
import OpNav_Plotting as BSK_plt

# Create your own scenario child class
class scenario_OpNav(BSKSim):
    def __init__(self):
        super(scenario_OpNav, self).__init__(BSKSim)
        self.fswRate = 0.5
        self.dynRate = 0.5
        self.set_DynModel(BSK_OpNavDynamics)
        self.set_FswModel(BSK_OpNavFsw)
        self.initInterfaces()
        self.name = 'scenario_opnav'
        self.filterUse = "bias" #"relOD"
        self.configure_initial_conditions()

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000*1E3 # meters
        oe.e = 0.
        oe.i = 20 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 100. * macros.D2R #90 good
        mu = self.get_DynModel().marsGravBody.mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,0,0]
        if self.filterUse =="relOD":
            self.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        if self.filterUse == "bias":
            self.get_FswModel().pixelLineFilterData.stateInit = rN.tolist() + vN.tolist() + bias
        # self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        # self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        self.get_DynModel().cameraMod.fieldOfView = np.deg2rad(55)

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep
        # self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorCamData.outputDataName, samplingTime)

        self.TotalSim.logThisMessage(self.get_DynModel().scObject.scStateOutMsgName,samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().imageProcessing.opnavCirclesOutMsgName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)

        # Dynamics process outputs: pull log messages below if any
        # Lr = self.pullMessageLogData(self.get_FswModel().mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))

        ## Spacecraft true states
        position_N = self.pullMessageLogData(
            self.get_DynModel().scObject.scStateOutMsgName + ".r_BN_N", range(3))
        ## Attitude
        sigma_BN = self.pullMessageLogData(
            self.get_DynModel().scObject.scStateOutMsgName + ".sigma_BN", range(3))
        ## Image processing
        validCircle = self.pullMessageLogData(
            self.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".valid", range(1))

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

    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    if not os.path.exists(runLog):
        os.makedirs(runLog)

    TheScenario.get_DynModel().cameraMod.fieldOfView = np.deg2rad(55)  # in degrees
    TheScenario.get_DynModel().cameraMod.cameraIsOn = 1
    TheScenario.get_DynModel().cameraMod.saveImages = 1
    TheScenario.get_DynModel().cameraMod.saveDir = runLog.split('/')[-2] +'/' +runLog.split('/')[-1] + '/'
    TheScenario.get_DynModel().vizInterface.opNavMode = 2

    mode = ["None", "-directComm", "-opNavMode"]
    # The following code spawns the Vizard application from python as a function of the mode selected above, and the platform.
    if platform != "darwin":
        child = subprocess.Popen([TheScenario.vizPath, "--args", mode[TheScenario.get_DynModel().vizInterface.opNavMode],
             "tcp://localhost:5556"])
    else:
        child = subprocess.Popen(["open", TheScenario.vizPath, "--args", mode[TheScenario.get_DynModel().vizInterface.opNavMode],
                                  "tcp://localhost:5556"])
    print("Vizard spawned with PID = " + str(child.pid))

    # Configure FSW mode
    TheScenario.modeRequest = 'imageGen'
    # Initialize simulation
    TheScenario.InitializeSimulationAndDiscover()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(100.)
    TheScenario.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    TheScenario.ExecuteSimulation()

    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'de430.bsp')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'naif0012.tls')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'de-403-masses.tpc')
    TheScenario.get_DynModel().SpiceObject.unloadSpiceKernel(TheScenario.get_DynModel().SpiceObject.SPICEDataPath, 'pck00010.tpc')

    return


if __name__ == "__main__":
    # Instantiate base simulation

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav()
    run(TheScenario)
