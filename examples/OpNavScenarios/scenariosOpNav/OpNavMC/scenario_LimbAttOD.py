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

from Basilisk.utilities import RigidBodyKinematics as rbk
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
        self.configure_initial_conditions()

        self.msgRecList = {}
        self.retainedMessageNameSc = "scMsg"
        self.retainedMessageNameFilt = "filtMsg"
        self.retainedMessageNameOpNav = "opnavMsg"
        self.retainedMessageNameLimb = "limbMsg"

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

        MRP= [0,-0.3,0]
        self.get_FswModel().relativeODData.stateInit = (rN+rError).tolist() + (vN+vError).tolist()
        self.get_DynModel().scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.get_FswModel().relativeODData.qNoise = qNoiseIn.reshape(36).tolist()
        self.get_FswModel().horizonNavData.noiseSF = 20

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

        self.msgRecList[self.retainedMessageNameLimb] = FswModel.limbFinding.opnavLimbOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.retainedMessageNameLimb])

        return

    def pull_outputs(self, showPlots):
        # Dynamics process outputs: pull log messages below if any
        ## Spacecraft true states
        scRec = self.msgRecList[self.retainedMessageNameSc]
        position_N = unitTestSupport.addTimeColumn(scRec.times(), scRec.r_BN_N)
        velocity_N = unitTestSupport.addTimeColumn(scRec.times(), scRec.v_BN_N)

        ## Attitude
        sigma_BN = unitTestSupport.addTimeColumn(scRec.times(), scRec.sigma_BN)

        ## Image processing
        limbRec = self.msgRecList[self.retainedMessageNameLimb]
        limb = unitTestSupport.addTimeColumn(limbRec.times(), limbRec.limbPoints)
        numLimbPoints = unitTestSupport.addTimeColumn(limbRec.times(), limbRec.numLimbPoints)
        validLimb = unitTestSupport.addTimeColumn(limbRec.times(), limbRec.valid)

        ## OpNav Out
        opNavRec = self.msgRecList[self.retainedMessageNameOpNav]
        measPos = unitTestSupport.addTimeColumn(opNavRec.times(), opNavRec.r_BN_N)
        r_C = unitTestSupport.addTimeColumn(opNavRec.times(), opNavRec.r_BN_C)
        measCovar = unitTestSupport.addTimeColumn(opNavRec.times(), opNavRec.covar_N)
        covar_C = unitTestSupport.addTimeColumn(opNavRec.times(), opNavRec.covar_C)

        NUM_STATES = 6
        ## Navigation results
        filtRec = self.msgRecList[self.retainedMessageNameFilt]
        navState = unitTestSupport.addTimeColumn(filtRec.times(), filtRec.state)
        navCovar = unitTestSupport.addTimeColumn(filtRec.times(), filtRec.covar)
        navPostFits = unitTestSupport.addTimeColumn(filtRec.times(), filtRec.postFitRes)

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
        stateError = np.zeros([len(position_N[:,0]), NUM_STATES+1])
        navCovarLong = np.full([len(position_N[:,0]), 1+NUM_STATES*NUM_STATES], np.nan)
        navCovarLong[:,0] = np.copy(position_N[:,0])
        stateError[:, 0:4] = np.copy(position_N)
        stateError[:,4:7] = np.copy(velocity_N[:,1:])
        measError = np.full([len(measPos[:,0]), 4], np.nan)
        measError[:,0] = measPos[:,0]
        measError_C = np.full([len(measPos[:,0]), 5], np.nan)
        measError_C[:,0] = measPos[:,0]
        trueRhat_C = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueR_C = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueCircles = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueCircles[:,0] = numLimbPoints[:,0]
        trueRhat_C[:,0] = numLimbPoints[:,0]
        trueR_C[:,0] = numLimbPoints[:,0]


        switchIdx = 0

        Rmars = 3396.19*1E3
        for j in range(len(stateError[:, 0])):
            if stateError[j, 0] in navState[:, 0]:
                stateError[j, 1:4] -= navState[j - switchIdx, 1:4]
                stateError[j, 4:] -= navState[j - switchIdx, 4:]
            else:
                stateError[j, 1:] = np.full(NUM_STATES, np.nan)
                switchIdx += 1
        for i in range(len(numLimbPoints[:,0])):
            if numLimbPoints[i,1] > 1E-8:
                measError[i, 1:4] = position_N[i +switchIdx, 1:4] - measPos[i, 1:4]
                measError_C[i, 4] = np.linalg.norm(position_N[i +switchIdx, 1:4]) - np.linalg.norm(r_C[i, 1:4])
                trueR_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[i +switchIdx, 1:4])) , position_N[i +switchIdx, 1:4])
                trueRhat_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[i +switchIdx, 1:4])) ,position_N[i +switchIdx, 1:4])/np.linalg.norm(position_N[i +switchIdx, 1:4])
                trueCircles[i,3] = focal*np.tan(np.arcsin(Rmars/np.linalg.norm(position_N[i,1:4])))/pixelSize[0]
                trueRhat_C[i,1:] *= focal/trueRhat_C[i,3]
                measError_C[i, 1:4] = trueRhat_C[i,1:] - r_C[i, 1:4]/np.linalg.norm(r_C[i, 1:4])
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0]/2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1]/2 - 0.5
            else:
                measCovar[i,1:] = np.full(3*3, np.nan)
                covar_C[i, 1:] = np.full(3 * 3, np.nan)
        navCovarLong[switchIdx:,:] = np.copy(navCovar)

        timeData = position_N[:, 0] * macros.NANO2MIN

        BSK_plt.plot_TwoOrbits(position_N, measPos)
        BSK_plt.diff_vectors(trueR_C, r_C, validLimb, "Limb")
        BSK_plt.plot_limb(limb, numLimbPoints, validLimb, sizeOfCam)
        # BSK_plt.AnimatedScatter(sizeOfCam, circleCenters, circleRadii, validCircle)
        BSK_plt.plotStateCovarPlot(stateError, navCovarLong)

        # BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))
        BSK_plt.plotPostFitResiduals(navPostFits, measCovar)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(TheScenario):

    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheScenario.get_FswModel().imageProcessing.saveImages = 0
    TheScenario.get_DynModel().vizInterface.opNavMode = 1

    mode = ["None", "-directComm", "-noDisplay"]
    vizard = subprocess.Popen(
        [TheScenario.vizPath, "--args", mode[TheScenario.get_DynModel().vizInterface.opNavMode],
         "tcp://localhost:5556"], stdout=subprocess.DEVNULL)
    print("Vizard spawned with PID = " + str(vizard.pid))

    # Configure FSW mode
    TheScenario.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheScenario.InitializeSimulation()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(3.)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()
    TheScenario.modeRequest = 'OpNavAttODLimb'
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
