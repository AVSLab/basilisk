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

This scenario uses the two OpNav methods implemented in previous scenarios for fault detection.
The algorithm provides a similarity check and can be found in :ref:`faultDetection`.
More details can be found in Chapter 5 of `Thibaud Teil's PhD thesis <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.


The script can be run at full length by calling::

    python3 scenario_faultDetOpNav.py

"""


# Get current file path
import inspect
import os
import sys
import time

from Basilisk.utilities import RigidBodyKinematics as rbk
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_OpNav import BSKSim, BSKScenario
import BSK_OpNavDynamics
import BSK_OpNavFsw
import numpy as np

# Import plotting file for your scenario
sys.path.append(path + '/../plottingOpNav')
import OpNav_Plotting as BSK_plt

from Basilisk.architecture import messaging


# Create your own scenario child class
class scenario_OpNav(BSKScenario):
    """Main Simulation Class"""

    def __init__(self, masterSim, showPlots=False):
        super(scenario_OpNav, self).__init__(masterSim, showPlots)
        self.name = 'scenario_opnav'
        self.masterSim = masterSim

        # declare additional class variables
        self.opNavRec = None
        self.opNavPrimRec = None
        self.opNavSecRec = None
        self.scRec = None
        self.filtRec = None

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000 * 1E3  # meters
        oe.e = 0.6
        oe.i = 10 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 80. * macros.D2R  # 90 good
        mu = self.masterSim.get_DynModel().gravFactory.gravBodies['mars barycenter'].mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,-0.3,0]
        self.masterSim.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        # primary_opnav, secondary_opnav
        FswModel = self.masterSim.get_FswModel()
        messaging.OpNavMsg_C_addAuthor(FswModel.horizonNavData.opNavOutMsg, FswModel.opnavPrimaryMsg)
        messaging.OpNavMsg_C_addAuthor(FswModel.pixelLineData.opNavOutMsg, FswModel.opnavSecondaryMsg)

        # Filter noise param
        self.masterSim.get_FswModel().relativeODData.noiseSF = 5

        # Camera noise params
        # self.masterSim.get_DynModel().cameraMod.gaussian = 2 #3 #
        # self.masterSim.get_DynModel().cameraMod.darkCurrent = 0 #1 #
        # self.masterSim.get_DynModel().cameraMod.saltPepper =  0.5 #1 #
        self.masterSim.get_DynModel().cameraMod.cosmicRays = 2 # 2 #
        # self.masterSim.get_DynModel().cameraMod.blurParam = 3 #4 #

        # Fault params
        self.masterSim.get_FswModel().opNavFaultData.sigmaFault = 1
        self.masterSim.get_FswModel().opNavFaultData.faultMode = 0

    def log_outputs(self):
        # Dynamics process outputs: log messages below if desired.
        FswModel = self.masterSim.get_FswModel()
        DynModel = self.masterSim.get_DynModel()

        # FSW process outputs
        samplingTimeFsw = self.masterSim.get_FswModel().processTasksTimeStep
        samplingTimeDyn = self.masterSim.get_DynModel().processTasksTimeStep

        self.filtRec = FswModel.relativeODData.filtDataOutMsg.recorder(samplingTimeFsw)
        self.opNavRec = FswModel.opnavMsg.recorder(samplingTimeFsw)
        self.scRec = DynModel.scObject.scStateOutMsg.recorder(samplingTimeDyn)
        self.opNavPrimRec = FswModel.opnavPrimaryMsg.recorder(samplingTimeFsw)
        self.opNavSecRec = FswModel.opnavSecondaryMsg.recorder(samplingTimeFsw)

        self.masterSim.AddModelToTask(DynModel.taskName, self.filtRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.opNavRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.scRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.opNavPrimRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.opNavSecRec)

        return

    def pull_outputs(self, showPlots):
        NUM_STATES = 6

        ## Spacecraft true states
        position_N = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.r_BN_N)
        velocity_N = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.v_BN_N)

        ## Attitude
        sigma_BN = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.sigma_BN)

        ## Navigation results
        navState = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.state)
        navCovar = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.covar)

        validLimb = unitTestSupport.addTimeColumn(self.opNavPrimRec.times(), self.opNavPrimRec.valid)
        validHough = unitTestSupport.addTimeColumn(self.opNavSecRec.times(), self.opNavSecRec.valid)

        ## Fault Detection
        measPos = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.r_BN_N)
        valid = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.valid)
        faults = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.faultDetected)
        r_C = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.r_BN_C)
        measCovar = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.covar_N)
        covar_C = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.covar_C)

        sigma_CB = self.masterSim.get_DynModel().cameraMRP_CB
        sizeMM = self.masterSim.get_DynModel().cameraSize
        sizeOfCam = self.masterSim.get_DynModel().cameraRez
        focal = self.masterSim.get_DynModel().cameraFocal  # in m

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
        trueRhat_C = np.full([len(measPos[:,0]), 4], np.nan)
        trueRhat_C[:,0] = measPos[:,0]
        truth = np.zeros([len(position_N[:,0]), 7])
        truth[:,0:4] = np.copy(position_N)
        truth[:,4:7] = np.copy(velocity_N[:,1:])

        switchIdx = 0

        Rmars = 3396.19*1E3
        for j in range(len(stateError[:, 0])):
            if stateError[j, 0] in navState[:, 0]:
                stateError[j, 1:4] -= navState[j - switchIdx, 1:4]
                stateError[j, 4:] -= navState[j - switchIdx, 4:]
            else:
                stateError[j, 1:] = np.full(NUM_STATES, np.nan)
                switchIdx += 1
        for i in range(len(measPos[:,0])):
            if measPos[i,1] > 1E-8:
                measError[i, 1:4] = position_N[i +switchIdx, 1:4] - measPos[i, 1:4]
                measError_C[i, 4] = np.linalg.norm(position_N[i +switchIdx, 1:4]) - np.linalg.norm(r_C[i, 1:4])
                measError_C[i, 1:4] = trueRhat_C[i,1:] - r_C[i, 1:4]/np.linalg.norm(r_C[i, 1:4])
                trueRhat_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[i +switchIdx, 1:4])) ,position_N[i +switchIdx, 1:4])/np.linalg.norm(position_N[i +switchIdx, 1:4])
                trueRhat_C[i,1:] *= focal/trueRhat_C[i,3]
            else:
                measCovar[i,1:] = np.full(3*3, np.nan)
                covar_C[i, 1:] = np.full(3 * 3, np.nan)
        navCovarLong[switchIdx:,:] = np.copy(navCovar)

        timeData = position_N[:, 0] * macros.NANO2MIN

        # BSK_plt.diff_methods(position_N[switchIdx:,:], measPos, measPosPL, validLimb, validCircle)
        # BSK_plt.plot_cirlces(circleCenters, circleRadii, validCircle, sizeOfCam)
        # BSK_plt.plot_limb(limb, numLimbPoints, validLimb, sizeOfCam)
        # BSK_plt.AnimatedScatter(sizeOfCam, circleCenters, circleRadii, validCircle)
        # BSK_plt.plot_cirlces(timeData[switchIdx:], circleCenters, circleRadii, validCircle, sizeOfCam)
        BSK_plt.plot_faults(faults, validLimb, validHough)
        BSK_plt.nav_percentages(truth[switchIdx:,:], navState, navCovar, valid, "Fault")
        BSK_plt.plotStateCovarPlot(stateError, navCovarLong)

        # BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))
        # BSK_plt.plotPostFitResiduals(navPostFits, measCovar)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList

# Time in min
def run(showPlots, simTime = None):

    # Instantiate base simulation
    TheBSKSim = BSKSim(fswRate=0.5, dynRate=0.5)
    TheBSKSim.set_DynModel(BSK_OpNavDynamics)
    TheBSKSim.set_FswModel(BSK_OpNavFsw)

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav(TheBSKSim, showPlots)
    TheScenario.configure_initial_conditions()
    TheScenario.log_outputs()

    TheBSKSim.get_DynModel().cameraMod.saveImages = 0
    # opNavMode 1 is used for viewing the spacecraft as it navigates, opNavMode 2 is for headless camera simulation
    TheBSKSim.get_DynModel().vizInterface.opNavMode = 2

    # The following code spawns the Vizard application from python
    mode = ["None", "-directComm", "-noDisplay"]
    TheScenario.run_vizard(mode[TheBSKSim.get_DynModel().vizInterface.opNavMode])

    # Configure FSW mode
    TheScenario.masterSim.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheBSKSim.InitializeSimulation()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(3.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    t1 = time.time()
    TheBSKSim.ExecuteSimulation()
    TheScenario.masterSim.modeRequest = 'FaultDet'
    if simTime != None:
        simulationTime = macros.min2nano(simTime)
    else:
        simulationTime = macros.min2nano(600)
    TheBSKSim.ConfigureStopTime(simulationTime)
    TheBSKSim.ExecuteSimulation()
    t2 = time.time()
    print('Finished Execution in ', t2-t1, ' seconds. Post-processing results')
    # Terminate vizard and show plots
    figureList = TheScenario.end_scenario()
    return figureList


if __name__ == "__main__":
    run(True)
