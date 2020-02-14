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

This scenario is similar to the pointing scenario. The spacecraft is simply attempting to point to the planet.


On top of using src/fswAlgorithms/attGuidance/opNavPoint, it also filters the measurements using a "Switch" filter
found in src/fswAlgorithms/attDetermination/headingSuKF.
More details can be found in Chapter 2-3 of `Thibaud Teil's PhD thesis <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.

The script can be run at full length by calling::

    python3 scenario_OpNavHeading.py

"""
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


# Get current file path
import sys, os, inspect, time, subprocess, signal
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_OpNav import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np
from sys import platform

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import OpNav_Plotting as BSK_plt

def DCM(bVec, d):
    DCM_exp = np.zeros([3,3])

    if np.linalg.norm(np.cross(bVec,d)) <1E-5:
        return np.eye(3)
    else:
        DCM_exp[:, 0] = np.array(d) / np.linalg.norm(d)
        DCM_exp[:, 1] = np.cross(DCM_exp[:, 0], bVec) / np.linalg.norm(np.array(np.cross(DCM_exp[:, 0], bVec)))
        DCM_exp[:, 2] = np.cross(DCM_exp[:, 0], DCM_exp[:, 1]) / np.linalg.norm(
            np.cross(DCM_exp[:, 0], DCM_exp[:, 1]))
        return DCM_exp

# Create your own scenario child class
class scenario_OpNav(BSKScenario):
    """Main Simulation Class"""
    def __init__(self, masterSim):
        super(scenario_OpNav, self).__init__(masterSim)
        self.name = 'scenario_opnav'
        self.masterSim = masterSim
        self.filterUse = "bias" #"relOD"

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 5000*1E3 # meters
        self.semiMajAxis = oe.a
        oe.e = 0.
        oe.i = 20 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 100. * macros.D2R #90 good
        mu = self.masterSim.get_DynModel().marsGravBody.mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,0,0]
        if self.filterUse =="relOD":
            self.masterSim.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        if self.filterUse == "bias":
            self.masterSim.get_FswModel().pixelLineFilterData.stateInit = rN.tolist() + vN.tolist() + bias
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        # Search
        self.masterSim.get_FswModel().opNavPointData.omega_RN_B = [0.001, 0.0, -0.001]
        # self.masterSim.get_FswModel().opNavPointData.opnavDataInMsgName = "heading_filtered"
        self.masterSim.get_FswModel().imageProcessing.noiseSF = 0.5
        self.masterSim.get_FswModel().headingUKFData.noiseSF = 1.001

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        # self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorCamData.outputDataName, samplingTime)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().opNavPointData.attGuidanceOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().scObject.scStateOutMsgName,samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data", "rw_config_3_data"]
        self.masterSim.AddVariableForLogging('headingUKF.bVec_B', samplingTime, 0, 2)
        self.masterSim.TotalSim.logThisMessage('heading_filter_data', samplingTime)
        self.masterSim.TotalSim.logThisMessage("heading_filtered", samplingTime)

        for item in rwOutName:
            self.masterSim.TotalSim.logThisMessage(item, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)

        # Dynamics process outputs: pull log messages below if any
        # Lr = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().mrpFeedbackControlData.outputDataName + ".torqueRequestBody", range(3))

        ## Spacecraft true states
        position_N = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".r_BN_N", range(3))
        velocity_N = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".v_BN_N", range(3))
        ## Attitude
        sigma_BN = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".sigma_BN", range(3))
        Outomega_BN = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().scObject.scStateOutMsgName  + ".omega_BN_B", range(3))
        ## Image processing
        circleCenters = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".circlesCenters", range(2*10))
        circleRadii = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".circlesRadii", range(10))
        validCircle = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".valid", range(1))

        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorCamData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorCamData.outputDataName + ".omega_BR_B", list(range(3)))
        frame = self.masterSim.GetLogVariableData('headingUKF.bVec_B')

        numRW = 4
        dataUsReq = self.masterSim.pullMessageLogData(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", list(range(numRW)))
        rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data", "rw_config_3_data"]
        dataRW = []
        for i in range(0, numRW):
            dataRW.append(self.masterSim.pullMessageLogData(rwOutName[i] + ".u_current", list(range(1))))

        measPos = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".r_BN_N", range(3))
        r_C = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".r_BN_C", range(3))
        measCovar = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".covar_N", range(3*3))
        covar_C = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".covar_C", range(3*3))
        covar_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".covar_B", range(3*3))


        FilterType = "Switch-SRuKF"
        numStates = 5
        # Get the filter outputs through the messages
        stateLog = self.masterSim.pullMessageLogData('heading_filter_data' + ".state", range(numStates))
        r_BN_C = self.masterSim.pullMessageLogData('heading_filtered' + ".r_BN_C", range(3))
        postFitLog = self.masterSim.pullMessageLogData('heading_filter_data' + ".postFitRes", range(3))
        covarLog = self.masterSim.pullMessageLogData('heading_filter_data' + ".covar", range(numStates * numStates))

        for i in range(len(stateLog[:, 0])):
            stateLog[i, 1:4] = stateLog[i, 1:4] / np.linalg.norm(stateLog[i, 1:4])

        sHat_B = np.zeros(np.shape(position_N))
        sHatDot_B = np.zeros(np.shape(position_N))
        for i in range(len(position_N[:, 0])):
            sHat_N = - position_N[i,1:4]/np.linalg.norm(position_N[i,1:4])
            dcm_BN = rbk.MRP2C(sigma_BN[i, 1:])
            sHat_B[i, 0] = sHatDot_B[i, 0] = position_N[i, 0]
            sHat_B[i, 1:] = np.dot(dcm_BN, sHat_N)
            sHatDot_B[i, 1:] = - np.cross(Outomega_BN[i, 1:], sHat_B[i, 1:])

        stateLogSEKF = np.zeros([len(stateLog[:, 0]), 7])
        stateLogSEKF[:, 0:4] = stateLog[:, 0:4]

        expected = np.zeros(np.shape(stateLog))
        expectedSEKF = np.zeros(np.shape(stateLogSEKF))
        expectedSEKF[:, 0:4] = sHat_B
        expectedSEKF[:, 4:] = sHatDot_B[:, 1:]
        expected[:, 0:4] = sHat_B


        filterOmega_BN = np.zeros([len(stateLog[:, 0]), 4])
        filterOmega_BN[:, 0] = np.copy(stateLog[:, 0])
        trueOmega_BN_S = np.zeros([len(stateLog[:, 0]), 4])
        trueOmega_BN_S[:, 0] = np.copy(stateLog[:, 0])
        covarLog_B = np.copy(covarLog)
        filterOmega_BN_S = np.zeros(np.shape(trueOmega_BN_S))
        filterOmega_BN_S[:, 2:] = stateLog[:, 4:6]
        dcmBS = np.zeros([position_N.shape[0],3,3])
        for i in range(len(stateLog[:, 0])):
            DCM_BS = DCM(frame[i, 1:], sHat_B[i, 1:])
            dcmBS[i,:,:] = DCM_BS
            expected[i, 4:6] = np.dot(DCM_BS, Outomega_BN[i, 1:])[1:3]
            trueOmega_BN_S[i, 1:] = np.dot(np.transpose(DCM_BS), Outomega_BN[i, 1:])
            filterOmega_BN[i, 1:] = np.dot(DCM_BS, np.array([0., stateLog[i, 4], stateLog[i, 5]]))
            stateLogSEKF[i, 4:] = - np.cross(filterOmega_BN[i, 1:], stateLog[i, 1:4])
            tempCovar = np.zeros([3, 3])
            tempCovar[1:, 1:] = np.reshape(covarLog[i, 1:5 * 5 + 1], [5, 5])[3:, 3:]
            covarLog_B[i, -4:] = np.dot(np.dot(DCM_BS, tempCovar), np.transpose(DCM_BS))[1:, 1:].flatten()

        #
        #   plot the results
        #
        errorVsTruth = np.copy(stateLog)
        errorVsTruth[:, 1:] -= expected[:, 1:]
        errorVsTruthSEKF = np.copy(stateLogSEKF)
        errorVsTruthSEKF[:, 1:] -= expectedSEKF[:, 1:]

        errorDeg = np.zeros([len(expected[:, 0]), 2])
        rateError = np.zeros([len(expected[:, 0]), 2])
        covarDeg = np.zeros([len(expected[:, 0]), 2])
        for i in range(len(errorDeg[:, 0])):
            errorDeg[i, 0] = stateLog[i, 0]
            rateError[i, 0] = stateLog[i, 0]
            covarDeg[i, 0] = stateLog[i, 0]
            errorDeg[i, 1] = np.arccos(np.dot(stateLogSEKF[i, 1:4], expectedSEKF[i, 1:4]))
            rateError[i, 1] = np.linalg.norm(errorVsTruthSEKF[i, 4:])

            covarVec = np.array([stateLog[i, 1] + np.sqrt(covarLog[i, 1]), stateLog[i, 2] + np.sqrt(covarLog[i, 2 + numStates]),
                 stateLog[i, 3] + np.sqrt(covarLog[i, 3 + 2 * numStates])])
            covarVec = covarVec / np.linalg.norm(covarVec)
            covarDeg[i, 1] = 3 * np.arccos(np.dot(covarVec, stateLog[i, 1:4]))
            # covarDeg[i, 1] = np.linalg.norm(np.array([np.sqrt(covarLog[i,1]),np.sqrt(covarLog[i,1]),np.sqrt(covarLog[i,1])]))

        FilterNames = []
        errorsDict = {}
        sigmas = {}
        omegaErrors_S = {}
        omegaErrors_B = {}
        FilterNames.append(FilterType)
        errorsDict[FilterType] = [errorDeg, covarDeg]
        sigmas[FilterType] = [sigma_BN]
        omegaErrors_S[FilterType] = [filterOmega_BN_S, trueOmega_BN_S, covarLog]
        omegaErrors_B[FilterType] = [filterOmega_BN, Outomega_BN, covarLog_B]

        sigma_CB = self.masterSim.get_DynModel().cameraMRP_CB
        sizeMM = self.masterSim.get_DynModel().cameraSize
        sizeOfCam = self.masterSim.get_DynModel().cameraRez
        focal = self.masterSim.get_DynModel().cameraFocal #in m

        pixelSize = []
        pixelSize.append(sizeMM[0] / sizeOfCam[0])
        pixelSize.append(sizeMM[1] / sizeOfCam[1])

        dcm_CB = rbk.MRP2C(sigma_CB)
        # Plot results
        BSK_plt.clear_all_plots()
        pixCovar = np.ones([len(circleCenters[:,0]), 3*3+1])
        pixCovar[:,0] = circleCenters[:,0]
        pixCovar[:,1:]*=np.array([1,0,0,0,1,0,0,0,2])

        measError = np.full([len(measPos[:,0]), 4], np.nan)
        measError[:,0] = measPos[:,0]
        measError_C = np.full([len(measPos[:,0]), 5], np.nan)
        measError_C[:,0] = measPos[:,0]

        trueRhat_C = np.full([len(circleCenters[:,0]), 4], np.nan)
        trueCircles = np.full([len(circleCenters[:,0]), 4], np.nan)
        trueCircles[:,0] = circleCenters[:,0]
        trueRhat_C[:,0] = circleCenters[:,0]

        centerBias = np.copy(circleCenters)
        radBias = np.copy(circleRadii)

        ModeIdx = 0
        modeSwitch = 0
        Rmars = 3396.19*1E3
        for j in range(len(position_N[:, 0])):
            if circleCenters[j, 1] > 0:
                modeSwitch = j
                break
        covarC = np.zeros([covarLog.shape[0], 3, 3])
        covarOmega = np.zeros([covarLog.shape[0], 2, 2])
        for i in range(len(circleCenters[:,0])):
            trueRhat_C[i, 1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[ModeIdx + i, 1:4])),
                                       position_N[ModeIdx + i, 1:4]) / np.linalg.norm(position_N[ModeIdx + i, 1:4])
            trueRhat_C[i, 1:] *= focal / trueRhat_C[i, 3]
            covarC[i, :, :] = np.array(covarLog[i, 1:]).reshape([5, 5])[:3,:3]
            covarOmega[i, :, :] = np.array(covarLog[i, 1:]).reshape([5, 5])[4:,4:]
            covarC[i, :,:] = np.dot(np.dot(dcm_CB, covarC[i, :, :]), dcm_CB.T)
            temp = np.zeros([3,3])
            temp[0,0] = covarOmega[i, 1, 1]
            temp[1:,1:] = covarOmega[i, :, :]
            covarOmega[i, :,:] = np.dot(np.dot(dcmBS[i,:,:], temp), dcmBS[i,:,:].T)[1:,1:]
            if circleCenters[i,1:].any() > 1E-8 or circleCenters[i,1:].any() < -1E-8:
                trueCircles[i,3] = focal*np.tan(np.arcsin(Rmars/np.linalg.norm(position_N[ModeIdx+i,1:4])))/pixelSize[0]
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0]/2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1]/2 - 0.5

                measError[i, 1:4] = position_N[ModeIdx+i, 1:4] - measPos[i, 1:4]
                measError_C[i, 4] = np.linalg.norm(position_N[ModeIdx+i, 1:4]) - np.linalg.norm(r_C[i, 1:4])
                measError_C[i, 1:4] = trueRhat_C[i,1:] - r_C[i, 1:4]/np.linalg.norm(r_C[i, 1:4])
            else:
                measCovar[i,1:] = np.full(3*3, np.nan)

        timeData = position_N[:, 0] * macros.NANO2MIN

        # BSK_plt.AnimatedCircles(sizeOfCam, circleCenters, circleRadii, validCircle)
        # BSK_plt.plot_cirlces(timeData[switchIdx:], circleCenters, circleRadii, validCircle, sizeOfCam)
        # plt.close('all')
        show_plots = True
        m2km = 1E-3
        covar_B[:,1:] *=1./(self.semiMajAxis**2)
        covarOmega[:,1,1] *=3
        r_NB_hat_C = np.copy(r_BN_C)
        for i in range(r_NB_hat_C.shape[0]):
            r_NB_hat_C[i,1:]*= -1./np.linalg.norm(r_NB_hat_C[i,1:])
            trueRhat_C[i,1:]*=1./np.linalg.norm(trueRhat_C[i,1:])
        rError = np.copy(r_NB_hat_C)
        rError[:,1:] -= trueRhat_C[:,1:]
        omegaError = np.zeros([position_N.shape[0],3])
        omegaError[:,0] = position_N[:,0]
        omegaError[:,1:] = errorVsTruth[:,4:]
        BSK_plt.vecTrack(trueRhat_C[modeSwitch:,:], r_NB_hat_C[modeSwitch:,:], covarC[modeSwitch:,:])
        # BSK_plt.omegaTrack(omegaError[modeSwitch:], covarOmega[modeSwitch:,:,:])
        # BSK_plt.PostFitResiduals(postFitLog[modeSwitch:,:], covar_B[modeSwitch:,:], FilterType, show_plots)
        # BSK_plt.plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
        # BSK_plt.plot_attitude_error(timeData, sigma_BR)
        # BSK_plt.plot_rate_error(timeData, omega_BR_B)
        #
        # BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))
        # BSK_plt.centerXY(circleCenters, np.array(sizeOfCam))

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots, simTime = None):

    # Instantiate base simulation
    TheBSKSim = BSKSim(fswRate=0.5, dynRate=0.5)
    TheBSKSim.set_DynModel(BSK_OpNavDynamics)
    TheBSKSim.set_FswModel(BSK_OpNavFsw)
    TheBSKSim.initInterfaces()

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav(TheBSKSim)
    if showPlots:
        TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheBSKSim.get_DynModel().cameraMod.saveImages = 0
    # opNavMode 1 is used for viewing the spacecraft as it navigates, opNavMode 2 is for headless camera simulation
    TheBSKSim.get_DynModel().vizInterface.opNavMode = 2

    mode = ["None", "-directComm", "-opNavMode"]
    # The following code spawns the Vizard application from python as a function of the mode selected above, and the platform.
    if platform != "darwin":
        child = subprocess.Popen([TheBSKSim.vizPath, "--args", mode[TheBSKSim.get_DynModel().vizInterface.opNavMode],
             "tcp://localhost:5556"])
    else:
        child = subprocess.Popen(["open", TheBSKSim.vizPath, "--args", mode[TheBSKSim.get_DynModel().vizInterface.opNavMode],
                                  "tcp://localhost:5556"])
    print("Vizard spawned with PID = " + str(child.pid))

    # Configure FSW mode
    TheScenario.masterSim.modeRequest = 'pointHead'
    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()
    # Configure run time and execute simulation
    if simTime != None:
        simulationTime = macros.min2nano(simTime)
    else:
        simulationTime = macros.min2nano(200)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    t1 = time.time()
    TheBSKSim.ExecuteSimulation()
    t2 = time.time()
    print('Finished Execution in ', t2-t1, ' seconds. Post-processing results')

    try:
        os.kill(child.pid + 1, signal.SIGKILL)
    except:
        print("IDK how to turn this thing off")

    # Pull the results of the base simulation running the chosen scenario
    if showPlots:
        figureList = TheScenario.pull_outputs(showPlots)
        return figureList
    else:
        return {}

if __name__ == "__main__":
    run(True)
