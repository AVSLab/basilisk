
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


# Get current file path
import sys, os, inspect, time, subprocess, signal
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np


# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import OpNav_Plotting as BSK_plt

# Create your own scenario child class
class scenario_OpNav(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_OpNav, self).__init__(masterSim)
        self.name = 'scenario_opnav'
        self.masterSim = masterSim
        self.filterUse = "bias" #"relOD"

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
        ## Image processing
        circleCenters = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".circlesCenters", range(2*10))
        circleRadii = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".circlesRadii", range(10))
        validCircle = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().imageProcessing.opnavCirclesOutMsgName+ ".valid", range(1))

        # sigma_RN = self.masterSim.pullMessageLogData(
        #     self.masterSim.get_FswModel().opNavPointData.attGuidanceOutMsgName + ".sigma_RN", list(range(3)))
        # omega_RN_N = self.masterSim.pullMessageLogData(
        #     self.masterSim.get_FswModel().opNavPointData.attGuidanceOutMsgName + ".omega_RN_B", list(range(3)))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorCamData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorCamData.outputDataName + ".omega_BR_B", list(range(3)))


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
        Rmars = 3396.19*1E3
        for j in range(len(position_N[:, 0])):
            if position_N[j, 0] in circleCenters[:, 0]:
                ModeIdx = j
                break
        for i in range(len(circleCenters[:,0])):
            if circleCenters[i,1:].any() > 1E-8 or circleCenters[i,1:].any() < -1E-8:
                trueRhat_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[ModeIdx+i , 1:4])) ,position_N[ModeIdx+i, 1:4])/np.linalg.norm(position_N[ModeIdx+i, 1:4])
                trueCircles[i,3] = focal*np.tan(np.arcsin(Rmars/np.linalg.norm(position_N[ModeIdx+i,1:4])))/pixelSize[0]
                trueRhat_C[i,1:] *= focal/trueRhat_C[i,3]
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0]/2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1]/2 - 0.5

                measError[i, 1:4] = position_N[ModeIdx+i, 1:4] - measPos[i, 1:4]
                measError_C[i, 4] = np.linalg.norm(position_N[ModeIdx+i, 1:4]) - np.linalg.norm(r_C[i, 1:4])
                measError_C[i, 1:4] = trueRhat_C[i,1:] - r_C[i, 1:4]/np.linalg.norm(r_C[i, 1:4])
            else:
                measCovar[i,1:] = np.full(3*3, np.nan)
                covar_C[i, 1:] = np.full(3 * 3, np.nan)

        timeData = position_N[:, 0] * macros.NANO2MIN

        # BSK_plt.AnimatedScatter(sizeOfCam, circleCenters, circleRadii, validCircle)
        # BSK_plt.plot_cirlces(timeData[switchIdx:], circleCenters, circleRadii, validCircle, sizeOfCam)
        BSK_plt.plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)

        BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))
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
    TheBSKSim.get_DynModel().vizInterface.opNavMode = 2

    if TheBSKSim.get_DynModel().vizInterface.opNavMode == 2:
        child = subprocess.Popen(["open", TheBSKSim.vizPath, "--args", "-opNavMode",
                                  "tcp://localhost:5556"])  # ,, "-batchmode"
    if TheBSKSim.get_DynModel().vizInterface.opNavMode == 1:
        child = subprocess.Popen(["open", TheBSKSim.vizPath, "--args", "-directComm",
                                  "tcp://localhost:5556"])  # ,, "-batchmode"
    print("Vizard spawned with PID = " + str(child.pid))

    # Configure FSW mode
    TheScenario.masterSim.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()
    # Configure run time and execute simulation
    simulationTime = macros.min2nano(5.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    t1 = time.time()
    TheBSKSim.ExecuteSimulation()
    TheScenario.masterSim.modeRequest = 'pointLimb'
    if simTime != None:
        simulationTime = macros.min2nano(simTime)
    else:
        simulationTime = macros.min2nano(50)
    TheBSKSim.ConfigureStopTime(simulationTime)
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
