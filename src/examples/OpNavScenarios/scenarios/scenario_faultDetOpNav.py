
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk


# Get current file path
import sys, os, inspect, time, signal, subprocess
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_OpNavDynamics
import BSK_OpNavFsw
import numpy as np
from sys import platform

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import OpNav_Plotting as BSK_plt

# Create your own scenario child class
class scenario_OpNav(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_OpNav, self).__init__(masterSim)
        self.name = 'scenario_opnav'
        self.masterSim = masterSim

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
        mu = self.masterSim.get_DynModel().marsGravBody.mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,-0.3,0]
        self.masterSim.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        #primary_opnav, secondary_opnav
        self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName = "primary_opnav"
        self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName ="secondary_opnav"

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
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTimeFsw = self.masterSim.get_FswModel().processTasksTimeStep
        samplingTimeDyn = self.masterSim.get_DynModel().processTasksTimeStep
        # self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorCamData.outputDataName, samplingTime)
        # self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName,samplingTimeFsw)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName,samplingTimeFsw)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().scObject.scStateOutMsgName, samplingTimeDyn)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName,
                                               samplingTimeFsw)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName,
                                               samplingTimeFsw)

        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        NUM_STATES = 6

        ## Spacecraft true states
        position_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".r_BN_N", range(3))
        velocity_N = self.masterSim.pullMessageLogData(self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".v_BN_N", range(3))
        # position_N[:,1:] -= mars_pos[:,1:]
        # velocity_N[:,1:] -= mars_vel[:,1:]
        ## Attitude
        sigma_BN = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".sigma_BN", range(3))
        ## Navigation results
        navState = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName + ".state", range(NUM_STATES))
        navCovar = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName + ".covar",
            range(NUM_STATES * NUM_STATES))
        validLimb = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName + ".valid", range(1))
        validHough = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().pixelLineData.opNavOutMsgName + ".valid", range(1))
        ## Fault Detection
        measPos = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".r_BN_N", range(3))
        valid = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".valid", range(1))
        faults = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".faultDetected", range(1))
        r_C = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".r_BN_C", range(3))
        measCovar = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".covar_N", range(3 * 3))
        covar_C = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().opNavFaultData.opNavOutMsgName + ".covar_C", range(3 * 3))

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
    TheBSKSim.initInterfaces()

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav(TheBSKSim)
    TheScenario.configure_initial_conditions()
    TheScenario.log_outputs()

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
    TheScenario.masterSim.modeRequest = 'prepOpNav'
    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()
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
