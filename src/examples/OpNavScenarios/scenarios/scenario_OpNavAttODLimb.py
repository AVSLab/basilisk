
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

        rError= np.array([10000.,10000., -10000])
        vError= np.array([100, -10, 10])

        MRP= [0,-0.3,0]
        self.masterSim.get_FswModel().relativeODData.stateInit = (rN+rError).tolist() +  (vN+vError).tolist()
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.masterSim.get_FswModel().relativeODData.qNoise = qNoiseIn.reshape(36).tolist()
        self.masterSim.get_FswModel().horizonNavData.noiseSF = 70
        self.masterSim.get_FswModel().relativeODData.noiseSF = 5

        self.masterSim.get_DynModel().cameraMod.cameraIsOn = 1
        # Camera noise params
        # self.masterSim.get_DynModel().cameraMod.gaussian = 5 #2  #
        # self.masterSim.get_DynModel().cameraMod.darkCurrent = 0.5 #0 #
        # self.masterSim.get_DynModel().cameraMod.saltPepper = 1 # 0.5 #
        self.masterSim.get_DynModel().cameraMod.cosmicRays = 1 #1 #
        # self.masterSim.get_DynModel().cameraMod.blurParam = 5 #3 #


    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        # self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorCamData.outputDataName, samplingTime)
        # self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName, samplingTime)

        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_DynModel().scObject.scStateOutMsgName,samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().limbFinding.opnavLimbOutMsgName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)

        # Dynamics process outputs: pull log messages below if any
        ## Spacecraft true states
        position_N = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".r_BN_N", range(3))
        velocity_N = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".v_BN_N", range(3))
        ## Attitude
        sigma_BN = self.masterSim.pullMessageLogData(
            self.masterSim.get_DynModel().scObject.scStateOutMsgName + ".sigma_BN", range(3))
        ## Image processing
        limb = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().limbFinding.opnavLimbOutMsgName + ".limbPoints", range(2*2000))
        numLimbPoints = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().limbFinding.opnavLimbOutMsgName + ".numLimbPoints", range(1))
        validLimb = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().limbFinding.opnavLimbOutMsgName + ".valid", range(1))
        ## OpNav Out
        measPos = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName + ".r_BN_N", range(3))
        r_C = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName + ".r_BN_C", range(3))
        measCovar = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName + ".covar_N", range(3 * 3))
        covar_C = self.masterSim.pullMessageLogData(
        self.masterSim.get_FswModel().horizonNavData.opNavOutMsgName + ".covar_C", range(3 * 3))
        NUM_STATES = 6
        ## Navigation results
        navState = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName + ".state", range(NUM_STATES))
        navCovar = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName + ".covar",
            range(NUM_STATES * NUM_STATES))
        navPostFits = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().relativeODData.filtDataOutMsgName + ".postFitRes", range(NUM_STATES - 3))

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
        trueRhat_C = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueR_C = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueCircles = np.full([len(numLimbPoints[:,0]), 4], np.nan)
        trueCircles[:,0] = numLimbPoints[:,0]
        trueRhat_C[:,0] = numLimbPoints[:,0]
        trueR_C[:,0] = numLimbPoints[:,0]
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
        BSK_plt.nav_percentages(truth[switchIdx:,:], navState, navCovar, validLimb, "Limb")
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


def run(showPlots, simTime = None):

    # Instantiate base simulation
    TheBSKSim = BSKSim(fswRate=0.5, dynRate=0.5)
    TheBSKSim.set_DynModel(BSK_OpNavDynamics)
    TheBSKSim.set_FswModel(BSK_OpNavFsw)
    TheBSKSim.initInterfaces()

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheBSKSim.get_DynModel().cameraMod.saveImages = 0
    TheBSKSim.get_DynModel().vizInterface.opNavMode = 1

    if TheBSKSim.get_DynModel().vizInterface.opNavMode == 2:
        child = subprocess.Popen(["open", TheBSKSim.vizPath, "--args", "-opNavMode", "tcp://localhost:5556"])  # ,, "-batchmode"
    if TheBSKSim.get_DynModel().vizInterface.opNavMode == 1:
        child = subprocess.Popen(["open", TheBSKSim.vizPath, "--args", "-directComm", "tcp://localhost:5556"])  # ,, "-batchmode"
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
    TheScenario.masterSim.modeRequest = 'OpNavAttODLimb'
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
