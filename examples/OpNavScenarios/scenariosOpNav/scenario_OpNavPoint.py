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

This scenario only performs the pointing component to the OpNav FSW stack.
It uses Hough Circles to identify the planet center.
More details can be found in Chapter 2 of `Thibaud Teil's PhD thesis <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.

The script can be run at full length by calling::

    python3 scenario_OpNavPoint.py

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
import BSK_OpNavDynamics, BSK_OpNavFsw
import numpy as np

# Import plotting file for your scenario
sys.path.append(path + '/../plottingOpNav')
import OpNav_Plotting as BSK_plt

# Create your own scenario child class
class scenario_OpNav(BSKScenario):
    """Main Simulation Class"""

    def __init__(self, masterSim, showPlots=False):
        super(scenario_OpNav, self).__init__(masterSim, showPlots)
        self.name = 'scenario_opnav'
        self.masterSim = masterSim
        self.filterUse = "bias" #"relOD"

        # declare additional class variables
        self.rwMotorRec = None
        self.opNavRec = None
        self.attGuidRec = None
        self.circlesRec = None
        self.scRec = None
        self.rwLogs = []

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 18000*1E3 # meters
        oe.e = 0.
        oe.i = 20 * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 190. * macros.D2R
        oe.f = 100. * macros.D2R  # 90 good
        mu = self.masterSim.get_DynModel().gravFactory.gravBodies['mars barycenter'].mu

        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        bias = [0, 0, -2]

        MRP= [0,0,0]
        if self.filterUse == "relOD":
            self.masterSim.get_FswModel().relativeODData.stateInit = rN.tolist() + vN.tolist()
        if self.filterUse == "bias":
            self.masterSim.get_FswModel().pixelLineFilterData.stateInit = rN.tolist() + vN.tolist() + bias
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = rN
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = vN
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B
        # Search
        self.masterSim.get_FswModel().opNavPointData.omega_RN_B = [0.001, 0.0, -0.001]

    def log_outputs(self):
        # Dynamics process outputs: log messages below if desired.
        FswModel = self.masterSim.get_FswModel()
        DynModel = self.masterSim.get_DynModel()
        # FSW process outputs
        samplingTime = FswModel.processTasksTimeStep

        self.opNavRec = FswModel.opnavMsg.recorder(samplingTime)
        self.attGuidRec = FswModel.attGuidMsg.recorder(samplingTime)
        self.rwMotorRec = FswModel.rwMotorTorqueData.rwMotorTorqueOutMsg.recorder(samplingTime)
        self.circlesRec = FswModel.opnavCirclesMsg.recorder(samplingTime)
        self.scRec = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.masterSim.AddModelToTask(DynModel.taskName, self.opNavRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.attGuidRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.rwMotorRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.circlesRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.scRec)

        self.rwLogs = []
        for item in range(4):
            self.rwLogs.append(DynModel.rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
            self.masterSim.AddModelToTask(DynModel.taskName, self.rwLogs[item])

        return

    def pull_outputs(self, showPlots):

        ## Spacecraft true states
        position_N = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.r_BN_N)

        ## Attitude
        sigma_BN = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.sigma_BN)

        ## Image processing
        circleCenters = unitTestSupport.addTimeColumn(self.circlesRec.times(), self.circlesRec.circlesCenters)
        circleRadii = unitTestSupport.addTimeColumn(self.circlesRec.times(), self.circlesRec.circlesRadii)

        numRW = 4
        dataUsReq = unitTestSupport.addTimeColumn(self.rwMotorRec.times(), self.rwMotorRec.motorTorque)
        dataRW = []
        for i in range(numRW):
            dataRW.append(unitTestSupport.addTimeColumn(self.rwMotorRec.times(), self.rwLogs[i].u_current))

        measPos = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.r_BN_N)
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

        BSK_plt.plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)

        BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots, simTime=None):

    # Instantiate base simulation
    TheBSKSim = BSKSim(fswRate=0.5, dynRate=0.5)
    TheBSKSim.set_DynModel(BSK_OpNavDynamics)
    TheBSKSim.set_FswModel(BSK_OpNavFsw)

    # Configure a scenario in the base simulation
    TheScenario = scenario_OpNav(TheBSKSim, showPlots)
    if showPlots:
        TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    TheBSKSim.get_DynModel().cameraMod.saveImages = 0
    # opNavMode 1 is used for viewing the spacecraft as it navigates, opNavMode 2 is for headless camera simulation
    TheBSKSim.get_DynModel().vizInterface.opNavMode = 2

    # The following code spawns the Vizard application from python
    mode = ["None", "-directComm", "-noDisplay"]
    TheScenario.run_vizard(mode[TheBSKSim.get_DynModel().vizInterface.opNavMode])
    # Configure FSW mode
    TheScenario.masterSim.modeRequest = 'pointOpNav'
    # Initialize simulation
    TheBSKSim.InitializeSimulation()
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
    # Terminate vizard and show plots
    figureList = TheScenario.end_scenario()
    return figureList


if __name__ == "__main__":
    run(True)
