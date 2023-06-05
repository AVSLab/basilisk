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

This scenario uses the OpNav FSW stack to perform both pointing towards the target planet, and Orbit Determination.
As the primary scenario for OpNav, it will be detailed more carefully.

The orbit is a 18,000km orbit around Mars, with eccentricity of 0.6, and is pictured alongside the measurements:

.. image:: /_images/static/Orbit_OpNav.svg
   :width: 500px
   :align: center

Hough Circles is the image processing method used. After processing measurements through a Orbit Determination Filter,
spacecraft position and velocity estimates are pictured in the Mars-frame:

    .. image:: /_images/static/Filterpos1_1.svg
       :width: 300
    .. image:: /_images/static/Filtervel1_1.svg
       :width: 300
    .. image:: /_images/static/Filterpos2_1.svg
       :width: 300
    .. image:: /_images/static/Filtervel2_1.svg
       :width: 300
    .. image:: /_images/static/Filterpos3_1.svg
       :width: 300
    .. image:: /_images/static/Filtervel3_1.svg
       :width: 300



More details can be found in Chapter 2 of `Thibaud Teil's PhD thesis <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.

The script can be run at full length by calling::

    python3 scenario_OpNavAttOD.py

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
        self.filterUse = "relOD"

        # declare additional class variables
        self.opNavRec = None
        self.circlesRec = None
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
        rError= np.array([10000.,10000., -10000])
        vError= np.array([100, -10, 10])
        MRP= [0,0,0]
        if self.filterUse =="relOD":
            self.masterSim.get_FswModel().relativeODData.stateInit = (rN + rError).tolist() + (vN + vError).tolist()
        if self.filterUse == "bias":
            self.masterSim.get_FswModel().relativeODData.stateInit = (rN + rError).tolist() + (vN + vError).tolist() + bias
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = rN
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = vN
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[MRP[0]], [MRP[1]], [MRP[2]]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.masterSim.get_FswModel().relativeODData.qNoise = qNoiseIn.reshape(36).tolist()
        self.masterSim.get_FswModel().imageProcessing.noiseSF = 1
        self.masterSim.get_FswModel().relativeODData.noiseSF = 5

        self.masterSim.get_DynModel().cameraMod.cameraIsOn = 1
        # Camera noise params
        # self.masterSim.get_DynModel().cameraMod.gaussian = 5
        # self.masterSim.get_DynModel().cameraMod.darkCurrent = 0.5
        # self.masterSim.get_DynModel().cameraMod.saltPepper = 1
        self.masterSim.get_DynModel().cameraMod.cosmicRays = 1
        # self.masterSim.get_DynModel().cameraMod.blurParam = 5

    def log_outputs(self):
        # Dynamics process outputs: log messages below if desired.
        FswModel = self.masterSim.get_FswModel()
        DynModel = self.masterSim.get_DynModel()

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep

        if self.filterUse == "relOD":
            self.filtRec = FswModel.relativeODData.filtDataOutMsg.recorder(samplingTime)
            self.masterSim.AddModelToTask(DynModel.taskName, self.filtRec)
            self.opNavRec = FswModel.opnavMsg.recorder(samplingTime)
            self.masterSim.AddModelToTask(DynModel.taskName, self.opNavRec)
        if self.filterUse == "bias":
            self.filtRec = FswModel.pixelLineFilterData.filtDataOutMsg.recorder(samplingTime)
            self.masterSim.AddModelToTask(DynModel.taskName, self.filtRec)

        self.scRec = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.circlesRec = FswModel.opnavCirclesMsg.recorder(samplingTime)
        self.masterSim.AddModelToTask(DynModel.taskName, self.scRec)
        self.masterSim.AddModelToTask(DynModel.taskName, self.circlesRec)

        return

    def pull_outputs(self, showPlots):
        ## Spacecraft true states
        position_N = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.r_BN_N)
        velocity_N = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.v_BN_N)

        ## Attitude
        sigma_BN = unitTestSupport.addTimeColumn(self.scRec.times(), self.scRec.sigma_BN)

        ## Image processing
        circleCenters = unitTestSupport.addTimeColumn(self.circlesRec.times(), self.circlesRec.circlesCenters)
        circleRadii = unitTestSupport.addTimeColumn(self.circlesRec.times(), self.circlesRec.circlesRadii)
        validCircle = unitTestSupport.addTimeColumn(self.circlesRec.times(), self.circlesRec.valid)
        if self.filterUse == "bias":
            NUM_STATES = 9
            ## Navigation results
            navState = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.state)
            navCovar = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.covar)
            navPostFits = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.postFitRes)
        if self.filterUse == "relOD":
            NUM_STATES = 6
            ## Navigation results
            navState = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.state)
            navCovar = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.covar)
            navPostFits = unitTestSupport.addTimeColumn(self.filtRec.times(), self.filtRec.postFitRes)
            measPos = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.r_BN_N)
            r_C = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.r_BN_C)
            measCovar = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.covar_N)
            covar_C = unitTestSupport.addTimeColumn(self.opNavRec.times(), self.opNavRec.covar_C)

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
        pixCovar = np.ones([len(circleCenters[:,0]), 3*3+1])
        pixCovar[:,0] = circleCenters[:,0]
        pixCovar[:,1:]*=np.array([1,0,0,0,1,0,0,0,2])
        if self.filterUse == "relOD":
            measError = np.full([len(measPos[:,0]), 4], np.nan)
            measError[:,0] = measPos[:,0]
            measError_C = np.full([len(measPos[:,0]), 5], np.nan)
            measError_C[:,0] = measPos[:,0]
        trueRhat_C = np.full([len(circleCenters[:,0]), 4], np.nan)
        trueR_C = np.full([len(circleCenters[:,0]), 4], np.nan)
        trueCircles = np.full([len(circleCenters[:,0]), 4], np.nan)
        trueCircles[:,0] = circleCenters[:,0]
        trueRhat_C[:,0] = circleCenters[:,0]
        trueR_C[:,0] = circleCenters[:,0]
        truth = np.zeros([len(position_N[:,0]), 7])
        truth[:,0:4] = np.copy(position_N)
        truth[:,4:7] = np.copy(velocity_N[:,1:])

        centerBias = np.copy(circleCenters)
        radBias = np.copy(circleRadii)

        switchIdx = 0

        Rmars = 3396.19*1E3
        for j in range(len(stateError[:, 0])):
            if stateError[j, 0] in navState[:, 0]:
                stateError[j, 1:4] -= navState[j - switchIdx, 1:4]
                stateError[j, 4:] -= navState[j - switchIdx, 4:]
            else:
                stateError[j, 1:] = np.full(NUM_STATES, np.nan)
                switchIdx += 1
        for i in range(len(circleCenters[:,0])):
            if circleCenters[i,1:].any() > 1E-8 or circleCenters[i,1:].any() < -1E-8:
                trueR_C[i, 1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[i + switchIdx, 1:4])),
                                            position_N[i + switchIdx, 1:4])
                trueRhat_C[i,1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[i +switchIdx, 1:4])) ,position_N[i +switchIdx, 1:4])/np.linalg.norm(position_N[i +switchIdx, 1:4])
                trueCircles[i,3] = focal*np.tan(np.arcsin(Rmars/np.linalg.norm(position_N[i,1:4])))/pixelSize[0]
                trueRhat_C[i,1:] *= focal/trueRhat_C[i,3]
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0]/2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1]/2 - 0.5
                if self.filterUse == "bias":
                    centerBias[i,1:3] = np.round(navState[i, 7:9])
                    radBias[i,1] = np.round(navState[i, -1])
                if self.filterUse == "relOD":
                    measError[i, 1:4] = position_N[i +switchIdx, 1:4] - measPos[i, 1:4]
                    measError_C[i, 4] = np.linalg.norm(position_N[i +switchIdx, 1:4]) - np.linalg.norm(r_C[i, 1:4])
                    measError_C[i, 1:4] = trueRhat_C[i,1:] - r_C[i, 1:4]/np.linalg.norm(r_C[i, 1:4])
            else:
                if self.filterUse == "relOD":
                    measCovar[i,1:] = np.full(3*3, np.nan)
                    covar_C[i, 1:] = np.full(3 * 3, np.nan)
        navCovarLong[switchIdx:,:] = np.copy(navCovar)

        BSK_plt.plot_TwoOrbits(position_N[switchIdx:,:], measPos)
        BSK_plt.diff_vectors(trueR_C, r_C, validCircle, "Circ")
        BSK_plt.nav_percentages(truth[switchIdx:,:], navState, navCovar, validCircle, "Circ")

        BSK_plt.plot_cirlces(circleCenters, circleRadii, validCircle, sizeOfCam)
        BSK_plt.plotStateCovarPlot(stateError, navCovarLong)
        if self.filterUse == "bias":
            circleCenters[i,1:] += centerBias[i,1:]
            circleRadii[i,1:] += radBias[i,1:]
            BSK_plt.plotPostFitResiduals(navPostFits, pixCovar)
        BSK_plt.imgProcVsExp(trueCircles, circleCenters, circleRadii, np.array(sizeOfCam))
        if self.filterUse == "relOD":
            BSK_plt.plotPostFitResiduals(navPostFits, measCovar)
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
    TheScenario.filterUse = "relOD"
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

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
    if TheScenario.filterUse == "bias":
        TheScenario.masterSim.modeRequest = 'OpNavAttODB'
    if TheScenario.filterUse == "relOD":
        TheScenario.masterSim.modeRequest = 'OpNavAttOD'
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
