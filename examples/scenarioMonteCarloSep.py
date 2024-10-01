#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

#
# Basilisk Integrated Test
#
# Purpose:  Integrated test of the MonteCarlo module.  Runs multiple
#           scenarioSepMomentumManagement with dispersed initial parameters
#


import inspect
import math
import os
import shutil

import matplotlib.pyplot as plt
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))


from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError, oneAxisSolarArrayPoint, rwMotorTorque,
                                    hingedRigidBodyPIDMotor, solarArrayReference, thrusterPlatformReference,
                                    thrusterPlatformState, thrustCMEstimation, torqueScheduler)
from Basilisk.simulation import (reactionWheelStateEffector, simpleNav, simpleMassProps, spacecraft,
                                 spinningBodyOneDOFStateEffector,
                                 spinningBodyTwoDOFStateEffector, thrusterStateEffector, facetSRPDynamicEffector)
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, simIncludeRW,
                                unitTestSupport, vizSupport, RigidBodyKinematics as rbk)

# import message declarations
from Basilisk.architecture import messaging

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import (UniformEulerAngleMRPDispersion, UniformDispersion, UniformVectorDispersion,
                                                       NormalVectorCartDispersion, InertiaTensorDispersion,
                                                       NormalVectorSingleAngleDispersion)

from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter
from bokeh.io import output_file, show
from bokeh.layouts import column

NUMBER_OF_RUNS = 6
VERBOSE = True


# Here are the name of some messages that we want to retain or otherwise use
# rwMotorTorqueMsgName = "rwMotorTorqueMsg"
guidMsgName = "guidMsg"
# transMsgName = "transMsg"
# rwSpeedMsgName = "rwSpeedMsg"
# voltMsgName = "voltMsg"
# rwOutName = ["rw1Msg", "rw2Msg", "rw3Msg", "rw4Msg"]


# We also will need the simulationTime and samplingTimes
simulationTime = macros.hour2nano(2)
simulationTimeStepDyn = macros.sec2nano(0.5)
simulationTimeStepFsw = macros.sec2nano(2)
simulationTimeStepPlt = macros.hour2nano(1)
numDataPoints = simulationTime / simulationTimeStepFsw
samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStepFsw, numDataPoints)


def run(saveFigures, case, show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        saveFigures (bool): flag if the scenario figures should be saved for html documentation
        case (int): Case 1 is normal MC, case 2 is initial condition run
        show_plots (bool): Determines if the script should display plots
    """

    # A MonteCarlo simulation can be created using the `MonteCarlo` module.
    # This module is used to execute monte carlo simulations, and access
    # retained data from previously executed MonteCarlo runs.

    # First, the `Controller` class is used in order to define the simulation
    monteCarlo = Controller()

    # Every MonteCarlo simulation must define a function that creates the `SimulationBaseClass` to
    # execute and returns it. Within this function, the simulation is created and configured
    monteCarlo.setSimulationFunction(createScenarioSepMomentumManagement)

    # Also, every MonteCarlo simulation must define a function which executes the simulation that was created.
    monteCarlo.setExecutionFunction(executeScenario)

    # A Monte Carlo simulation must define how many simulation runs to execute
    monteCarlo.setExecutionCount(NUMBER_OF_RUNS)

    # The simulations can have random seeds of each simulation dispersed randomly
    monteCarlo.setShouldDisperseSeeds(True)

    monteCarlo.setShowProgressBar(True)
    # Optionally set the number of cores to use
    # monteCarlo.setThreadCount(PROCESSES)

    # Whether to print more verbose information during the run
    monteCarlo.setVerbose(VERBOSE)

    # We set up where to retain the data to.
    dirName = "montecarlo_test" + str(os.getpid())
    monteCarlo.setArchiveDir(dirName)

    # Statistical dispersions can be applied to initial parameters using the MonteCarlo module
    # dispMRPInit = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
    dispOmegaInit = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
    dispMass = 'TaskList[0].TaskModels[0].hub.mHub'
    dispCoMOff = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
    dispInertia = 'TaskList[0].TaskModels[0].hub.IHubPntBc_B'
    dispRWAxis = []
    dispRWOmega = []
    dispRWInertia = []
    for idx in range(4):
        dispRWAxis.append(f"RW[{idx}].gsHat_B")
        dispRWOmega.append(f"RW[{idx}].Omega")
        dispRWInertia.append(f"RW[{idx}].Js")
    dispMassSA1 = 'TaskList[0].TaskModels[4].mass'
    dispMassSA2 = 'TaskList[0].TaskModels[5].mass'
    dispInertiaSA1 = 'TaskList[0].TaskModels[4].IPntSc_S'
    dispInertiaSA2 = 'TaskList[0].TaskModels[5].IPntSc_S'
    dispCMEstGuess = 'cmGuessOffset'
    dispThrMag = 'TaskList[0].TaskModels[7].thrusterData[0].MaxThrust'
    # dispThrAxis = 'TaskModels[7].thrusterData[0].thrDir_B'
    dispList = [dispOmegaInit, dispMass, dispInertia, dispCoMOff,
                dispMassSA1, dispMassSA2, dispInertiaSA1, dispInertiaSA2,
                dispCMEstGuess, dispThrMag] + dispRWAxis + dispRWOmega + dispRWInertia

    # Add dispersions with their dispersion type
    # monteCarlo.addDispersion(UniformEulerAngleMRPDispersion(dispMRPInit))
    monteCarlo.addDispersion(NormalVectorCartDispersion(dispOmegaInit, 0.0, 0.25 / 3.0 * np.pi / 180))
    monteCarlo.addDispersion(UniformDispersion(dispMass, ([2500.0 * 0.95, 2500.0 * 1.05])))
    monteCarlo.addDispersion(NormalVectorCartDispersion(dispCoMOff, [0.008, -0.010, 1.214], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))
    monteCarlo.addDispersion(InertiaTensorDispersion(dispInertia, stdAngle=1.0 / 3 * np.pi / 180))
    Js = 0.15915494309189535
    for idx in range(4):
        monteCarlo.addDispersion(NormalVectorSingleAngleDispersion(dispRWAxis[idx], phiStd=2.0 / 3 * np.pi / 180))
        monteCarlo.addDispersion(UniformDispersion(dispRWOmega[idx], ([-10.0, 10.0])))
        monteCarlo.addDispersion(UniformDispersion(dispRWInertia[idx], ([Js - 0.05*Js, Js + 0.05*Js])))
    monteCarlo.addDispersion(UniformDispersion(dispMassSA1, ([85 * 0.95, 85 * 1.05])))
    monteCarlo.addDispersion(UniformDispersion(dispMassSA2, ([85 * 0.95, 85 * 1.05])))
    monteCarlo.addDispersion(InertiaTensorDispersion(dispInertiaSA1, stdAngle=1.0 / 3 * np.pi / 180))
    monteCarlo.addDispersion(InertiaTensorDispersion(dispInertiaSA2, stdAngle=1.0 / 3 * np.pi / 180))
    monteCarlo.addDispersion(UniformVectorDispersion(dispCMEstGuess, bounds=[-0.05, 0.05]))
    monteCarlo.addDispersion(UniformDispersion(dispThrMag, ([0.27 * 0.95, 0.27 * 1.05])))

    # A `RetentionPolicy` is used to define what data from the simulation should be retained. A `RetentionPolicy`
    # is a list of messages and variables to log from each simulation run. It also has a callback,
    # used for plotting/processing the retained data.
    retentionPolicy = RetentionPolicy()
    # define the data to retain
    # retentionPolicy.addMessageLog(rwMotorTorqueMsgName, ["motorTorque"])
    retentionPolicy.addMessageLog(guidMsgName, ["sigma_BR", "omega_BR_B"])
    # retentionPolicy.addMessageLog(transMsgName, ["r_BN_N"])
    # retentionPolicy.addMessageLog(rwSpeedMsgName, ["wheelSpeeds"])
    # retentionPolicy.addMessageLog(voltMsgName, ["voltage"])
    # for msgName in rwOutName:
    #     retentionPolicy.addMessageLog(msgName, ["u_current"])
    if show_plots:
        # plot data only if show_plots is true, otherwise just retain
        retentionPolicy.setDataCallback(plotSim)
    if saveFigures:
        # plot data only if show_plots is true, otherwise just retain
        retentionPolicy.setDataCallback(plotSim)
    monteCarlo.addRetentionPolicy(retentionPolicy)

    if case == 1:
        # After the monteCarlo run is configured, it is executed.
        # This method returns the list of jobs that failed.
        failures = monteCarlo.executeSimulations()

        assert len(failures) == 0, "No runs should fail"

        # Now in another script (or the current one), the data from this simulation can be easily loaded.
        # This demonstrates loading it from disk
        monteCarloLoaded = Controller.load(dirName)

        # Then retained data from any run can then be accessed in the form of a dictionary
        # with two sub-dictionaries for messages and variables:
        retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
        assert retainedData is not None, "Retained data should be available after execution"
        assert "messages" in retainedData, "Retained data should retain messages"
        assert guidMsgName + ".sigma_BR" in retainedData["messages"], "Retained messages should exist"

        # We also can rerun a case using the same parameters and random seeds
        # If we rerun a properly set-up run, it should output the same data.
        # Here we test that if we rerun the case the data doesn't change
        oldOutput = retainedData["messages"][guidMsgName + ".sigma_BR"]

        # Rerunning the case shouldn't fail
        failed = monteCarloLoaded.reRunCases([NUMBER_OF_RUNS-1])
        assert len(failed) == 0, "Should rerun case successfully"

        # Now access the newly retained data to see if it changed
        retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
        newOutput = retainedData["messages"][guidMsgName + ".sigma_BR"]
        for k1, v1 in enumerate(oldOutput):
            for k2, v2 in enumerate(v1):
                assert math.fabs(oldOutput[k1][k2] - newOutput[k1][k2]) < .001, \
                "Outputs shouldn't change on runs if random seeds are same"

        # We can also access the initial parameters
        # The random seeds should differ between runs, so we will test that
        params1 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-1)
        params2 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-2)
        assert "TaskList[0].TaskModels[0].RNGSeed" in params1, "random number seed should be applied"
        for dispName in dispList:
            assert dispName in params1, "dispersion should be applied"
            # assert two different runs had different parameters.
            assert params1[dispName] != params2[dispName], "dispersion should be different in each run"

        # Now we execute our callback for the retained data.
        # For this run, that means executing the plot.
        # We can plot only runs 4,6,7 overlapped
        # monteCarloLoaded.executeCallbacks([4,6,7])
        # or execute the plot on all runs
        monteCarloLoaded.executeCallbacks()

    #########################################################
    if case == 2:
        # Now run initial conditions
        icName = path + "/Support/run_MC_IC"
        monteCarlo.setICDir(icName)
        monteCarlo.setICRunFlag(True)
        numberICs = 3
        monteCarlo.setExecutionCount(numberICs)

        # Rerunning the case shouldn't fail
        runsList = list(range(numberICs))
        failed = monteCarlo.runInitialConditions(runsList)
        assert len(failed) == 0, "Should run ICs successfully"

        # monteCarlo.executeCallbacks([4,6,7])
        runsList = list(range(numberICs))
        monteCarloLoaded.executeCallbacks(runsList)

        # And possibly show the plots
        if show_plots:
            plt.show()
            # close the plots being saved off to avoid over-writing old and new figures
            plt.close("all")

        # Now we clean up data from this test
        os.remove(icName + '/' + 'MonteCarlo.data')
        for i in range(numberICs):
            os.remove(icName + '/' + 'run' + str(i) + '.data')
        assert not os.path.exists(icName + '/' + 'MonteCarlo.data'), "No leftover data should exist after the test"

    # Now we clean up data from this test
    # shutil.rmtree(dirName)
    # assert not os.path.exists(dirName), "No leftover data should exist after the test"

    # And possibly show the plots
    if show_plots:
        print("Test concluded, showing plots now via matplot...")
        plt.show()
        # close the plots being saved off to avoid over-writing old and new figures
        plt.close("all")

    return dirName

# This function creates the simulation to be executed in parallel.
def createScenarioSepMomentumManagement():

    momentumManagement = True
    cmEstimation = True

    # Create simulation variable names
    fswTask = "fswTask"
    pltRefTask = "pltRefTask"
    dynTask = "dynTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time and integration update time
    dynProcess.addTask(scSim.CreateNewTask(dynTask, simulationTimeStepDyn))
    dynProcess.addTask(scSim.CreateNewTask(pltRefTask, simulationTimeStepPlt))
    dynProcess.addTask(scSim.CreateNewTask(fswTask, simulationTimeStepFsw))

    #
    # setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Spacecraft"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTask, scObject, 1)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # Next a series of gravitational bodies are included
    gravBodies = gravFactory.createBodies(['sun'])
    gravBodies['sun'].isCentralBody = True
    mu = gravBodies['sun'].mu

    # The configured gravitational bodies are added to the spacecraft dynamics with the usual command:
    gravFactory.addBodiesTo(scObject)

    # Next, the default SPICE support module is created and configured.
    timeInitString = "2023 OCTOBER 22 00:00:00.0"

    # The following is a support macro that creates a `gravFactory.spiceObject` instance
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # Sun is gravity center
    gravFactory.spiceObject.zeroBase = 'Sun'

    # The SPICE object is added to the simulation task list.
    scSim.AddModelToTask(fswTask, gravFactory.spiceObject, 2)

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 150e9      # meters
    oe.e = 0.001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = -135.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN                          # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN                          # m/s - v_BN_N
    scObject.hub.sigma_BNInit = [0, 0., 0.]              # MRP set to customize initial inertial attitude
    scObject.hub.omega_BN_BInit = [[0.], [0.], [0.]]      # rad/s - omega_CN_B

    # define the simulation inertia
    I = [ 1725,    -5,   -12,
            -5,  5525,    43,
            -12,   43,  4810]
    scObject.hub.mHub = 2500  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.008], [-0.010], [1.214]]  # [m] - position vector of hub CM relative to the body-fixed point B
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # add RW devices
    #
    # Make RW factory instance
    rwFactory = simIncludeRW.rwFactory()

    # specify RW momentum capacity
    maxRWMomentum = 100.  # Nms

    # Define orthogonal RW pyramid
    # -- Pointing directions
    rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * macros.D2R
    rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * macros.D2R
    rwPosVector = [[0.8, 0.8, 1.8],
                    [0.8, -0.8, 1.8],
                    [-0.8, -0.8, 1.8],
                    [-0.8, 0.8, 1.8]]

    Gs = []
    for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
        gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
        Gs.append(gsHat)
        rwFactory.create('Honeywell_HR16', gsHat, maxMomentum=maxRWMomentum, rWB_B=posVector, Omega=0.)

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    scSim.RW = []
    for idx in range(numRW):
        scSim.RW.append(rwFactory.rwList[f"RW{idx+1}"])

    # add RW object array to the simulation process
    scSim.AddModelToTask(dynTask, rwStateEffector, 2)

    # Setup the FSW RW configuration message.
    scSim.fswRwConfigMsg = rwFactory.getConfigMessage()

    # add the simple Navigation sensor module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTask, sNavObject)

    # Set up the simple mass props object
    simpleMassPropsObject = simpleMassProps.SimpleMassProps()
    scSim.AddModelToTask(dynTask, simpleMassPropsObject)

    # Set up the rotating solar arrays
    numRSA = 2
    RSAList = []
    # 1st solar array
    RSAList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
    scSim.AddModelToTask(dynTask, RSAList[0])
    RSAList[0].r_SB_B = [0.75, 0.0, 0.45]
    RSAList[0].r_ScS_S = [0.0, 3.75, 0.0]
    RSAList[0].sHat_S = [0, 1, 0]
    RSAList[0].dcm_S0B = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
    RSAList[0].IPntSc_S = [[250.0, 0.0, 0.0],
                           [0.0, 250.0, 0.0],
                           [0.0, 0.0, 500.0]]
    RSAList[0].mass = 85
    RSAList[0].k = 0
    RSAList[0].c = 0
    RSAList[0].thetaInit = 0
    RSAList[0].thetaDotInit = 0
    RSAList[0].ModelTag = "solarArray1"
    scObject.addStateEffector(RSAList[0])
    # 2nd solar array
    RSAList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
    scSim.AddModelToTask(dynTask, RSAList[1])
    RSAList[1].r_SB_B = [-0.75, 0.0, 0.45]
    RSAList[1].r_ScS_S = [0.0, 3.75, 0.0]
    RSAList[1].sHat_S = [0, 1, 0]
    RSAList[1].dcm_S0B = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    RSAList[1].IPntSc_S = [[250.0, 0.0, 0.0],
                           [0.0, 250.0, 0.0],
                           [0.0, 0.0, 500.0]]
    RSAList[1].mass = 85
    RSAList[1].k = 0
    RSAList[1].c = 0
    RSAList[1].thetaInit = 0
    RSAList[1].thetaDotInit = 0
    RSAList[1].ModelTag = "solarArray2"
    scObject.addStateEffector(RSAList[1])

    # Set up the dual-gimbaled platform
    platform = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    scSim.AddModelToTask(dynTask, platform)
    platform.theta1Init = 0
    platform.theta1DotInit = 0
    platform.theta2Init = 0
    platform.theta2DotInit = 0
    platform.mass1 = 0
    platform.mass2 = 10
    platform.k1 = 0
    platform.k2 = 0
    platform.r_S1B_B = [0, 0, 0]
    platform.r_S2S1_S1 = [0, 0, 0]
    platform.r_Sc1S1_S1 = [0, 0, 0]
    platform.r_Sc2S2_S2 = [0, 0, 0]
    platform.s1Hat_S1 = [1, 0, 0]
    platform.s2Hat_S2 = [0, 1, 0]
    platform.IS1PntSc1_S1 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
    platform.IS2PntSc2_S2 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
    platform.dcm_S10B = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    platform.dcm_S20S1 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    platform.ModelTag = "platform1"
    scObject.addStateEffector(platform)

    # Set up the SEP thruster
    sepThruster = thrusterStateEffector.ThrusterStateEffector()
    scSim.AddModelToTask(dynTask, sepThruster)
    thruster = thrusterStateEffector.THRSimConfig()
    r_TF_F = [0, 0, 0]  # Thruster application point in F frame coordinates
    tHat_F = [0, 0, 1]  # Thrust unit direction vector in F frame coordinates
    thruster.thrLoc_B = r_TF_F
    thruster.thrDir_B = tHat_F
    thruster.MaxThrust = 0.27
    thruster.steadyIsp = 1600
    thruster.MinOnTime = 0.006
    thruster.cutoffFrequency = 5
    sepThruster.addThruster(thruster, platform.spinningBodyConfigLogOutMsgs[1])
    sepThruster.kappaInit = messaging.DoubleVector([0.0])
    sepThruster.ModelTag = "sepThruster"
    scObject.addStateEffector(sepThruster)

    # Write THR Config Msg
    THRConfig = messaging.THRConfigMsgPayload()
    THRConfig.rThrust_B = r_TF_F
    THRConfig.tHatThrust_B = tHat_F
    THRConfig.maxThrust = thruster.MaxThrust
    scSim.thrConfigFMsg = messaging.THRConfigMsg().write(THRConfig)

    # Set up the SRP dynamic effector
    SRP = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    SRP.numFacets = 10
    SRP.numArticulatedFacets = 4
    scSim.AddModelToTask(dynTask, SRP)

    # Define the spacecraft geometry for populating the FacetedSRPSpacecraftGeometryData structure in the SRP module
    # Define the facet surface areas
    lenXHub = 1.50  # [m]
    lenYHub = 1.8  # [m]
    lenZHub = 2.86  # [m]
    area2 = np.pi*(0.5 * 7.262)*(0.5 * 7.262)  # [m^2]
    facetAreas = [lenYHub * lenZHub, lenXHub * lenZHub, lenYHub * lenZHub, lenXHub * lenZHub, lenXHub * lenYHub, lenXHub * lenYHub, area2, area2, area2, area2]

    # Define the facet normals in B frame components
    facetNormal1 = np.array([1.0, 0.0, 0.0])
    facetNormal2 = np.array([0.0, 1.0, 0.0])
    facetNormal3 = np.array([-1.0, 0.0, 0.0])
    facetNormal4 = np.array([0.0, -1.0, 0.0])
    facetNormal5 = np.array([0.0, 0.0, 1.0])
    facetNormal6 = np.array([0.0, 0.0, -1.0])
    facetNormal7 = np.array([0.0, 1.0, 0.0])
    facetNormal8 = np.array([0.0, -1.0, 0.0])
    facetNormal9 = np.array([0.0, 1.0, 0.0])
    facetNormal10 = np.array([0.0, -1.0, 0.0])
    normals_B = [facetNormal1, facetNormal2, facetNormal3, facetNormal4, facetNormal5, facetNormal6, facetNormal7, facetNormal8, facetNormal9, facetNormal10]

    # Define the facet center of pressure locations with respect to point B in B frame components
    facetLoc1 = np.array([0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
    facetLoc2 = np.array([0.0, 0.5 * lenYHub, 0.5 * lenZHub])  # [m]
    facetLoc3 = np.array([-0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
    facetLoc4 = np.array([0.0, -0.5 * lenYHub, 0.5 * lenZHub])  # [m]
    facetLoc5 = np.array([0.0, 0.0, lenZHub])  # [m]
    facetLoc6 = np.array([0.0, 0.0, 0.0])  # [m]
    facetLoc7 = np.array([3.75 + 0.5 * lenXHub, 0.0, 0.45])  # [m]
    facetLoc8 = np.array([3.75 + 0.5 * lenXHub, 0.00, 0.45])  # [m]
    facetLoc9 = np.array([-(3.75 + 0.5 * lenXHub), 0.0, 0.45])  # [m]
    facetLoc10 = np.array([-(3.75 + 0.5 * lenXHub), 0.0, 0.45])  # [m]

    locationsPntB_B = [facetLoc1, facetLoc2, facetLoc3, facetLoc4, facetLoc5, facetLoc6, facetLoc7, facetLoc8, facetLoc9, facetLoc10]

    # Define facet articulation axes in B frame components
    rotAxes_B = [np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([0.0, 0.0, 0.0]),
                 np.array([1.0, 0.0, 0.0]),
                 np.array([1.0, 0.0, 0.0]),
                 np.array([-1.0, 0.0, 0.0]),
                 np.array([-1.0, 0.0, 0.0])]

    # Define the facet optical coefficients
    specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    # Populate the scGeometry structure with the facet information
    for i in range(len(facetAreas)):
        SRP.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i], rotAxes_B[i])

    SRP.ModelTag = "FacetSRP"
    SRP.addArticulatedFacet(RSAList[0].spinningBodyOutMsg)
    SRP.addArticulatedFacet(RSAList[0].spinningBodyOutMsg)
    SRP.addArticulatedFacet(RSAList[1].spinningBodyOutMsg)
    SRP.addArticulatedFacet(RSAList[1].spinningBodyOutMsg)
    scObject.addDynamicEffector(SRP)

    #
    #   setup the FSW algorithm modules
    #

    # Set up thruster platform state module
    pltState = thrusterPlatformState.thrusterPlatformState()
    pltState.ModelTag = "thrusterPlatformState"
    pltState.sigma_MB = np.array([0, 0, 0])
    pltState.r_BM_M = [0, 0, 0]
    pltState.r_FM_F = [0, 0, 0]
    scSim.AddModelToTask(fswTask, pltState, 30)

    # Set up the CM estimator module
    scSim.cmGuessOffset = [0.0, 0.0, 0.0]
    cmEstimator = thrustCMEstimation.ThrustCMEstimation()
    cmEstimator.ModelTag = "cmEstimator"
    cmEstimator.attitudeTol = 1e-6
    cmEstimator.r_CB_B = np.array([0.113244, 0.025605, 1.239834]) + np.array(scSim.cmGuessOffset)
    cmEstimator.P0 = [0.0025, 0.0025, 0.0025]
    cmEstimator.R0 = [4e-8, 4e-8, 4e-8]
    scSim.AddModelToTask(fswTask, cmEstimator, None, 29)

    # create the FSW vehicle configuration message for CoM
    vehicleConfigData = messaging.VehicleConfigMsgPayload()
    vehicleConfigData.CoM_B = np.array([0.113244, 0.025605, 1.239834]) + np.array(scSim.cmGuessOffset)    # use the same initial CoM guess as the cmEstimator module
    scSim.vcMsg_CoM = messaging.VehicleConfigMsg_C().write(vehicleConfigData)

    # create the FSW vehicle configuration message for inertias
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I       # use the same inertia in the FSW algorithm as in the simulation
    scSim.vcMsg_I = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Set up platform reference module
    pltReference = thrusterPlatformReference.thrusterPlatformReference()
    pltReference.ModelTag = 'thrusterPlatformReference'
    pltReference.sigma_MB = pltState.sigma_MB
    pltReference.r_BM_M = pltState.r_BM_M
    pltReference.r_FM_F = pltState.r_FM_F
    pltReference.theta1Max = np.pi/12
    pltReference.theta2Max = np.pi/12
    if momentumManagement:
        pltReference.K = 2.5e-4
    else:
        pltReference.K = 0
    pltReference.Ki = 0
    scSim.AddModelToTask(pltRefTask, pltReference, 28)

    # Set up the two platform PD controllers
    pltController = []
    for item in range(2):
        pltController.append(hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotor())
        pltController[item].ModelTag = "PltMototorGimbal"+str(item+1)
        pltController[item].K = 0.5
        pltController[item].P = 3
        scSim.AddModelToTask(fswTask, pltController[item], 27)

    # Set up the torque scheduler module
    pltTorqueScheduler = torqueScheduler.torqueScheduler()
    pltTorqueScheduler.ModelTag = "TorqueScheduler"
    pltTorqueScheduler.tSwitch = 60
    pltTorqueScheduler.lockFlag = 0
    scSim.AddModelToTask(fswTask, pltTorqueScheduler, 26)

    # Set up attitude guidance module
    sepPoint = oneAxisSolarArrayPoint.oneAxisSolarArrayPoint()
    sepPoint.ModelTag = "sepPointGuidance"
    sepPoint.a1Hat_B = [1, 0, 0]          # solar array drive axis
    sepPoint.a2Hat_B = [0, 1, 0]          # antiparallel direction to the sensitive surface
    sepPoint.hHat_N = [1, 0, 0]           # random inertial thrust direction
    scSim.AddModelToTask(fswTask, sepPoint, 25)

    # Set up the solar array reference modules
    saReference = []
    for item in range(numRSA):
        saReference.append(solarArrayReference.solarArrayReference())
        saReference[item].ModelTag = "SolarArrayReference"+str(item+1)
        saReference[item].a1Hat_B = [(-1)**item, 0, 0]
        saReference[item].a2Hat_B = [0, 1, 0]
        scSim.AddModelToTask(fswTask, saReference[item], 24)

    # Set up solar array controller modules
    saController = []
    for item in range(numRSA):
        saController.append(hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotor())
        saController[item].ModelTag = "SolarArrayMotor"+str(item+1)
        saController[item].K = 1.25
        saController[item].P = 50
        saController[item].I = 3e-3
        scSim.AddModelToTask(fswTask, saController[item], 23)

    # Set up attitude tracking error
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "AttitudeTrackingError"
    scSim.AddModelToTask(fswTask, attError, 22)

    # Set up the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    mrpControl.Ki = 1e-5
    mrpControl.P = 275
    mrpControl.K = 9
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1
    mrpControl.controlLawType = 1
    scSim.AddModelToTask(fswTask, mrpControl, 21)

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    rwMotorTorqueObj.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    scSim.AddModelToTask(fswTask, rwMotorTorqueObj, 20)

    # Configure thruster on-time message
    thrOnTimeMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    thrOnTimeMsgData.OnTimeRequest = [3600*4*30]
    thrOnTimeMsg = messaging.THRArrayOnTimeCmdMsg().write(thrOnTimeMsgData)

    # Write cmEstimator output msg to the standalone message vcMsg_CoM
    # This is needed because platformReference runs on its own task at a different frequency,
    # but it receives inputs and provides outputs to modules that run on the main flight software task
    messaging.VehicleConfigMsg_C_addAuthor(cmEstimator.vehConfigOutMsgC, scSim.vcMsg_CoM)

    # Connect messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    sNavObject.sunStateInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    simpleMassPropsObject.scMassPropsInMsg.subscribeTo(scObject.scMassOutMsg)
    RSAList[0].motorTorqueInMsg.subscribeTo(saController[0].motorTorqueOutMsg)
    RSAList[1].motorTorqueInMsg.subscribeTo(saController[1].motorTorqueOutMsg)
    platform.motorTorqueInMsg.subscribeTo(pltTorqueScheduler.motorTorqueOutMsg)
    platform.motorLockInMsg.subscribeTo(pltTorqueScheduler.effectorLockOutMsg)
    SRP.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    pltState.thrusterConfigFInMsg.subscribeTo(scSim.thrConfigFMsg)
    pltState.hingedRigidBody1InMsg.subscribeTo(platform.spinningBodyOutMsgs[0])
    pltState.hingedRigidBody2InMsg.subscribeTo(platform.spinningBodyOutMsgs[1])
    cmEstimator.thrusterConfigBInMsg.subscribeTo(pltState.thrusterConfigBOutMsg)
    cmEstimator.intFeedbackTorqueInMsg.subscribeTo(mrpControl.intFeedbackTorqueOutMsg)
    cmEstimator.attGuidInMsg.subscribeTo(attError.attGuidOutMsg)
    cmEstimator.vehConfigInMsg.subscribeTo(simpleMassPropsObject.vehicleConfigOutMsg)
    if cmEstimation:
        pltReference.vehConfigInMsg.subscribeTo(scSim.vcMsg_CoM)                                 # connect to this msg for estimated CM
    else:
        pltReference.vehConfigInMsg.subscribeTo(simpleMassPropsObject.vehicleConfigOutMsg) # connect to this msg for exact CM information
    pltReference.thrusterConfigFInMsg.subscribeTo(scSim.thrConfigFMsg)
    pltReference.rwConfigDataInMsg.subscribeTo(scSim.fswRwConfigMsg)
    pltReference.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    pltTorqueScheduler.motorTorque1InMsg.subscribeTo(pltController[0].motorTorqueOutMsg)
    pltTorqueScheduler.motorTorque2InMsg.subscribeTo(pltController[1].motorTorqueOutMsg)
    sepPoint.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    sepPoint.bodyHeadingInMsg.subscribeTo(pltReference.bodyHeadingOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(sepPoint.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(scSim.vcMsg_I)
    mrpControl.rwParamsInMsg.subscribeTo(scSim.fswRwConfigMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(scSim.fswRwConfigMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    for item in range(numRSA):
        saReference[item].attNavInMsg.subscribeTo(sNavObject.attOutMsg)
        saReference[item].attRefInMsg.subscribeTo(sepPoint.attRefOutMsg)
        saReference[item].hingedRigidBodyInMsg.subscribeTo(RSAList[item].spinningBodyOutMsg)
        saController[item].hingedRigidBodyInMsg.subscribeTo(RSAList[item].spinningBodyOutMsg)
        saController[item].hingedRigidBodyRefInMsg.subscribeTo(saReference[item].hingedRigidBodyRefOutMsg)
    for item in range(2):
        pltController[item].hingedRigidBodyInMsg.subscribeTo(platform.spinningBodyOutMsgs[item])
    pltController[0].hingedRigidBodyRefInMsg.subscribeTo(pltReference.hingedRigidBodyRef1OutMsg)
    pltController[1].hingedRigidBodyRefInMsg.subscribeTo(pltReference.hingedRigidBodyRef2OutMsg)
    sepThruster.cmdsInMsg.subscribeTo(thrOnTimeMsg)

    # store the msg recorder modules in a dictionary list so the retention policy class can pull the data
    # when the simulation ends
    scSim.msgRecList = {}

    # scSim.msgRecList[rwMotorTorqueMsgName] = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    # scSim.AddModelToTask(dynTask, scSim.msgRecList[rwMotorTorqueMsgName])

    scSim.msgRecList[guidMsgName] = attError.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, scSim.msgRecList[guidMsgName])

    # scSim.msgRecList[transMsgName] = sNavObject.transOutMsg.recorder(samplingTime)
    # scSim.AddModelToTask(simTaskName, scSim.msgRecList[transMsgName])
    #
    # scSim.msgRecList[rwSpeedMsgName] = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    # scSim.AddModelToTask(dynTask, scSim.msgRecList[rwSpeedMsgName])
    #
    # scSim.msgRecList[voltMsgName] = fswRWVoltage.voltageOutMsg.recorder(samplingTime)
    # scSim.AddModelToTask(simTaskName, scSim.msgRecList[voltMsgName])

    # c = 0
    # for msgName in rwOutName:
    #     scSim.msgRecList[msgName] = rwStateEffector.rwOutMsgs[c].recorder(samplingTime)
    #     scSim.AddModelToTask(dynTask, scSim.msgRecList[msgName])
    #     c += 1

    # This is a hack because of a bug in Basilisk... leave this line it keeps
    # variables from going out of scope after this function returns
    scSim.additionalReferences = [scObject, gravBodies['sun'], rwMotorTorqueObj, mrpControl, attError]

    return scSim


def executeScenario(sim):
    #   initialize Simulation
    sim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    sim.ConfigureStopTime(simulationTime)
    sim.ExecuteSimulation()


# This method is used to plot the retained data of a simulation.
# It is called once for each run of the simulation, overlapping the plots
def plotSim(data, retentionPolicy):
    #   retrieve the logged data
    # dataUsReq = data["messages"][rwMotorTorqueMsgName + ".motorTorque"][:,1:]
    dataSigmaBR = data["messages"][guidMsgName + ".sigma_BR"][:,1:]
    # dataOmegaBR = data["messages"][guidMsgName + ".omega_BR_B"][:,1:]
    # dataPos = data["messages"][transMsgName + ".r_BN_N"][:,1:]
    # dataOmegaRW = data["messages"][rwSpeedMsgName + ".wheelSpeeds"][:,1:]
    # dataVolt = data["messages"][voltMsgName + ".voltage"][:,1:]
    # dataRW = []
    # for msgName in rwOutName:
    #     dataRW.append(data["messages"][msgName+".u_current"][:,1:])
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #

    timeData = data["messages"][guidMsgName + ".sigma_BR"][:,0] * macros.NANO2MIN

    figureList = {}
    plt.figure(1)
    pltName = 'AttitudeError'
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 label='Run ' + str(data["index"]) + r' $\sigma_'+str(idx)+'$')
    # plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList[pltName] = plt.figure(1)

    # plt.figure(2)
    # pltName = 'RWMotorTorque'
    # for idx in range(3):
    #     plt.plot(timeData, dataUsReq[:, idx],
    #              '--',
    #              label='Run ' + str(data["index"]) + r' $\hat u_{s,'+str(idx)+'}$')
    #     plt.plot(timeData, dataRW[idx],
    #              label='Run ' + str(data["index"]) + ' $u_{s,' + str(idx) + '}$')
    # # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('RW Motor Torque (Nm)')
    # figureList[pltName] = plt.figure(2)

    # plt.figure(3)
    # pltName = 'RateTrackingError'
    # for idx in range(3):
    #     plt.plot(timeData, dataOmegaBR[:, idx],
    #              label='Run ' + str(data["index"]) + r' $\omega_{BR,'+str(idx)+'}$')
    # # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('Rate Tracking Error (rad/s) ')
    # figureList[pltName] = plt.figure(3)
    #
    # plt.figure(4)
    # pltName = 'RWSpeed'
    # for idx in range(len(rwOutName)):
    #     plt.plot(timeData, dataOmegaRW[:, idx]/macros.RPM,
    #              label='Run ' + str(data["index"]) + r' $\Omega_{'+str(idx)+'}$')
    # # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('RW Speed (RPM) ')
    # figureList[pltName] = plt.figure(4)

    # plt.figure(5)
    # pltName = 'RWVoltage'
    # for idx in range(len(rwOutName)):
    #     plt.plot(timeData, dataVolt[:, idx],
    #              label='Run ' + str(data["index"]) + ' $V_{' + str(idx) + '}$')
    # # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('RW Voltage (V) ')
    # figureList[pltName] = plt.figure(5)

    return figureList


# This statement below ensures that the unit test script can be run as a
# # stand-along python script
# Run this script with the command:
# python scenarioMonteCarloAttRW.py --bokeh-server
#
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--bokeh-server":
        from bokeh.server.server import Server
        from bokeh.application import Application
        from bokeh.application.handlers.function import FunctionHandler

        def bk_worker(doc):
            # Run the Monte Carlo simulation and get the directory name
            dirName = run(saveFigures=True, case=1, show_plots=False)
            
            # Create the Bokeh application
            plotter = MonteCarloPlotter(dirName)
            plotter.load_data([
                # rwMotorTorqueMsgName + ".motorTorque",
                guidMsgName + ".sigma_BR",
                # guidMsgName + ".omega_BR_B",
                # rwSpeedMsgName + ".wheelSpeeds",
                # voltMsgName + ".voltage"
            ])
            downsampled_data = plotter.get_downsampled_plots()
            
            # Add the plots to the document
            for variable, plots in plotter.plots.items():
                for plot in plots:
                    doc.add_root(plot)

        server = Server({'/': Application(FunctionHandler(bk_worker))})
        server.start()
        print('Opening Bokeh application on http://localhost:5006/')
        server.io_loop.add_callback(server.show, "/")
        server.io_loop.start()
    else:
        run(saveFigures=True, case=1, show_plots=True)
