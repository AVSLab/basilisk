import sys
import os
import inspect  # Don't worry about this, standard stuff plus file discovery

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

from MonteCarlo.controller import Controller
import MonteCarlo.dispersions as dispersions

import SimulationBaseClass
import numpy as np
import shutil

# import simulation related support
import spacecraftPlus
import orbitalMotion
import gravityEffector
import simIncludeGravity
import macros
from math import fabs

import pytest
import unitTestSupport  # general support file with common unit test functions

NUMBER_OF_RUNS = 20
LENGTH_SHRINKAGE_FACTOR = 50 # make the simulations faster by dividing the length of the sim by this.
VERBOSE = True
PROCESSES=8


def myCreationFunction():
    ''' function that returns a functional simulation '''
    #  Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()
    sim.TotalSim.terminateSimulation()

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  create the simulation process
    dynProcess = sim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(sim.CreateNewTask(simTaskName, simulationTimeStep))

    #   setup the simulation tasks/objects
    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    # add spacecraftPlus object to the simulation process
    sim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # Earth
    simIncludeGravity.addEarth()
    # ensure this is the central gravitational body
    simIncludeGravity.gravBodyList[-1].isCentralBody = True
    # useSphericalHarmonics:
    simIncludeGravity.gravBodyList[-1].useSphericalHarmParams = True
    gravityEffector.loadGravFromFile(bskPath + 'External/LocalGravData/GGM03S-J2-only.txt', simIncludeGravity.gravBodyList[-1].spherHarm, 2)
    mu = simIncludeGravity.gravBodyList[-1].mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    # orbitCase is 'GEO':
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    # this stores consistent initial orbit elements
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    # with circular or equatorial orbit, some angles are
    # arbitrary
    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    # useSphericalHarmonics:
    simulationTime = macros.sec2nano(3. * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    # useSphericalHarmonics:
    numDataPoints = 400
    samplingTime = simulationTime / (numDataPoints - 1)
    sim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    # create simulation messages
    #
    simIncludeGravity.addDefaultEphemerisMsg(sim.TotalSim, simProcessName)

    #   configure a simulation stop time time and execute the simulation run
    sim.ConfigureStopTime(simulationTime / LENGTH_SHRINKAGE_FACTOR)

    return sim

def myExecutionFunction(sim):
    ''' function that executes a simulation '''
    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    sim.InitializeSimulationAndDiscover()

    sim.ExecuteSimulation()
    pass


# The data to be retained from a simulation is defined in this format
myRetainedData = {
    "messages": {
        "inertial_state_output.r_BN_N": range(3),
        "inertial_state_output.v_BN_N": range(3)
    },
    "variables": {

    }
}


@pytest.mark.slowtest
def test_MonteCarloSimulation():
    # test a montecarlo simulation
    dirName = "tmp_montecarlo_test"
    monteCarlo = Controller()
    monteCarlo.setShouldDisperseSeeds(True)
    monteCarlo.setExecutionFunction(myExecutionFunction)
    monteCarlo.setSimulationFunction(myCreationFunction)
    monteCarlo.setRetentionParameters(myRetainedData)
    monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
    monteCarlo.setThreadCount(PROCESSES)
    monteCarlo.setVerbose(VERBOSE)
    monteCarlo.setArchiveDir(dirName)

    # add some dispersions
    disp1Name = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
    disp2Name = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
    disp3Name = 'TaskList[0].TaskModels[0].hub.mHub'
    disp4Name = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
    monteCarlo.addNewDispersion(dispersions.UniformEulerAngleMRPDispersion(disp1Name))
    monteCarlo.addNewDispersion(dispersions.NormalVectorCartDispersion(disp2Name, 0.0, 0.75 / 3.0 * np.pi / 180))
    monteCarlo.addNewDispersion(dispersions.UniformDispersion(disp3Name, ([1300.0 - 812.3, 1500.0 - 812.3])))
    monteCarlo.addNewDispersion(dispersions.NormalVectorCartDispersion(disp4Name, [0.0, 0.0, 1.0], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))

    failures = monteCarlo.executeSimulations()

    assert len(failures) == 0, "No runs should fail"

    monteCarloLoaded = Controller.load(dirName)

    retainedData = monteCarloLoaded.getRetainedData(19)
    assert retainedData is not None, "Retained data should be available after execution"
    assert "messages" in retainedData, "Retained data should retain messages"
    assert "inertial_state_output.r_BN_N" in retainedData["messages"], "Retained messages should exist"
    assert "inertial_state_output.v_BN_N" in retainedData["messages"], "Retained messages should exist"
    oldOutput = retainedData["messages"]["inertial_state_output.r_BN_N"]

    # rerun the case and it should be the same, because we dispersed random seeds
    failed = monteCarloLoaded.reRunCases([NUMBER_OF_RUNS-1])
    assert len(failed) == 0, "Should rerun case successfully"
    retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
    newOutput = retainedData["messages"]["inertial_state_output.r_BN_N"]

    for (k1,v1) in enumerate(oldOutput):
        for (k2,v2) in enumerate(v1):
            assert fabs(oldOutput[k1][k2] - newOutput[k1][k2]) < .001, \
            "Outputs shouldn't change on runs if random seeds are same"

    params = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-1)

    assert "TaskList[0].TaskModels[0].RNGSeed" in params, "random number seed should be applied"
    assert disp1Name in params, "dispersion should be applied"
    assert disp2Name in params, "dispersion should be applied"
    assert disp3Name in params, "dispersion should be applied"
    assert disp4Name in params, "dispersion should be applied"

    # assert two different runs had different parameters.

    params1 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-1)
    params2 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-2)
    assert params1[disp2Name] != params2[disp2Name], "dispersion should be different in each run"
    assert params1[disp3Name] != params2[disp3Name], "dispersion should be different in each run"
    assert params1[disp4Name] != params2[disp4Name], "dispersion should be different in each run"

    # TODO why is this always the same
    # assert params1[disp1Name] != params2[disp1Name], "dispersion should be different in each run"
    # print "Note how", disp1Name, "is same for different runs...", params1[disp1Name], params2[disp1Name]

    shutil.rmtree(dirName)
    assert not os.path.exists(dirName), "No leftover data should exist after the test"

if __name__ == "__main__":
    test_MonteCarloSimulation()
