import sys
import os
import inspect  # Don't worry about this, standard stuff plus file discovery

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import UniformEulerAngleMRPDispersion, UniformDispersion, NormalVectorCartDispersion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.modules import orbitalMotion
from Basilisk.modules import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.modules import SimulationBaseClass
import numpy as np

import pytest
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from math import fabs
import shutil

import matplotlib.pyplot as plt

NUMBER_OF_RUNS = 20
LENGTH_SHRINKAGE_FACTOR = 50 # make the simulations faster by dividing the length of the sim by this.
VERBOSE = True
PROCESSES=8


def myCreationFunction():
    ''' function that returns a simulation '''
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


    # Earth
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu

    # attach gravity model to spaceCraftPlus
    #scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #   setup orbit and simulation time
    # setup the orbit using classical orbit elements
    # orbitCase is 'GEO':
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe) # this stores consistent initial orbit elements
    oe = orbitalMotion.rv2elem(mu, rN, vN) # with circular or equatorial orbit, some angles are arbitrary
    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    # useSphericalHarmonics:
    simulationTime = macros.sec2nano(3. * P)

    #   Setup data logging before the simulation is initialized
    numDataPoints = 400
    samplingTime = simulationTime / (numDataPoints - 1)

    sim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #   configure a simulation stop time time and execute the simulation run
    sim.ConfigureStopTime(simulationTime / LENGTH_SHRINKAGE_FACTOR)

    return sim

def myExecutionFunction(sim):
    ''' function that executes a simulation '''
    sim.InitializeSimulationAndDiscover()
    sim.ExecuteSimulation()

retainedMessageName = "inertial_state_output"
retainedRate = 100
var1 = "v_BN_N"
var2 = "r_BN_N"
dataType1 = range(3)
dataType2 = range(3)

def myDataCallback(monteCarloData, retentionPolicy):
    print monteCarloData
    data = np.array(monteCarloData["messages"]["inertial_state_output.v_BN_N"])
    plt.plot(data[:,1], data[:,2])

@pytest.mark.slowtest
def test_MonteCarloSimulation():
    # test a montecarlo simulation
    dirName = "tmp_montecarlo_test"
    monteCarlo = Controller()
    monteCarlo.setShouldDisperseSeeds(True)
    monteCarlo.setExecutionFunction(myExecutionFunction)
    monteCarlo.setSimulationFunction(myCreationFunction)
    monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
    monteCarlo.setThreadCount(PROCESSES)
    monteCarlo.setVerbose(False)
    monteCarlo.setArchiveDir(dirName)

    # add some dispersions
    disp1Name = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
    disp2Name = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
    disp3Name = 'TaskList[0].TaskModels[0].hub.mHub'
    disp4Name = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
    monteCarlo.addDispersion(UniformEulerAngleMRPDispersion(disp1Name))
    monteCarlo.addDispersion(NormalVectorCartDispersion(disp2Name, 0.0, 0.75 / 3.0 * np.pi / 180))
    monteCarlo.addDispersion(UniformDispersion(disp3Name, ([1300.0 - 812.3, 1500.0 - 812.3])))
    monteCarlo.addDispersion(NormalVectorCartDispersion(disp4Name, [0.0, 0.0, 1.0], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))

    # add retention policy
    retentionPolicy = RetentionPolicy()
    retentionPolicy.addMessageLog(retainedMessageName, [(var1, dataType1), (var2, dataType2)], retainedRate)
    retentionPolicy.setDataCallback(myDataCallback)

    monteCarlo.addRetentionPolicy(retentionPolicy)

    failures = monteCarlo.executeSimulations()

    assert len(failures) == 0, "No runs should fail"

    # Test loading data from runs from disk
    monteCarloLoaded = Controller.load(dirName)

    retainedData = monteCarloLoaded.getRetainedData(19)
    assert retainedData is not None, "Retained data should be available after execution"
    assert "messages" in retainedData, "Retained data should retain messages"
    assert "inertial_state_output.r_BN_N" in retainedData["messages"], "Retained messages should exist"
    assert "inertial_state_output.v_BN_N" in retainedData["messages"], "Retained messages should exist"

    # rerun the case and it should be the same, because we dispersed random seeds
    oldOutput = retainedData["messages"]["inertial_state_output.r_BN_N"]

    failed = monteCarloLoaded.reRunCases([NUMBER_OF_RUNS-1])
    assert len(failed) == 0, "Should rerun case successfully"

    retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
    newOutput = retainedData["messages"]["inertial_state_output.r_BN_N"]
    for k1, v1 in enumerate(oldOutput):
        for k2, v2 in enumerate(v1):
            assert fabs(oldOutput[k1][k2] - newOutput[k1][k2]) < .001, \
            "Outputs shouldn't change on runs if random seeds are same"

    # test the initial parameters were saved from runs, and they differ between runs
    params1 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-1)
    params2 = monteCarloLoaded.getParameters(NUMBER_OF_RUNS-2)
    assert "TaskList[0].TaskModels[0].RNGSeed" in params1, "random number seed should be applied"
    for dispName in [disp1Name, disp2Name, disp3Name, disp4Name]:
        assert dispName in params1, "dispersion should be applied"
        # assert two different runs had different parameters.
        assert params1[dispName] != params2[dispName], "dispersion should be different in each run"

    monteCarloLoaded.executeCallbacks()
    plt.show()

    shutil.rmtree(dirName)
    assert not os.path.exists(dirName), "No leftover data should exist after the test"

if __name__ == "__main__":
    test_MonteCarloSimulation()
