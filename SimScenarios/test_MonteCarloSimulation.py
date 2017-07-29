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

import MonteCarlo
import SimulationBaseClass
import numpy as np
import shutil


# import simulation related support
import spacecraftPlus
import orbitalMotion
import gravityEffector
import simIncludeGravity
import macros

import pytest
import unitTestSupport          # general support file with common unit test functions

NUMBER_OF_RUNS = 20
# scale this up to make the simulations faster by shortening the length of
# the sim.
LENGTH_SHRINKAGE_FACTOR = 50
VERBOSE = True
PROCESSES=1


def myCreationFunction():
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
    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    sim.InitializeSimulationAndDiscover()

    sim.ExecuteSimulation()
    pass


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

    #
    # create a montecarlo class to test
    #
    dirName = "tmp_montecarlo_test"
    monteCarlo = MonteCarlo.MonteCarloController()
    monteCarlo.setDisperseSeeds(True)
    monteCarlo.setExecutionModule(myExecutionFunction)
    monteCarlo.setSimulationObject(myCreationFunction)
    monteCarlo.setRetentionParameters(myRetainedData)
    monteCarlo.setDisperseSeeds(True)
    monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
    monteCarlo.setThreadCount(PROCESSES)
    monteCarlo.setVerbose(VERBOSE)
    monteCarlo.archiveICs(dirName)
    # monteCarlo.addNewDispersion()

    failures = monteCarlo.executeSimulations()

    assert len(failures) == 0, "No runs should fail"

    monteCarloLoaded = MonteCarlo.MonteCarloController.load(dirName)

    retainedData = monteCarloLoaded.getRetainedData(19)
    assert retainedData is not None, "Retained data should be available after execution"
    assert "messages" in retainedData, "Retained data should retain messages"
    assert "inertial_state_output.r_BN_N" in retainedData["messages"], "Retained messages should exist"
    assert "inertial_state_output.v_BN_N" in retainedData["messages"], "Retained messages should exist"

    shutil.rmtree(dirName)
    assert not os.path.exists(dirName), "No leftover data should exist after the test"


if __name__ == "__main__":
    test_MonteCarloSimulation()
