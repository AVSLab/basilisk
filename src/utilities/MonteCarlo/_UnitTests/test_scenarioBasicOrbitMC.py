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
import inspect  # Don't worry about this, standard stuff plus file discovery
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# bskName = 'Basilisk'
# splitPath = path.split(bskName)
# bskPath = splitPath[0] + '/' + bskName + '/'
# sys.path.append(bskPath + 'modules')
# sys.path.append(bskPath + 'PythonModules')

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import UniformEulerAngleMRPDispersion, UniformDispersion, NormalVectorCartDispersion, OrbitalElementDispersion
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
import shutil
import matplotlib.pyplot as plt
import numpy as np
import pytest

NUMBER_OF_RUNS = 4
VERBOSE = True
PROCESSES = 2

retainedMessageName = "spacecraftStateMsg"
retainedRate = macros.sec2nano(10)
var1 = "v_BN_N"
var2 = "r_BN_N"

def myCreationFunction():
    """ function that returns a simulation """
    #  Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create the simulation process
    dynProcess = sim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(sim.CreateNewTask(simTaskName, simulationTimeStep))

    # Setup the simulation modules
    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # Add spacecraft object to the simulation process
    sim.AddModelToTask(simTaskName, scObject)

    # Setup Earth gravity body and attach gravity model to spacecraft
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    planet.useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt'
                                        , 2
                                        )
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([planet])

    # Setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rGEO = 42000. * 1000  # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(planet.mu, oe)  # this stores consistent initial orbit elements
    oe = orbitalMotion.rv2elem(planet.mu, rN, vN)  # with circular or equatorial orbit, some angles are arbitrary

    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    mean_motion = np.sqrt(planet.mu / oe.a / oe.a / oe.a)
    period = 2. * np.pi / mean_motion
    simulationTime = macros.sec2nano(period / 4)

    sim.msgRecList = {}
    sim.msgRecList[retainedMessageName] = scObject.scStateOutMsg.recorder(retainedRate)
    sim.AddModelToTask(simTaskName, sim.msgRecList[retainedMessageName])

    #   configure a simulation stop time and execute the simulation run
    sim.ConfigureStopTime(simulationTime)

    return sim


def myExecutionFunction(sim):
    """ function that executes a simulation """
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

colorList = ['b', 'r', 'g', 'k']

def myDataCallback(monteCarloData, retentionPolicy):
    data = np.array(monteCarloData["messages"][retainedMessageName + ".r_BN_N"])
    plt.plot(data[:, 1], data[:, 2], colorList[monteCarloData["index"]], label="run " + str(monteCarloData["index"]))
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.legend()



@pytest.mark.slowtest
def test_MonteCarloSimulation(show_plots):
    # Test a montecarlo simulation
    dirName = os.path.abspath(os.path.dirname(__file__)) + "/tmp_montecarlo_test"
    monteCarlo = Controller()
    monteCarlo.setShouldDisperseSeeds(True)
    monteCarlo.setExecutionFunction(myExecutionFunction)
    monteCarlo.setSimulationFunction(myCreationFunction)
    monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
    monteCarlo.setThreadCount(PROCESSES)
    monteCarlo.setVerbose(True)
    monteCarlo.setArchiveDir(dirName)

    # Add some dispersions
    disp1Name = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
    disp2Name = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
    disp3Name = 'TaskList[0].TaskModels[0].hub.mHub'
    disp4Name = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
    disp5Name = 'TaskList[0].TaskModels[0].hub.r_CN_NInit'
    disp6Name = 'TaskList[0].TaskModels[0].hub.v_CN_NInit'
    dispDict = {}
    dispDict["mu"] = 0.3986004415E+15
    dispDict["a"] = ["normal", 42000 * 1E3, 2000 * 1E3]
    dispDict["e"] = ["uniform", 0, 0.5]
    dispDict["i"] = ["uniform", -80, 80]
    dispDict["Omega"] = None
    dispDict["omega"] = ["uniform", 80, 90]
    dispDict["f"] = ["uniform", 0, 359]
    monteCarlo.addDispersion(OrbitalElementDispersion(disp5Name, disp6Name, dispDict))
    monteCarlo.addDispersion(UniformEulerAngleMRPDispersion(disp1Name))
    monteCarlo.addDispersion(NormalVectorCartDispersion(disp2Name, 0.0, 0.75 / 3.0 * np.pi / 180))
    monteCarlo.addDispersion(UniformDispersion(disp3Name, ([1300.0 - 812.3, 1500.0 - 812.3])))
    monteCarlo.addDispersion(
        NormalVectorCartDispersion(disp4Name, [0.0, 0.0, 1.0], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))

    # Add retention policy
    retentionPolicy = RetentionPolicy()
    retentionPolicy.addMessageLog(retainedMessageName, [var1, var2])
    retentionPolicy.setDataCallback(myDataCallback)
    monteCarlo.addRetentionPolicy(retentionPolicy)

    failures = monteCarlo.executeSimulations()

    assert len(failures) == 0, "No runs should fail"

    # Test loading data from runs from disk
    monteCarloLoaded = Controller.load(dirName)

    retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
    assert retainedData is not None, "Retained data should be available after execution"
    assert "messages" in retainedData, "Retained data should retain messages"
    assert retainedMessageName + ".r_BN_N" in retainedData["messages"], "Retained messages should exist"
    assert retainedMessageName + ".v_BN_N" in retainedData["messages"], "Retained messages should exist"

    # rerun the case and it should be the same, because we dispersed random seeds
    oldOutput = retainedData["messages"][retainedMessageName + ".r_BN_N"]

    failed = monteCarloLoaded.reRunCases([NUMBER_OF_RUNS-1])
    assert len(failed) == 0, "Should rerun case successfully"

    retainedData = monteCarloLoaded.getRetainedData(NUMBER_OF_RUNS-1)
    newOutput = retainedData["messages"][retainedMessageName + ".r_BN_N"]
    for k1, v1 in enumerate(oldOutput):
        for k2, v2 in enumerate(v1):
            assert np.fabs(oldOutput[k1][k2] - newOutput[k1][k2]) < .001, \
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
    if show_plots:
        plt.show()

    shutil.rmtree(dirName)
    assert not os.path.exists(dirName), "No leftover data should exist after the test"


if __name__ == "__main__":
    test_MonteCarloSimulation(show_plots=True)
