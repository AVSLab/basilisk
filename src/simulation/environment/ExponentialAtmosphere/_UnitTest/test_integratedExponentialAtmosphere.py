
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.



import inspect
import math
import os

import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.utilities import simIncludeGravBody


def test_unitExponentialAtmosphere():
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"

    testResults = []
    testMessage = []

    addScRes, addScMsg = AddSpacecraftToModel(newAtmo)
    testMessage.append(addScMsg)
    testResults.append(addScRes)

    exponentialRes, exponentialMsg = TestExponentialAtmosphere()
    testMessage.append(exponentialMsg)
    testResults.append(exponentialRes)

    #   print out success message if no error were found
    snippetName = "passFail"
    testSum = sum(testResults)
    if testSum == 0:
        colorText = 'ForestGreen'
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)

    if testSum == 0:
        print("Passed")

    assert testSum < 1, testMessage

def AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody"
    # add spacecraft object to the simulation process
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsg)
    atmoModel.addSpacecraftToModel(scObject2.scStateOutMsg)

    if len(atmoModel.scStateInMsgs) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough input message names.")

    if len(atmoModel.envOutMsgs) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough output message names.")
    return testFailCount, testMessages

##  Test specific atmospheric model performance

def TestExponentialAtmosphere():
    testFailCount = 0
    testMessages = []

    def expAtmoComp(alt, baseDens, scaleHeight):
        density = baseDens * math.exp(-alt/scaleHeight)
        return density


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    planet = gravFactory.createEarth()

    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    r_eq = 6371*1000.0
    refBaseDens = 1.217
    refScaleHeight = 8500.0

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.baseDensity = refBaseDens
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.planetRadius = r_eq


    oe.a = r_eq + 300.*1000
    oe.e = 0.0
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 0.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are
    # arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N


    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    simulationTime = macros.sec2nano(0.5*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    denLog = newAtmo.envOutMsgs[0].recorder(samplingTime)

    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, denLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    densData = denLog.neutralDensity
    np.set_printoptions(precision=16)

    #   Compare to expected values
    accuracy = 1e-5
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    if len(densData) > 0:
        for ind in range(0,len(densData)):
            dist = np.linalg.norm(posData[ind])
            alt = dist - newAtmo.planetRadius

            trueDensity = expAtmoComp(alt, refBaseDens, refScaleHeight)
            # check a vector values
            if not unitTestSupport.isDoubleEqualRelative(densData[ind], trueDensity, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED:  ExpAtmo failed density unit test at t=" + str(densData[ind, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(densData[ind,1]-trueDensity))
    else:
        testFailCount += 1
        testMessages.append("FAILED:  ExpAtmo failed to pull any logged data")

    return testFailCount, testMessages

if __name__=='__main__':
    # TestExponentialAtmosphere()
    test_unitExponentialAtmosphere()
