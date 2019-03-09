''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging
import pytest
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import atmosphere
from Basilisk.utilities import simIncludeGravBody


def test_unitAtmosphere():
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    newAtmo = atmosphere.Atmosphere()
    newAtmo.ModelTag = "ExpAtmo"

    testResults = []
    testMessage = []

    envTypeRes, envTypeMsg = setEnvType(newAtmo)
    testMessage.append(envTypeMsg)
    testResults.append(envTypeRes)

    epochRes, epochMsg = setEpoch(newAtmo)
    testMessage.append(epochMsg)
    testResults.append(epochRes)

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
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)


    assert testSum < 1, testMessage

##  Test basic environment parameters (set environment, date, add spacecraft)

def setEnvType(atmoModel):
    testFailCount = 0
    testMessages = []
    nameVec = ["exponential"]
    for name in nameVec:
        atmoModel.setEnvType(name)
        if atmoModel.envType != name:
            testFailCount += 1
            testMessages.append(
                "FAILED: ExponentialAtmosphere could not set type to "+name+".")
    return testFailCount, testMessages

def setEpoch(atmoModel):
    testFailCount = 0
    testMessages = []
    dateVec = [20000, 2000.]
    for date in dateVec:
        atmoModel.setEpoch(date)
        if atmoModel.epochDate != date:
            testFailCount += 1
            testMessages.append("FAILED: ExponentialAtmosphere could not set date to "+str(date)+".")

    return testFailCount, testMessages

def AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.scStateOutMsgName = "inertial_state_output_0"

    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "spacecraftBody"
    scObject.scStateOutMsgName = "inertial_state_output_1"
    # add spacecraftPlus object to the simulation process
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsgName)
    atmoModel.addSpacecraftToModel(scObject2.scStateOutMsgName)

    if len(atmoModel.scStateInMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough input message names.")

    if len(atmoModel.envOutMsgNames) != 2:
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
    scSim.TotalSim.terminateSimulation()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = atmosphere.Atmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.setEnvType("exponential")


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"


    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsgName)

    planet = gravFactory.createEarth()

    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    r_eq = 6371*1000.0
    refBaseDens = 1.217
    refScaleHeight = 8500.0

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.exponentialParams.baseDensity = refBaseDens
    newAtmo.exponentialParams.scaleHeight = refScaleHeight
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
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N


    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    simulationTime = macros.sec2nano(0.5*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage("exponential_0_data", samplingTime)


    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    densData = scSim.pullMessageLogData("exponential_0_data.neutralDensity")
    np.set_printoptions(precision=16)




    #   Compare to expected values
    accuracy = 1e-5

    for ind in range(0,len(densData)):
        dist = np.linalg.norm(posData[ind, 1:])
        alt = dist - newAtmo.planetRadius

        trueDensity = expAtmoComp(alt, refBaseDens, refScaleHeight)
        # check a vector values
        if not unitTestSupport.isDoubleEqualRelative(densData[ind,1], trueDensity,accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED:  ExpAtmo failed density unit test at t=" + str(densData[ind, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(densData[ind,1]-trueDensity))

    return testFailCount, testMessages

if __name__=='__main__':
    TestExponentialAtmosphere()