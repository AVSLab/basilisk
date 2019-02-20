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


# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import atmosphere
from Basilisk.utilities import unitTestSupport


def test_unitAtmosphere():
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    newAtmo = atmosphere.Atmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "ExpAtmo"

    showVal = False
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

    testSum = sum(testResults)
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
        if atmoModel.epochDate != dateVec:
            testFailCount += 1
            testMessages.append("FAILED: ExponentialAtmosphere could not set date to "+str(date)+".")

    return testFailCount, testMessages

def AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "spacecraftBody"
    # add spacecraftPlus object to the simulation process
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsgName)
    atmoModel.addSpacecraftToModel(scObject2.scStateOutMsgName)

    if len(atmoModel.scStateInMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough input message names.")

    if len(atmoModel.atmoDensOutMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough output message names.")
    return testFailCount, testMessages

##  Test specific atmospheric model performance

def TestExponentialAtmosphere():
    testFailCount = 0
    testMessages = []

    def expAtmoComp(alt, baseDens, scaleHeight):
        dens = baseDens * math.exp(-alt/scaleHeight)
    return dens

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
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "ExpAtmo"

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, newAtmo)


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    #scObject.addDynamicEffector(dragEffector)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

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
    newAtmo.atmosphereProps.baseDensity = refBaseDens
    newAtmo.atmosphereProps.scaleHeight = refScaleHeight
    newAtmo.atmosphereProps.planetRadius = r_eq

    orbAltMin = 300.0*1000.0
    orbAltMax = orbAltMin

    rMin = r_eq + orbAltMin
    rMax = r_eq + orbAltMax
    oe.a = (rMin+rMax)/2.0
    oe.e = 1.0 - rMin/oe.a
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 0.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are
    # arbitrary

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
    scSim.TotalSim.logThisMessage('atmo_dens0_data', samplingTime)
    scSim.AddVariableForLogging('ExpAtmo.relativePos', samplingTime, StartIndex=0, StopIndex=2)

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

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
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))
    densData = scSim.pullMessageLogData('atmo_dens0_data.neutralDensity')
    relPosData = scSim.GetLogVariableData('ExpAtmo.relativePos')
    np.set_printoptions(precision=16)

    #   Compare to expected values

    refAtmoDensData = []

    accuracy = 1e-13

    for relPos in relPosData:
        dist = np.linalg.norm(relPos[1:])
        alt = dist - r_eq
        refAtmoDensData.append(expAtmoComp(alt,refBaseDens,refScaleHeight))
        # check a vector values
    for ind in range(0,len(densData)):
        if not unitTestSupport.isDoubleEqual(densData[ind,:], refAtmoDensData[ind],accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED:  ExpAtmo failed density unit test at t=" + str(densData[ind, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(densData[ind,1]-refAtmoDensData[ind]))

