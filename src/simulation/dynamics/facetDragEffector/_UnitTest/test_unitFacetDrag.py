''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the validity of a simple exponential atmosphere model.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

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
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import facetDragDynamicEffector
from Basilisk.utilities import unitTestSupport
#print dir(exponentialAtmosphere)


def test_unitAtmosphere():
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    #   Initialize new atmosphere and drag model, add them to task
    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    showVal = False
    testResults = []
    testMessage = []

    planetRes, planetMsg = SetDensityMsgTest(newDrag)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    


    testSum = sum(testResults)
    assert testSum < 1, testMessage

def SetDensityMsgTest(dragEffector):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    scObject.addDynamicEffector(dragEffector)

    # add spacecraftPlus object to the simulation process
    dragEffector.setDensityMessage("testDensMsgName")

    if dragEffector.atmoDensInMsgName != "testDensMsgName":
        testFailCount += 1
        testMessages.append(
            "FAILED: DragEffector does not correctly set message names.")
    return testFailCount, testMessages

def TestDragCalculation():

    #   Init test support variables
    showVal = False
    testResults = []
    testMessage = []

    ##   Simulation initialization
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))
    spacecraft = spacecraftPlus.SpacecraftPlus()


    ##   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()

    try:
        scAreas = [1.0, 1.0]
        scCoeff = np.array([2.0, 2.0])
        B_normals = [np.array([1, 0, 0]), np.array([0, 1, 0])]
        B_locations = [np.array([0.1,0,0]), np.array([0,0.1,0])]

        for ind in range(0,len(scAreas)):
            newDrag.addFacet(scAreas[ind], scCoeff[ind], base_normals[ind], facet_locations[ind])

        spacecraft.mass = 6.0
    except:
        testFailCount += 1
        testMessage.append("ERROR: FacetDrag unit test failed while setting facet parameters.")
        return testFailCount, testMessage

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

    unitTestSupport.createMessage


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"


    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsgName)
    newDrag.setDensityMsg(newAtmo.ModelTag+"_0_data")

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
    scSim.TotalSim.logThisMessage(newAtmo.ModelTag+"_0_data", samplingTime)


    unitTestSim.AddVariableForLogging(newDrag.ModelTag + ".forceExternal_B",
                                      simulationTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(NewDrag.ModelTag + ".forceExternal_N",
                                      simulationTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(newDrag.ModelTag + ".torqueExternalPntB_B",
                                      simulationTime, 0, 2, 'double')


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


    dragDataForce_B = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_B")
    dragDataForce_N = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_N")
    dragTorqueData = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".torqueExternalPntB_B")
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    densData = scSim.pullMessageLogData(newAtmo.ModelTag+"_0_data.neutralDensity")
    np.set_printoptions(precision=16)




    #   Compare to expected values
    accuracy = 1e-5
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    if len(densData) > 0:
        for ind in range(0,len(densData)):
            dist = np.linalg.norm(posData[ind, 1:])
            alt = dist - newAtmo.planetRadius

            trueDensity = expAtmoComp(alt, refBaseDens, refScaleHeight)
            # check a vector values
            if not unitTestSupport.isDoubleEqualRelative(densData[ind,1], trueDensity,accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED:  ExpAtmo failed density unit test at t=" + str(densData[ind, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(densData[ind,1]-trueDensity))
    else:
        testFailCount += 1
        testMessages.append("FAILED:  ExpAtmo failed to pull any logged data")

    return testFailCount, testMessages




