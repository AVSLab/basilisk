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
#
#
# Purpose:  Test the facetDrag module.
# Author:   Andrew Harris
# Creation Date:  May 16 2019
#


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
bskName = 'Basilisk'
splitPath = path.split(bskName)


# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import facetDragDynamicEffector
from Basilisk.simulation import simple_nav
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeGravBody
#print dir(exponentialAtmosphere)


def test_unitFacetDrag():
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    #   Initialize new atmosphere and drag model, add them to task
    #newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    showVal = False
    testResults = []
    testMessage = []

    planetRes, planetMsg = SetDensityMsgTest(newDrag)
    testMessage.append(planetMsg)
    testResults.append(planetRes)

    dragRes, dragMsg = TestDragCalculation()
    testMessage.append(dragMsg)
    testResults.append(dragRes)

    shadowRes, shadowMsg = TestShadowCalculation()
    testMessage.append(shadowMsg)
    testResults.append(shadowRes)

    testSum = sum(testResults)

    snippetName = "unitTestPassFail"

    if testSum == 0:
        colorText = 'ForestGreen'
        print "PASSED"
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed"
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)

    assert testSum < 1, testMessage

def SetDensityMsgTest(dragEffector):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

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
    testFailCount = 0
    testResults = []
    testMessages = []

    ##   Simulation initialization
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    simpleNavObj = simple_nav.SimpleNav()
    simpleNavObj.inputStateName = scObject.scStateOutMsgName
    simpleNavObj.outputAttName = 'nav_att_out'

    ##   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsgName)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.setDensityMessage(newAtmo.envOutMsgNames[-1])
    newDrag.navAttInMsgName = 'nav_att_out'
    newDrag.ModelTag = "FacetDrag"

    scObject.addDynamicEffector(newDrag)

    try:
        scAreas = [1.0, 1.0]
        scCoeff = np.array([2.0, 2.0])
        B_normals = [np.array([1, 0, 0]), np.array([0, 1, 0])]
        B_locations = [np.array([0.1,0,0]), np.array([0,0.1,0])]

        for ind in range(0,len(scAreas)):
            newDrag.addFacet(scAreas[ind], scCoeff[ind], B_normals[ind], B_locations[ind])
    except:
        testFailCount += 1
        testMessage.append("ERROR: FacetDrag unit test failed while setting facet parameters.")
        return testFailCount, testMessage

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
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

    rN = np.array([r_eq+200.0e3,0,0])
    vN = np.array([0,7.788e3,0])
    sig_BN = np.array([0,0,0])
    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N
    scObject.hub.sigma_BNInit =  unitTestSupport.np2EigenVectorXd(sig_BN)

    simulationTime = macros.sec2nano(10.)
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10

    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, newDrag)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    samplingTime = simulationTimeStep
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(newAtmo.ModelTag+"_0_data", samplingTime)

    scSim.AddVariableForLogging(newDrag.ModelTag + ".extForce_B",
                                      simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(newDrag.ModelTag + ".extTorque_B",
                                      simulationTimeStep, 0, 2, 'double')

    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   Retrieve logged data
    dragDataForce_B = scSim.GetLogVariableData(newDrag.ModelTag + ".extForce_B")
    dragTorqueData = scSim.GetLogVariableData(newDrag.ModelTag + ".extTorque_B")
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))
    attData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.sigma_BN', range(3))
    densData = scSim.pullMessageLogData(newAtmo.ModelTag+"_0_data.neutralDensity")
    np.set_printoptions(precision=16)

    def checkFacetDragForce(dens, area, coeff, facet_dir, sigma_BN, inertial_vel):
        vel_dir = inertial_vel / np.linalg.norm(inertial_vel)
        dcm = rbk.MRP2C(sigma_BN)
        drag_force = -0.5 * dens * area * (facet_dir.dot(dcm.dot(vel_dir))) *  coeff * np.linalg.norm(inertial_vel)**2.0 * vel_dir
        return drag_force


    #   Compare to expected values
    accuracy = 1e-4
    #unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    if len(densData) > 0:
        for ind in range(1,len(densData)):
            test_val = checkFacetDragForce(densData[ind,1], scAreas[1], scCoeff[1], B_normals[1], attData[ind, 1:], velData[ind, 1:])
            print test_val
                            #   isArrayEqualRelative(result, truth, dim, accuracy):
            if not unitTestSupport.isArrayEqualRelative(dragDataForce_B[ind,:], test_val, 3,accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED:  FacetDragEffector failed force unit test at t=" + str(dragDataForce_B[ind,0]* macros.NANO2SEC) + "sec with a value difference of "+str(dragDataForce_B[ind,1:]-test_val))


    else:
        testFailCount += 1
        testMessages.append("FAILED:  ExpAtmo failed to pull any logged data")

    return testFailCount, testMessages


def TestShadowCalculation():

    #   Init test support variables
    showVal = False
    testFailCount = 0
    testResults = []
    testMessages = []

    ##   Simulation initialization
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    simpleNavObj = simple_nav.SimpleNav()
    simpleNavObj.inputStateName = scObject.scStateOutMsgName
    simpleNavObj.outputAttName = 'nav_att_out'

    ##   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsgName)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.setDensityMessage(newAtmo.envOutMsgNames[-1])
    newDrag.navAttInMsgName = 'nav_att_out'
    newDrag.ModelTag = "FacetDrag"


    scObject.addDynamicEffector(newDrag)

    try:
        scAreas = [1.0, 1.0]
        scCoeff = np.array([2.0, 2.0])
        B_normals = [np.array([0, 0, -1]), np.array([0, -1, 0])]
        B_locations = [np.array([0,0,0.1]), np.array([0,0.1,0])]

        for ind in range(0,len(scAreas)):
            newDrag.addFacet(scAreas[ind], scCoeff[ind], B_normals[ind], B_locations[ind])
    except:
        testFailCount += 1
        testMessage.append("ERROR: FacetDrag unit test failed while setting facet parameters.")
        return testFailCount, testMessage

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
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

    rN = np.array([r_eq+200.0e3,0,0])
    vN = np.array([0,7.788e3,0])
    sig_BN = np.array([0,0,0])

    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N
    scObject.hub.sigma_BNInit =  unitTestSupport.np2EigenVectorXd(sig_BN)

    simulationTime = macros.sec2nano(10.)
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10

    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, newDrag)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    samplingTime = simulationTimeStep
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(newAtmo.ModelTag+"_0_data", samplingTime)

    scSim.AddVariableForLogging(newDrag.ModelTag + ".extForce_B",
                                      simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(newDrag.ModelTag + ".extTorque_B",
                                      simulationTimeStep, 0, 2, 'double')

    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   Retrieve logged data
    #dragDataForce_B = scSim.GetLogVariableData(newDrag.ModelTag + ".extForce_B")
    dragDataForce_B = scSim.GetLogVariableData(newDrag.ModelTag + ".extForce_B")
    dragTorqueData = scSim.GetLogVariableData(newDrag.ModelTag + ".extTorque_B")
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))
    attData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.sigma_BN', range(3))
    densData = scSim.pullMessageLogData(newAtmo.ModelTag+"_0_data.neutralDensity")
    np.set_printoptions(precision=16)

    def checkFacetDragForce(dens, area, coeff, facet_dir, sigma_BN, inertial_vel):
        vel_dir = inertial_vel / np.linalg.norm(inertial_vel)
        dcm = rbk.MRP2C(sigma_BN)
        drag_force = -0.5 * dens * area * (facet_dir.dot(dcm.dot(vel_dir))) *  coeff * np.linalg.norm(inertial_vel)**2.0 * vel_dir
        return drag_force


    #   Compare to expected values
    accuracy = 1e-9
    #unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    if len(densData) > 0:
        for ind in range(1,len(densData)):
            if not unitTestSupport.isArrayZero(dragDataForce_B[ind,:], 3,accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED:  FacetDragEffector failed shadow unit test at t=" + str(dragDataForce_B[ind,0]* macros.NANO2SEC) + "sec with a value difference of "+str(dragDataForce_B[ind,1:]))


    else:
        testFailCount += 1
        testMessages.append("FAILED:  ExpAtmo failed to pull any logged data")

    return testFailCount, testMessages

if __name__=="__main__":
    test_unitFacetDrag()

