''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
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
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import sim_model

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simple_nav
from Basilisk.simulation import atmosphere
from Basilisk.simulation import spice_interface

# import FSW Algorithm related support
from Basilisk.utilities import simulationArchTypes



def test_unitAtmosphere():
    '''This function is called by the py.test environment.'''

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = atmosphere.Atmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "NrlmsiseAtmo"

    showVal = False
    testResults = []
    testMessage = []

    messagingRes, messagingMsg = MessagingUpdate(newAtmo)
    testMessage.append(messagingMsg)
    testResults.append(messagingRes)

    planetRes, planetMsg = AddSpacecraftToModel(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)



    testSum = sum(testResults)
    assert testSum < 1, testMessage

def AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "spacecraftBody"
    scObject2.hub.useTranslation = True
    scObject2.hub.useRotation = False

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

def AtmosphereUpdate(atmoModel):
    '''
    Verifies that the loaded model outputs are correctly transferred to the input struct and that the model output matches
    the test values included with NRLMSISE-00.h
    :param atmoModel:
    :return: testFailCount, testMessages
    '''

    testFailCount = 0
    testMessages = []

    testValidation = np.loadtxt(open("nrlmsiseTestData.csv","rb"), delimiter=",")


    msisModelVec = []
    aph = []
    validationMat = np.zeros([17,])

    for ind in range(0,7):
        aph.append(100.0)

    r_earth = 6378.137e3
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsgName)

    scObject.hub.r_CN_NInit = [[400000+r_earth], [0], [0]]
    scObject.hub.v_CN_NInit = [[7000.0], [0], [0]]

    for ind in range(0,17):
        tmpAtmosphere = atmosphere.Atmosphere()
        tmpAtmosphere.setEnvType('nrlmsise00')
        msisModelVec.append(tmpAtmosphere)

        msisModelVec[-1].doy = 172
        msisModelVec[-1].year = 0
        msisModelVec[-1].sec = 29000
        msisModelVec[-1].alt = 400
        msisModelVec[-1].g_lat = 60
        msisModelVec[-1].g_long = -70
        msisModelVec[-1].lst = 16
        msisModelVec[-1].f107A = 150
        msisModelVec[-1].f107 = 150
        msisModelVec[-1].ap = 4

    msisModelVec[1].doy = 81
    msisModelVec[2].sec = 75000
    msisModelVec[2].alt = 1000
    msisModelVec[3].alt = 100
    msisModelVec[10].alt = 0
    msisModelVec[11].alt = 10
    msisModelVec[12].alt = 30
    msisModelVec[13].alt = 50
    msisModelVec[14].alt = 70
    msisModelVec[16].alt = 100
    msisModelVec[4].g_lat = 0
    msisModelVec[5].g_long = 0
    msisModelVec[6].lst = 4
    msisModelVec[7].f107A = 70
    msisModelVec[8].f107 = 180
    msisModelVec[9].ap = 40
    msisModelVec[15].ap_a = aph
    msisModelVec[16].ap_a = aph

    for ind in range(0,15):
        msisModelVec[ind].updateLocalAtmosphere(0)

    flags.switches[9] = -1;
    for i in range(15,17):
        msisModelVec[ind].msisFlags.switches[9] = -1
        msisModelVec[ind].updateLocalAtmosphere(0)

    for ind in range(0,17):
        if validationMat[ind] != testValidation[ind]:
            testFailCount += 1
            testMessages.append("FAILED: NrlmsiseAtmosphere does not have enough input message names.")

    return testFailCount, testMessages

def MessagingUpdate(atmoModel):
    # each test method requires a single assert method to be called

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    unitTestSim.unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitTestSim.unitProcessName = "testProcess"  # arbitrary name (don't change)

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    unitTestSim.testProc = unitTestSim.CreateNewProcess(unitTestSim.unitProcessName,1)
    unitTestSim.testProc.addTask(unitTestSim.CreateNewTask(unitTestSim.unitTaskName, testProcessRate))


    gravFactory = simIncludeGravBody.gravBodyFactory()

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsgName)


    # Add test module to runtime call list


    unitTestSim.earthGravBody = gravFactory.createEarth()
    unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15  # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    earthEphemData = spice_interface.SpicePlanetStateSimMsg()
    earthEphemData.J2000Current = 0.0
    earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    earthEphemData.PlanetName = "earth"

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    unitTestSim.TotalSim.logThisMessage(atmoModel.atmoDensOutMsgNames[-1], testProcessRate)

    msgSize = earthEphemData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitTestSim.unitProcessName,
                                          unitTestSim.earthGravBody.bodyInMsgName, msgSize, 2)
    unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyInMsgName, msgSize, 0, earthEphemData)


    swMsgData = nrlmsiseAtmosphere.swDataSimMsg()
    msgSize2 = swMsgData.getStructSize()
    swMsgData.dataValue = 100.0 # arbitrary and unrealistic
    swMsgData.averagingType = 3.0 #   arbitrary and unrealistic
    swMsgData.centeringOffset = 12.0 #    arbitrary and unrealistic

    inMessageNameList = ["ap_24_0","ap_3_0","ap_3_-3","ap_3_-6","ap_3_-9","ap_3_-12","ap_3_-15","ap_3_-18","ap_3_-21",
                         "ap_3_-24","ap_3_-27","ap_3_-30","ap_3_-33","ap_3_-36","ap_3_-39","ap_3_-42","ap_3_-45",
                         "ap_3_-48","ap_3_-51","ap_3_-54","ap_3_-57","f107_1944_0","f107_24_-24"]

    for name in inMessageNameList:
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitTestSim.unitProcessName,
                                   name,
                                   swMsgData)

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.AddModelToTask(unitTestSim.unitTaskName, atmoModel)
    unitTestSim.AddModelToTask(unitTestSim.unitTaskName, scObject)


    unitTestSim.InitializeSimulationAndDiscover()
    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    testFailCount = 0
    testMessages = []
    postInputStruct = newAtmo.NrlmsiseAtmosphere_msisInput_get

    if newAtmo.msisInput.f107A != swMsgData.dataValue:
        testFailCount += 1
        testMessages.append("FAILED: NrlmsiseAtmosphere does not have enough input message names.")


    return testFailCount, testMessages

if __name__ == "__main__":
    newAtmo = nrlmsiseAtmosphere.NrlmsiseAtmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "NrlmsiseAtmo"

    AtmosphereUpdate(newAtmo)
    MessagingUpdate(newAtmo)


