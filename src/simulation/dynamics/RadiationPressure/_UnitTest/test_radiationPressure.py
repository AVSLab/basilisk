
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



#
# RadiationPressure Unit Test
#
# Purpose:  Test the proper function of the Radiation Pressure Dynamics module.
#           This is done by comparing expected torques and forces to
#           what is simulated
# Author:   Patrick Kenneally
# Creation Date:  Feb. 9, 2017
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')



#Import all of the modules that we are going to call in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import radiationPressure
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion as om
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("modelType, eclipseOn", [
      ("cannonball",False)
    , ("lookup", False)
    , ("lookup", True)
    , ("cannonballLookup", False)
])
def test_unitRadiationPressure(show_plots, modelType, eclipseOn):
    """Module Unit Test"""
    [testResults, testMessage] = unitRadiationPressure(show_plots, modelType, eclipseOn)
    assert testResults < 1, testMessage


def unitRadiationPressure(show_plots, modelType, eclipseOn):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(0.1)
    simulationTime = 0.2
    r_N = [-16937711153.5, -138435806556.0, -60051616256.6]  # [m]
    sun_r_N = [507128401.716, 22652490.9092, -14854379.6232]  # [m]
    sigma_BN = [0.1, 0.2, -0.3]

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Ensure simulation is empty
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    unitTestSim.AddModelToTask(testTaskName, scObject)


    srpDynEffector = radiationPressure.RadiationPressure()
    srpDynEffector.ModelTag = "RadiationPressure"
    srpDynEffector2 = radiationPressure.RadiationPressure()
    srpDynEffector2.ModelTag = "RadiationPressure2"
    scObject.addDynamicEffector(srpDynEffector)
    scObject.addDynamicEffector(srpDynEffector2)

    if modelType == "cannonball":
        srpDynEffector.setUseCannonballModel()
        srpDynEffector.area = 4
        srpDynEffector.coefficientReflection = 1.2
    elif modelType == "lookup":
        srpDynEffector.setUseFacetedCPUModel()
        handler = radiationPressure.SRPLookupTableHandler()
        handler.parseAndLoadXML(os.path.dirname(__file__) + "/cube_lookup.xml")
        for i in range(0, len(handler.forceBLookup)):
            srpDynEffector.addForceLookupBEntry(handler.forceBLookup[i, :])
            srpDynEffector.addTorqueLookupBEntry(handler.torqueBLookup[i, :])
            srpDynEffector.addSHatLookupBEntry(handler.sHatBLookup[i, :])
    elif modelType == "cannonballLookup":
        srpDynEffector.setUseFacetedCPUModel()
        handler = radiationPressure.SRPLookupTableHandler()
        handler.parseAndLoadXML(os.path.dirname(__file__) + "/cannonballLookup.xml")
        for i in range(0, len(handler.forceBLookup)):
            srpDynEffector.addForceLookupBEntry(handler.forceBLookup[i, :])
            srpDynEffector.addTorqueLookupBEntry(handler.torqueBLookup[i, :])
            srpDynEffector.addSHatLookupBEntry(handler.sHatBLookup[i, :])
        srpDynEffector2.setUseCannonballModel()
        srpDynEffector2.area = 182018.072141393 #set to give a force of 1N at 1AU to make spherical table generation easy
        srpDynEffector2.coefficientReflection = 1.2
        r_N = [np.sin(np.pi/4.)*np.cos(np.pi/4.)*10.*om.AU*1000., np.sin(np.pi/4.)*np.sin(np.pi/4.)*10.*om.AU*1000., np.cos(np.pi/4.)*10.*om.AU*1000.]  # [m]
        sun_r_N = [0., 0., 0.]  # [m]
        sigma_BN = [0., 0., 0.]

    if eclipseOn:
        sunEclipseMsgData = messaging.EclipseMsgPayload()
        sunEclipseMsgData.shadowFactor = 0.5
        sunEclMsg = messaging.EclipseMsg().write(sunEclipseMsgData)
        srpDynEffector.sunEclipseInMsg.subscribeTo(sunEclMsg)
        srpDynEffector2.sunEclipseInMsg.subscribeTo(sunEclMsg)

    unitTestSim.AddModelToTask(testTaskName, srpDynEffector, 3)
    unitTestSim.AddModelToTask(testTaskName, srpDynEffector2, 3)

    scObject.hub.r_CN_NInit = r_N
    scObject.hub.sigma_BNInit = sigma_BN


    sunSpiceMsg = messaging.SpicePlanetStateMsgPayload()
    sunSpiceMsg.PositionVector = sun_r_N
    sunMsg = messaging.SpicePlanetStateMsg().write(sunSpiceMsg)
    srpDynEffector.sunEphmInMsg.subscribeTo(sunMsg)
    srpDynEffector2.sunEphmInMsg.subscribeTo(sunMsg)

    srpDynEffectorLog = [
        effector.logger(["forceExternal_B", "forceExternal_N", "torqueExternalPntB_B"])
        for effector in [srpDynEffector, srpDynEffector2]
    ]
    for effLog in srpDynEffectorLog:
        unitTestSim.AddModelToTask(testTaskName, effLog)

    unitTestSim.InitializeSimulation()

    # Configure a simulation stop time and execute the simulation run
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    srpDynEffector.computeForceTorque(unitTestSim.TotalSim.CurrentNanos, testTaskRate)
    srpDynEffector2.computeForceTorque(unitTestSim.TotalSim.CurrentNanos, testTaskRate)
    unitTestSim.TotalSim.SingleStepProcesses()

    srpDataForce_B = unitTestSupport.addTimeColumn(srpDynEffectorLog[0].times(), srpDynEffectorLog[0].forceExternal_B)
    srpDataForce_N = unitTestSupport.addTimeColumn(srpDynEffectorLog[0].times(), srpDynEffectorLog[0].forceExternal_N)
    srpTorqueData = unitTestSupport.addTimeColumn(srpDynEffectorLog[0].times(), srpDynEffectorLog[0].torqueExternalPntB_B)

    srp2DataForce_B = unitTestSupport.addTimeColumn(srpDynEffectorLog[1].times(), srpDynEffectorLog[1].forceExternal_B)
    srp2DataForce_N = unitTestSupport.addTimeColumn(srpDynEffectorLog[1].times(), srpDynEffectorLog[1].forceExternal_N)
    srp2TorqueData = unitTestSupport.addTimeColumn(srpDynEffectorLog[1].times(), srpDynEffectorLog[1].torqueExternalPntB_B)

    errTol = 1E-12
    if modelType == "cannonball":
        truthForceExternal_B = [0, 0, 0]
        truthForceExternal_N = [-2.44694525395e-06, -1.94212316004e-05, -8.42121070088e-06]
        truthTorqueExternalPntB_B = [0, 0, 0]
        testFailCount, testMessages = unitTestSupport.compareVector(truthForceExternal_B,
                                                                    srpDataForce_B[1,1:],
                                                                    errTol,
                                                                    "Force_B",
                                                                    testFailCount,
                                                                    testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(truthForceExternal_N,
                                                                srpDataForce_N[1, 1:],
                                                                errTol,
                                                                "Force_N",
                                                                testFailCount,
                                                                testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(truthTorqueExternalPntB_B,
                                                                srpTorqueData[1, 1:],
                                                                errTol,
                                                                "Torque",
                                                                testFailCount,
                                                                testMessages)
    if modelType == "lookup":
        errTolTorque = errTol/100
        truthForceExternal_B = [0.26720220706099184E-04, - 0.13596079145805012E-04, 0.93948649829282319E-05]
        truthForceExternal_N = [0, 0, 0]
        truthTorqueExternalPntB_B = [-0.80492463017846114E-12, 0.50888380426172319E-12, 0.10249431804585393E-11]
        if eclipseOn:
            truthForceExternal_B = sunEclipseMsgData.shadowFactor*np.array(truthForceExternal_B)
            truthTorqueExternalPntB_B = sunEclipseMsgData.shadowFactor * np.array(truthTorqueExternalPntB_B)
        testFailCount, testMessages = unitTestSupport.compareVector(truthForceExternal_B,
                                                                    srpDataForce_B[1, 1:],
                                                                    errTol,
                                                                    "Force_B",
                                                                    testFailCount,
                                                                    testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(truthForceExternal_N,
                                                                    srpDataForce_N[1, 1:],
                                                                    errTol,
                                                                    "Force_N",
                                                                    testFailCount,
                                                                    testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(truthTorqueExternalPntB_B,
                                                                    srpTorqueData[1, 1:],
                                                                    errTolTorque,
                                                                    "Torque",
                                                                    testFailCount,
                                                                    testMessages)
    if modelType == "cannonballLookup":
        errTolTorque = errTol/100
        testFailCount, testMessages = unitTestSupport.compareVector(srp2DataForce_N[1, 1:],
                                                                    srpDataForce_B[1, 1:],
                                                                    errTol,
                                                                    "Force_B",
                                                                    testFailCount,
                                                                    testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(srp2DataForce_B[1, 1:],
                                                                    srpDataForce_N[1, 1:],
                                                                    errTol,
                                                                    "Force_N",
                                                                    testFailCount,
                                                                    testMessages)
        testFailCount, testMessages = unitTestSupport.compareVector(srp2TorqueData[1, 1:],
                                                                    srpTorqueData[1, 1:],
                                                                    errTolTorque,
                                                                    "Torque",
                                                                    testFailCount,
                                                                    testMessages)


    if eclipseOn:
        modelType = modelType + 'WithEclipse'   #Do this so that the AutoTeX messages are clearly distinguishable.

    if testFailCount == 0:
        print("PASSED: " + modelType)
        passFailText = "PASSED"
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
        snippetName = modelType + 'FailMsg'
        snippetContent = ""
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    else:
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetName = modelType + 'FailMsg'
        snippetContent = passFailText
        for message in testMessages:
            snippetContent += ". " + message
        snippetContent += "."
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    snippetName = modelType + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    # write test accuracy to LATEX file for AutoTex
    snippetName = modelType + 'Accuracy'
    snippetContent = '{:1.1e}'.format(errTol)#write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.
    if modelType == 'lookupWithEclipse' or modelType == 'lookup' or modelType == 'cannonballLookup':
        snippetName = modelType + 'TorqueAccuracy'
        snippetContent = '{:1.1e}'.format(errTolTorque)  # write formatted LATEX string to file to be used by auto-documentation.
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent,
                                        path)  # write formatted LATEX string to file to be used by auto-documentation.

    if testFailCount:
        print(testMessages)
    else:
        print("PASSED")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitRadiationPressure(False, "cannonball", False)
