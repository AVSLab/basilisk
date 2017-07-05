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
# RadiationPressure Unit Test
#
# Purpose:  Test the proper function of the Radiation Pressure Dynamics module.
#           This is done by comparing expected torques and forces to
#           what is sumulated
# Author:   Patrick Kenneally
# Creation Date:  Feb. 9, 2017
#

# @cond DOXYGEN_IGNORE
import sys
import os
import numpy as np
import pytest
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
# @endcond

#Import all of the modules that we are going to call in this simulation
import SimulationBaseClass
import unitTestSupport
import spacecraftPlus
import radiation_pressure
import macros
import spice_interface

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("modelType, eclipseOn", [
      ("cannonball",False)
    , ("lookup", False)
    , ("lookup", True)
])
def test_unitRadiationPressure(show_plots, modelType, eclipseOn):
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
    unitTestSim.TotalSim.terminateSimulation()
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    srpDynEffector = radiation_pressure.RadiationPressure()
    srpDynEffector.ModelTag = "RadiationPressure"

    if modelType == "cannonball":
        srpDynEffector.setUseCannonballModel(True)
        srpDynEffector.area = 4
        srpDynEffector.coefficientReflection = 1.2
    elif modelType == "lookup":
        srpDynEffector.setUseCannonballModel(False)
        handler = radiation_pressure.SRPLookupTableHandler()
        handler.parseAndLoadXML(os.path.dirname(__file__) + "/cube_lookup.xml")
        for i in xrange(0, len(handler.forceBLookup)):
            srpDynEffector.addForceLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.forceBLookup[i, :]))
            srpDynEffector.addTorqueLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.torqueBLookup[i, :]))
            srpDynEffector.addSHatLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.sHatBLookup[i, :]))
    if eclipseOn:
        sunEclipseInMsgName = "sun_eclipse"
        sunEclipseMsgData = radiation_pressure.EclipseSimMsg()
        sunEclipseMsgData.shadowFactor = 0.5
        unitTestSupport.setMessage(unitTestSim.TotalSim, testProcessName, sunEclipseInMsgName, sunEclipseMsgData)
        srpDynEffector.sunEclipseInMsgName = sunEclipseInMsgName

    unitTestSim.AddModelToTask(testTaskName, srpDynEffector, None, 3)

    scPlusStateMsg = spacecraftPlus.SCPlusStatesSimMsg()
    scPlusStateMsgName = "inertial_state_output"
    unitTestSim.TotalSim.CreateNewMessage(testProcessName, scPlusStateMsgName, scPlusStateMsg.getStructSize(), 2)
    scPlusStateMsg.r_BN_N = r_N
    scPlusStateMsg.sigma_BN = sigma_BN
    unitTestSim.TotalSim.WriteMessageData(scPlusStateMsgName, scPlusStateMsg.getStructSize(), 1, scPlusStateMsg)

    sunSpiceMsg = spice_interface.SpicePlanetStateSimMsg()
    sunSpiceMsgName = "sun_planet_data"
    unitTestSim.TotalSim.CreateNewMessage(testProcessName, sunSpiceMsgName, sunSpiceMsg.getStructSize(), 2)
    sunSpiceMsg.PositionVector = sun_r_N
    unitTestSim.TotalSim.WriteMessageData(sunSpiceMsgName, sunSpiceMsg.getStructSize(), 1, sunSpiceMsg)

    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".forceExternal_B",
                                      simulationTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".forceExternal_N",
                                      simulationTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".torqueExternalPntB_B",
                                      simulationTime, 0, 2, 'double')

    unitTestSim.InitializeSimulation()

    # Configure a simulation stop time time and execute the simulation run
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    srpDynEffector.computeBodyForceTorque(unitTestSim.TotalSim.CurrentNanos)
    unitTestSim.TotalSim.SingleStepProcesses()
    unitTestSim.RecordLogVars()

    srpDataForce_B = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_B")
    srpDataForce_N = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_N")
    srpTorqueData = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".torqueExternalPntB_B")

    errTol = 1E-12
    if modelType == "cannonball":
        truthForceExternal_B = [1.85556867482797E-05, -8.82911772465848E-06, 5.64107588371567E-06]
        truthForceExternal_N = [0, 0, 0]
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
                                                                    errTol/100,
                                                                    "Torque",
                                                                    testFailCount,
                                                                    testMessages)

    if testFailCount == 0:
        print "PASSED: " + modelType
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
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    # write test accuracy to LATEX file for AutoTex
    snippetName = modelType + 'Accuracy'
    snippetContent = '{:1.1e}'.format(errTol)#write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitRadiationPressure(False, "cannonball")
