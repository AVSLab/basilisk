
# ISC License
#
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
#   Unit Test Script
#   Module Name:        sunSafePointCpp
#   Author:             Hanspeter Schaub
#   Creation Date:      April 25, 2018
#   Updated:            March 14, 2024
#

import inspect
import numpy as np
import os
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import sunSafePointCpp
from Basilisk.architecture import messaging
from Basilisk.utilities import macros as mc

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
#@pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("case", [
     (1)        # sun is visible, vectors are not aligned
    ,(2)        # sun is not visible, vectors are not aligned
    ,(3)        # sun is visible, vectors are aligned
    ,(4)        # sun is visible, vectors are oppositely aligned
    ,(5)        # sun is visible, vectors are oppositely aligned, and command sc is b1
    ,(6)        # sun is not visible, vectors are not aligned, no specified omega_RN_B value
    ,(7)        # sun is visible, vectors not aligned, nominal spin rate specified about sun heading vector
])

def test_module(show_plots, case):
    """Module Unit Test"""
    # Each test method requires a single assert method to be called
    [testResults, testMessage] = sunSafePointTestFunction(show_plots, case)
    assert testResults < 1, testMessage

def sunSafePointTestFunction(show_plots, case):
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = mc.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = sunSafePointCpp.SunSafePointCpp()
    module.ModelTag = "sunSafePoint"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    sHat_Cmd_B = np.array([0.0, 0.0 ,1.0])
    if case == 5:
        sHat_Cmd_B = np.array([1.0, 0.0, 0.0])
    module.setSHatBdyCmd(sHat_Cmd_B)
    module.setMinUnitMag(0.1)
    if case == 2:
        omega_RN_B_Search = np.array([0.0, 0.0, 0.1])
        module.setOmega_RN_B(omega_RN_B_Search)
    module.setSmallAngle(0.01*mc.D2R)

    # Create input messages
    inputSunVecData = messaging.NavAttMsgPayload()
    sunVec_B = np.array([1.0, 1.0, 0.0])
    if (case == 2 or case == 6):  # No sun visible, providing a near zero norm direction vector
        sunVec_B = [0.0, module.getMinUnitMag()/2, 0.0]
    if (case == 3):
        sunVec_B = sHat_Cmd_B
    if (case == 4 or case == 5):
        sunVec_B = -sHat_Cmd_B
    inputSunVecData.vehSunPntBdy = sunVec_B
    sunInMsg = messaging.NavAttMsg().write(inputSunVecData)

    inputIMUData = messaging.NavAttMsgPayload()  # Create a structure for the input message
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    inputIMUData.omega_BN_B = omega_BN_B
    imuInMsg = messaging.NavAttMsg().write(inputIMUData)

    if case == 7:
        module.setSunAxisSpinRate(1.5*mc.D2R)
        omega_RN_B_Search = sunVec_B/np.linalg.norm(sunVec_B) * module.getSunAxisSpinRate()

    # Set up data logging
    dataLog = module.attGuidanceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Connect messages
    module.sunDirectionInMsg.subscribeTo(sunInMsg)
    module.imuInMsg.subscribeTo(imuInMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(mc.sec2nano(1.))
    module.Reset(0)
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    #
    # Check sigma_BR
    #
    # Set the filtered output truth states
    if (case == 1 or case == 7):
        eHat = np.cross(sunVec_B,sHat_Cmd_B)
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B/np.linalg.norm(sunVec_B),sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi/4.0)
        trueVector = [
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist()
                   ]
    if (case == 2 or case == 3 or case == 6):
        trueVector = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
    if (case == 4):
        eHat = np.cross(sHat_Cmd_B,np.array([1,0,0]))
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B/np.linalg.norm(sunVec_B),sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi/4.0)
        trueVector = [
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist()
               ]
    if (case == 5):
        eHat = np.cross(sHat_Cmd_B, np.array([0, 1, 0]))
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B/np.linalg.norm(sunVec_B), sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi / 4.0)
        trueVector = [
            sigmaTrue.tolist(),
            sigmaTrue.tolist(),
            sigmaTrue.tolist()
        ]

    # Compare the module results to the truth values
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    for i in range(0,len(trueVector)):
        # Check a vector values
        if not unitTestSupport.isArrayEqual(dataLog.sigma_BR[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed sigma_BR unit test at t=" +
                                str(dataLog.times()[i] * mc.NANO2SEC) +
                                "sec\n")

    # Check omega_BR_B
    # Set the filtered output truth states
    if (case == 1 or case == 3 or case == 4 or case == 5 or case == 6):
        trueVector = [
            omega_BN_B.tolist(),
            omega_BN_B.tolist(),
            omega_BN_B.tolist()
        ]
    if (case == 2 or case == 7):
        trueVector = [
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist()
        ]
    # Compare the module results to the truth values
    for i in range(0,len(trueVector)):
        if not unitTestSupport.isArrayEqual(dataLog.omega_BR_B[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed omega_BR_B unit test at t=" +
                                str(dataLog.times()[i] * mc.NANO2SEC) +
                                "sec\n")


    # Check omega_RN_B
    # Set the filtered output truth states
    if (case == 1 or case == 3 or case == 4 or case == 5 or case == 6):
        trueVector = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
    if (case == 2 or case == 7):
        trueVector = [
            omega_RN_B_Search,
            omega_RN_B_Search,
            omega_RN_B_Search
        ]
    # Compare the module results to the truth values
    for i in range(0,len(trueVector)):
        if not unitTestSupport.isArrayEqual(dataLog.omega_RN_B[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed omega_RN_B unit test at t=" +
                                str(dataLog.times()[i] * mc.NANO2SEC) +
                                "sec\n")

    # Check domega_RN_B
    # Set the filtered output truth states
    trueVector = [
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0]
               ]

    # Compare the module results to the truth values
    for i in range(0,len(trueVector)):
        if not unitTestSupport.isArrayEqual(dataLog.domega_RN_B[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed domega_RN_B unit test at t=" +
                                str(dataLog.times()[i] * mc.NANO2SEC) +
                                "sec\n")

    # Print out success message if no error were found
    snippentName = "passFail" + str(case)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("FAILED: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
        print(testMessages)
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    sunSafePointTestFunction(False, 1)
