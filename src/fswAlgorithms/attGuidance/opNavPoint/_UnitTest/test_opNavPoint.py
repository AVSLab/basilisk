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
#
#   Copy of the unit test for sunSafe Point adapted to any heading
#   Module Name:        opNavPoint
#   Author:             Thibaud Teil
#   Creation Date:      August 20, 2019
#

import pytest
import sys, os, inspect
import numpy as np
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))



# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms.opNavPoint import opNavPoint                   # import the module that is to be tested
from Basilisk.fswAlgorithms.fswMessages import OpNavFswMsg
from Basilisk.simulation.simFswInterfaceMessages import simFswInterfaceMessages
from Basilisk.utilities import macros as mc


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
#@pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("case", [
     (1)        # target is visible, vectors are not aligned
    ,(2)        # target is not visible, vectors are not aligned
    ,(3)        # target is visible, vectors are aligned
    ,(4)        # target is not visible, search
])

def test_module(show_plots, case):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = opNavPointTestFunction(show_plots, case)
    assert testResults < 1, testMessage


def opNavPointTestFunction(show_plots, case):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = mc.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = opNavPoint.OpNavPointConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "opNavPoint"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.attGuidanceOutMsgName = "outputName"
    moduleConfig.opnavDataInMsgName = "inputOpNavName"
    moduleConfig.imuInMsgName = "inputIMUDataName"
    moduleConfig.cameraConfigMsgName = "camera_config_data"
    camera_Z = [0.,0.,1.]
    moduleConfig.alignAxis_C = camera_Z
    moduleConfig.minUnitMag = 0.01
    moduleConfig.smallAngle = 0.01*mc.D2R
    moduleConfig.timeOut = 100

    # Create input messages
    #
    planet_B = [1.,1.,0.]
    inputOpNavData = OpNavFswMsg()  # Create a structure for the input message
    inputOpNavData.r_BN_C = planet_B
    inputOpNavData.valid = 1
    if (case == 2): #No valid measurement
        inputOpNavData.valid = 0
    if (case == 3): #No valid measurement
        inputOpNavData.r_BN_C = [0.,0.,-1.]
    if (case == 4): #No valid measurement
        inputOpNavData.valid = 0
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.opnavDataInMsgName,
                               inputOpNavData)

    inputIMUData = simFswInterfaceMessages.NavAttIntMsg()  # Create a structure for the input message
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    inputIMUData.omega_BN_B = omega_BN_B
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.imuInMsgName,
                               inputIMUData)
    omega_RN_B_Search = np.array([0.0, 0.0, 0.1])
    if (case ==2 or case==4):
        moduleConfig.omega_RN_B = omega_RN_B_Search

    cam = simFswInterfaceMessages.CameraConfigMsg()  # Create a structure for the input message
    cam.sigma_CB = [0.,0.,0]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.cameraConfigMsgName,
                               cam)


    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.attGuidanceOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(mc.sec2nano(1.))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    #
    # check sigma_BR
    #
    moduleOutputName = "sigma_BR"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attGuidanceOutMsgName + '.' + moduleOutputName,
                                                  list(range(3)))
    # set the filtered output truth states

    eHat = np.cross(-np.array(planet_B), np.array(camera_Z))
    eHat = eHat / np.linalg.norm(eHat)
    Phi = np.arccos(np.dot(-np.array(planet_B)/np.linalg.norm(-np.array(planet_B)),np.array(camera_Z)))
    sigmaTrue = eHat * np.tan(Phi/4.0)
    trueVector = [
                sigmaTrue.tolist(),
                sigmaTrue.tolist(),
                sigmaTrue.tolist()
               ]
    if (case == 2 or case == 3 or case == 4):
        trueVector = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
    # compare the module results to the truth values
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0] * mc.NANO2SEC) +
                                "sec\n")

    #
    # check omega_BR_B
    #
    moduleOutputName = "omega_BR_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attGuidanceOutMsgName + '.' + moduleOutputName,
                                                  list(range(3)))
    # set the filtered output truth states
    trueVector = [
        omega_BN_B.tolist(),
        omega_BN_B.tolist(),
        omega_BN_B.tolist()
    ]
    if (case == 2 or case==4):
        trueVector = [
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist()
        ]
    # compare the module results to the truth values
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0] * mc.NANO2SEC) +
                                "sec\n")
    #
    # check omega_RN_B
    #
    moduleOutputName = "omega_RN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attGuidanceOutMsgName + '.' + moduleOutputName,
                                                  list(range(3)))
    # set the filtered output truth states
    trueVector = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    if (case == 2 or case == 4):
        trueVector = [
            omega_RN_B_Search,
            omega_RN_B_Search,
            omega_RN_B_Search
        ]
    # compare the module results to the truth values
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0] * mc.NANO2SEC) +
                                "sec\n")

    #
    # check domega_RN_B
    #
    moduleOutputName = "domega_RN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attGuidanceOutMsgName + '.' + moduleOutputName,
                                                  list(range(3)))
    # set the filtered output truth states
    trueVector = [
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0]
               ]

    # compare the module results to the truth values
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0] * mc.NANO2SEC) +
                                "sec\n")


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
   # if show_plots:
   #     # plot a sample variable.
   #     plt.figure(1)
   #     plt.plot(variableState[:,0]*macros.NANO2SEC, variableState[:,1], label='Sample Variable')
   #     plt.legend(loc='upper left')
   #     plt.xlabel('Time [s]')
   #     plt.ylabel('Variable Description [unit]')
   #     plt.show()


    #   print out success message if no error were found
    snippentName = "passFail" + str(case)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + moduleWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("FAILED: " + moduleWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)



    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    opNavPointTestFunction(False, 1)
