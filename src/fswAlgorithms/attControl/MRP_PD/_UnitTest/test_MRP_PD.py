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
import numpy as np
import pytest
import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))







from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.fswAlgorithms import MRP_PD  # import the module that is to be tested
from Basilisk.utilities import macros

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("setExtTorque", [False, True])

def test_mrp_PD_tracking(show_plots, setExtTorque):
    [testResults, testMessage] = mrp_PD_tracking(show_plots, setExtTorque)
    assert testResults < 1, testMessage


def mrp_PD_tracking(show_plots, setExtTorque):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_PD_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = MRP_PD.MRP_PDConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "MRP_PD"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputGuidName = "inputGuidName"
    moduleConfig.inputVehicleConfigDataName = "vehicleConfigName"
    moduleConfig.outputDataName = "outputName"

    moduleConfig.K = 0.15
    moduleConfig.P = 150.0
    if setExtTorque:
        moduleConfig.knownTorquePntB_B = [0.1, 0.2, 0.3]

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.
    #   attGuidOut Message:
    guidCmdData = MRP_PD.AttGuidFswMsg()  # Create a structure for the input message
    guidCmdData.sigma_BR = [0.3, -0.5, 0.7]
    guidCmdData.omega_BR_B = [0.010, -0.020, 0.015]
    guidCmdData.omega_RN_B = [-0.02, -0.01, 0.005]
    guidCmdData.domega_RN_B = [0.0002, 0.0003, 0.0001]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputGuidName, guidCmdData)

    # vehicleConfig FSW Message:
    vehicleConfigOut = MRP_PD.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = [1000., 0., 0.,
                                  0., 800., 0.,
                                  0., 0., 800.]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName,
                               moduleConfig.inputVehicleConfigDataName, vehicleConfigOut)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))

    trueVector = [findTrueTorques(moduleConfig, guidCmdData, vehicleConfigOut)]*3
    # print trueVector

    # compare the module results to the truth values
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput, accuracy,
                                                               "torqueRequestBody", testFailCount, testMessages)

    snippentName = "passFail" + str(setExtTorque)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def findTrueTorques(moduleConfig, guidCmdData, vehicleConfigOut):
    sigma_BR = np.array(guidCmdData.sigma_BR)
    omega_BR_B = np.array(guidCmdData.omega_BR_B)
    omega_RN_B = np.array(guidCmdData.omega_RN_B)
    domega_RN_B = np.array(guidCmdData.domega_RN_B)

    I = np.identity(3)
    I[0][0] = vehicleConfigOut.ISCPntB_B[0]
    I[1][1] = vehicleConfigOut.ISCPntB_B[4]
    I[2][2] = vehicleConfigOut.ISCPntB_B[8]

    K = moduleConfig.K
    P = moduleConfig.P
    L = np.array(moduleConfig.knownTorquePntB_B)

    # Begin Method
    omega_BN_B = omega_BR_B + omega_RN_B
    temp1 = np.dot(I, omega_BN_B)
    temp2 = domega_RN_B - np.cross(omega_BN_B, omega_RN_B)
    Lr = K * sigma_BR + P * omega_BR_B - np.cross(omega_RN_B, temp1) - np.dot(I, temp2)
    Lr += L
    Lr *= -1.0

    return Lr



if __name__ == "__main__":
    test_mrp_PD_tracking(False, False)


