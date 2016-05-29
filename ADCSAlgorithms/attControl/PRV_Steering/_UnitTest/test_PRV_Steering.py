'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#   Unit Test Script
#   Module Name:        PRV_Steering
#   Author:             Hanspeter Schaub
#   Creation Date:      December 18, 2015
#
import pytest
import sys, os, inspect
import matplotlib.pyplot as plt
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np
import ctypes
import math
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')


#   Import all of the modules that we are going to call in this simulation
import MessagingAccess
import SimulationBaseClass
import sim_model
import alg_contain
import unitTestSupport                  # general support file with common unit test functions
import PRV_Steering                     # import the module that is to be tested
import sunSafePoint                     # import module(s) that creates the needed input message declaration
import vehicleConfigData                # import module(s) that creates the needed input message declaration
import macros

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_PRV_Steering(show_plots):     # update "subModule" in this function name to reflect the module name
    # each test method requires a single assert method to be called
    [testResults, testMessage] = subModuleTestFunction(show_plots)
    assert testResults < 1, testMessage


def subModuleTestFunction(show_plots):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()          # this is needed if multiple unit test scripts are run
                                                        # this create a fresh and consistent simulation environment for each test run

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    #   Construct algorithm and associated C++ container
    moduleConfig = PRV_Steering.PRV_SteeringConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        PRV_Steering.Update_PRV_Steering,
                                        PRV_Steering.SelfInit_PRV_Steering,
                                        PRV_Steering.CrossInit_PRV_Steering,
                                        PRV_Steering.Reset_PRV_Steering)
    moduleWrap.ModelTag = "PRV_Steering"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    #   Initialize the test module configuration data
    moduleConfig.inputGuidName  = "inputGuidName"
    moduleConfig.inputVehicleConfigDataName  = "vehicleConfigName"
    moduleConfig.outputDataName = "outputName"

    moduleConfig.K1 =   0.15
    moduleConfig.K3 =   1.0
    moduleConfig.Ki =   0.01
    moduleConfig.P  = 150.0
    moduleConfig.omega_max = 1.5*macros.D2R
    moduleConfig.integralLimit = 2./moduleConfig.Ki * 0.1;


    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.

    #   attGuidOut Message:
    inputMessageSize = 12*8                            # 4x3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputGuidName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    guidCmdData = sunSafePoint.attGuidOut()             # Create a structure for the input message
    sigma_BR = np.array([0.3, -0.5, 0.7])
    SimulationBaseClass.SetCArray(sigma_BR,
                                  'double',
                                  guidCmdData.sigma_BR)
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    SimulationBaseClass.SetCArray(omega_BR_B,
                                  'double',
                                  guidCmdData.omega_BR_B)
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    SimulationBaseClass.SetCArray(omega_RN_B,
                                  'double',
                                  guidCmdData.omega_RN_B)
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    SimulationBaseClass.SetCArray(domega_RN_B,
                                  'double',
                                  guidCmdData.domega_RN_B)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputGuidName,
                                          inputMessageSize,
                                          0,
                                          guidCmdData)


    # vehicleConfigData Message:
    inputMessageSize = 18*8+8                           # 18 doubles + 1 32bit integer
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    SimulationBaseClass.SetCArray(I,
                                  'double',
                                  vehicleConfigOut.I)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)



    #   Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds) 
    
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()


    #   This pulls the actual data log from the simulation run.
    #   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                    range(3))


    # set the filtered output truth states
    trueVector = [
               [2.9352922876097969, -6.2831737715827778, 4.0554726129822907]
              ,[2.9352922876097969, -6.2831737715827778, 4.0554726129822907]
              ,[2.9353853745179044, -6.2833455830962901, 4.0556481491012084]
              ,[2.9352922876097969, -6.2831737715827778, 4.0554726129822907]
              ,[2.9353853745179044, -6.2833455830962901, 4.0556481491012084]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessage.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(moduleOutput[i,0]*macros.NANO2SEC) + "sec\n")





    ## plot a sample variable
    #plt.figure(1)
    #plt.plot(dummyState[:,0]*macros.NANO2SEC, dummyState[:,1], label='Sample Variable')
    #plt.legend(loc='upper left')
    #plt.xlabel('Time [s]')
    #plt.ylabel('Variable Description [unit]')





    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
          plt.show()

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]





#
#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   authmatically executes the runUnitTest() method
#
if __name__ == "__main__":
    test_PRV_Steering(False)
