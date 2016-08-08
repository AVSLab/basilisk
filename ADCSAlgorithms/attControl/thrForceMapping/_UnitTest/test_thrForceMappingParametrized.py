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
#   Module Name:        thrForceMapping
#   Author:             Hanspeter Schaub
#   Creation Date:      July 4, 2016
#

import pytest
import sys, os, inspect
import matplotlib.pyplot as plt
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

# Import all of the modules that we are going to be called in this simulation
import SimulationBaseClass
import alg_contain
import unitTestSupport                  # general support file with common unit test functions
import thrForceMapping
import macros
import MRP_Steering
import vehicleConfigData

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("ignoreAxis2, useCOMOffset, dropThruster, use2ndLr, useNegForce", [
    (False, False, False, False, False)
    ,(True, False, False, False, False)
    ,(False, True, False, False, False)
    ,(False, True, True, False, False)
    ,(False, True, True, True, False)
    ,(False, True, False, False, True)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, ignoreAxis2, useCOMOffset,dropThruster, use2ndLr, useNegForce):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrusterForceTest(show_plots, ignoreAxis2, useCOMOffset, dropThruster,
                                                   use2ndLr, useNegForce)
    assert testResults < 1, testMessage


def thrusterForceTest(show_plots, ignoreAxis2, useCOMOffset, dropThruster, use2ndLr, useNegForce):
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
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = thrForceMapping.thrForceMappingConfig()                          # update with current values
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        thrForceMapping.Update_thrForceMapping,
                                        thrForceMapping.SelfInit_thrForceMapping,
                                        thrForceMapping.CrossInit_thrForceMapping,
                                        thrForceMapping.Reset_thrForceMapping)
    moduleWrap.ModelTag = "thrForceMapping"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputVehControlName = "LrRequested"
    moduleConfig.inputThrusterConfName = "RCSThrusters"
    moduleConfig.outputDataName = "thrusterForceOut"
    moduleConfig.inputVehicleConfigDataName = "vehicleConfigName"

    # write vehicle configuration message
    inputMessageSize = 21 * 8 + 8  # 21 doubles + 1 32bit integer
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    BS = [1.0, 0., 0.,
         0., 1.0, 0.,
         0., 0., 1.0]
    if useCOMOffset == 1:
        CoM_S = [0.1, 0.05, 0.01]
    else:
        CoM_S = [0,0,0]
    SimulationBaseClass.SetCArray(BS,
                                  'double',
                                  vehicleConfigOut.BS)
    SimulationBaseClass.SetCArray(CoM_S,
                                  'double',
                                  vehicleConfigOut.CoM_B)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageSize = 3*8                              # 3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    inputMessageData = MRP_Steering.vehControlOut()     # Create a structure for the input message
    if use2ndLr:
        requestedTorque = [0.1, 0.16, 0.005]            # Set up a list as a 3-vector
    else:
        requestedTorque = [1.0, -0.5, 0.7]              # Set up a list as a 3-vector
    SimulationBaseClass.SetCArray(requestedTorque,                      # specify message variable
                                  'double',                             # specify message variable type
                                  inputMessageData.torqueRequestBody)   # write torque request to input message
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          0,
                                          inputMessageData)             # write data into the simulator

    if useNegForce:
        moduleConfig.thrForceSign = -1
    else:
        moduleConfig.thrForceSign = +1

    moduleConfig.epsilon = 0.0005
    if ignoreAxis2==0:
        controlAxes_B = [
             1,0,0
            ,0,1,0
            ,0,0,1
        ]
    else:
        controlAxes_B = [
             1,0,0
            ,0,0,1
        ]

    SimulationBaseClass.SetCArray(controlAxes_B, 'double', moduleConfig.controlAxes_B)

    rcsClass = vehicleConfigData.ThrusterCluster()
    rcsPointer = vehicleConfigData.ThrusterPointData()
    if dropThruster:
        numThrusters = 7;
        rcsLocationData = [ \
                   [-0.86360, -0.82550, 1.79070],
                   [-0.82550, -0.86360, 1.79070],
                   [0.82550, 0.86360, 1.79070],
                   [0.86360, 0.82550, 1.79070],
                   [-0.86360, -0.82550, -1.79070],
                   [-0.82550, -0.86360, -1.79070],
                   [0.82550, 0.86360, -1.79070]\
                   ]
        rcsDirectionData = [ \
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 1.0, 0.0]\
                        ]
    else:
        numThrusters = 8;
        rcsLocationData = [ \
                   [-0.86360, -0.82550, 1.79070],
                   [-0.82550, -0.86360, 1.79070],
                   [0.82550, 0.86360, 1.79070],
                   [0.86360, 0.82550, 1.79070],
                   [-0.86360, -0.82550, -1.79070],
                   [-0.82550, -0.86360, -1.79070],
                   [0.82550, 0.86360, -1.79070],
                   [0.86360, 0.82550, -1.79070] \
                   ]
        rcsDirectionData = [ \
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [1.0, 0.0, 0.0] \
                        ]
    moduleConfig.numThrusters = numThrusters


    for i in range(numThrusters):
        SimulationBaseClass.SetCArray(rcsLocationData[i], 'double', rcsPointer.rThrust_S)
        SimulationBaseClass.SetCArray(rcsDirectionData[i], 'double', rcsPointer.tHatThrust_S)
        vehicleConfigData.ThrustConfigArray_setitem(rcsClass.thrusters, i, rcsPointer)

    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, moduleConfig.inputThrusterConfName,
                           vehicleConfigData.MAX_EFF_CNT*6*8, 2)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputThrusterConfName, vehicleConfigData.MAX_EFF_CNT*6*8, 0, rcsClass)




    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    moduleWrap.Reset(0)

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "effectorRequest"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(numThrusters))

    # set the output truth states
    trueVector=[];
    if ignoreAxis2==0:
        if useCOMOffset == 0:
            trueVector = [
                       [0.1396102082984308,0.4912131482746326,0.2119927316777711,0,0.3516029399762018,0.2792204165968615,0,0.2119927316777711],
                       [0.1396102082984308,0.4912131482746326,0.2119927316777711,0,0.3516029399762018,0.2792204165968615,0,0.2119927316777711]
                       ]
        else:
            if dropThruster:
                if use2ndLr:
                    trueVector = [
                        [0,0.0309505092550829,0.00302846759539673,0.07552754243563822,0,0.02792204165968615,0],
                        [0,0.0309505092550829,0.00302846759539673,0.07552754243563822,0,0.02792204165968615,0]
                    ]
                else:
                    trueVector = [
                        [0.1396102082984308, 0.7032058799524038, 0.4239854633555422, 0, 0.1396102082984307,
                         0.2792204165968615, 0],
                        [0.1396102082984308, 0.7032058799524038, 0.4239854633555422, 0, 0.1396102082984307,
                         0.2792204165968615, 0]
                    ]
            else:
                if useNegForce:
                    trueVector = [
                               [-0.211992731677771,0,-0.2792204165968616,-0.3516029399762018,0,-0.211992731677771,-0.4912131482746326,-0.1396102082984309],
                               [-0.211992731677771,0,-0.2792204165968616,-0.3516029399762018,0,-0.211992731677771,-0.4912131482746326,-0.1396102082984309]
                               ]
                else:
                    trueVector = [
                               [0.1396102082984308,0.4912131482746326,0.2119927316777711,0,0.3516029399762017,0.2792204165968615,0,0.2119927316777711],
                               [0.1396102082984308,0.4912131482746326,0.2119927316777711,0,0.3516029399762017,0.2792204165968615,0,0.2119927316777711]
                               ]
    else:
        if ignoreAxis2==1:
            trueVector = [
           [0,0.4912131482746326,0.2119927316777711,0,0.2119927316777711,0.2792204165968615,0,0.2119927316777711],
           [0,0.4912131482746326,0.2119927316777711,0,0.2119927316777711,0.2792204165968615,0,0.2119927316777711]
                   ]
        else:
            testFailCount+=1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")


    # compare the module results to the truth values
    accuracy = 1e-8
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], numThrusters, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
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
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,
                 False,           # ignoreAxis2 value
                 False,           # use COM offset
                 False,           # drop last thruster
                 False,           # use alternate Lr torque
                 False            # use pos/neg thruster forces
               )
