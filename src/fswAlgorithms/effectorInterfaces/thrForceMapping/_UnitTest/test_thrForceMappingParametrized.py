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
#   Unit Test Script
#   Module Name:        thrForceMapping
#   Author:             Hanspeter Schaub
#   Creation Date:      July 4, 2016
#

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupThrusters

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useDVThruster, useCOMOffset, dropThruster, dropAxis", [
      (False, False, False, False)
    , (False, False, False, True)
    , (False, True, False, False)
    # , (False, False, True, False)
    , (True, False, False, False)
    , (True, False, True, False)
    , (True, True, False, False)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, useDVThruster, useCOMOffset, dropThruster, dropAxis):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, dropAxis)
    assert testResults < 1, testMessage


def thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, dropAxis):
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
    vehicleConfigOut = thrForceMapping.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    if useCOMOffset == 1:
        CoM_B = [0,0,0.02]
    else:
        CoM_B = [0,0,0]
    vehicleConfigOut.CoM_B = CoM_B
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = thrForceMapping.CmdTorqueBodyIntMsg()  # Create a structure for the input message
    inputMessageSize = inputMessageData.getStructSize()                           # 3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    requestedTorque = [1.0, -0.5, 0.7]              # Set up a list as a 3-vector
    inputMessageData.torqueRequestBody = requestedTorque   # write torque request to input message
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          0,
                                          inputMessageData)             # write data into the simulator

    moduleConfig.epsilon = 0.0005
    fswSetupThrusters.clearSetup()
    if useDVThruster:
        # DV thruster setup
        moduleConfig.thrForceSign = -1
        moduleConfig.controlAxes_B = [
              1, 0, 0
            , 0, 1, 0
        ]
        if dropThruster:
            rcsLocationData = [ \
            [0.35766849176297305, 0.20650000000000013, -0.1671],
            [0.3576684917629732, -0.20649999999999988, -0.1671],
            [-0.35766849176297305, -0.20650000000000018, -0.1671],
            [-0.35766849176297333, 0.20649999999999968, -0.1671] \
                    ]
            rcsDirectionData = [ \
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0] \
                ]
        else:
            rcsLocationData = [ \
                [0, 0.413, -0.1671],
                [0.35766849176297305, 0.20650000000000013, -0.1671],
                [0.3576684917629732, -0.20649999999999988, -0.1671],
                [0, -0.413, -0.1671],
                [-0.35766849176297305, -0.20650000000000018, -0.1671],
                [-0.35766849176297333, 0.20649999999999968, -0.1671] \
                ]
            rcsDirectionData = [ \
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0] \
                ]

    else:
        # RCS thruster setup
        moduleConfig.thrForceSign = +1
        if dropAxis:
            moduleConfig.controlAxes_B = [
                1, 0, 0
                , 0, 0, 1
            ]
        else:
            moduleConfig.controlAxes_B = [
                  1, 0, 0
                , 0, 1, 0
                , 0, 0, 1
            ]
        if dropThruster:
            rcsLocationData = [ \
                [-0.86360, -0.82550, 1.79070],
                [-0.82550, -0.86360, 1.79070],
                [0.82550, 0.86360, 1.79070],
                [0.86360, 0.82550, 1.79070],
                [-0.86360, -0.82550, -1.79070],
                [-0.82550, -0.86360, -1.79070],
                [0.82550, 0.86360, -1.79070] \
                ]
            rcsDirectionData = [ \
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0] \
                ]
        else:
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
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0] \
                ]



    for i in range(len(rcsLocationData)):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], 0.95)
    fswSetupThrusters.writeConfigMessage(  moduleConfig.inputThrusterConfName,
                                           unitTestSim.TotalSim,
                                           unitProcessName)
    numThrusters = fswSetupThrusters.getNumOfDevices()

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "thrForce"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(numThrusters))
    print moduleOutput

    # set the output truth states
    trueVector=[];
    if useDVThruster:
        # DV Thruster results
        if useCOMOffset:
            trueVector = [
                [0, 0, -0.1081312318123977, -1.6142050040355125, -1.5060737722231148, 0],
                [0, 0, -0.1081312318123977, -1.6142050040355125, -1.5060737722231148, 0]
            ]
        elif dropThruster:
            trueVector = [
                [0, -1.722336235847909, -3.120278776258626, 0],
                [0, -1.722336235847909, -3.120278776258626, 0]
            ]
        else:
            trueVector = [
                [0, 0, -0.1081312318123977, -1.6142050040355125, -1.5060737722231148, 0],
                [0, 0, -0.1081312318123977, -1.6142050040355125, -1.5060737722231148, 0]
            ]
    else:
        # RCS thruster results
        if dropAxis:
            trueVector = [
                [0.351603, 0., 0.27922, 0.351603, 0.351603, 0.27922, 0., 0.351603],
                [0.351603, 0., 0.27922, 0.351603, 0.351603, 0.27922, 0., 0.351603]
            ]
        elif useCOMOffset:
            trueVector = [
                [0.284128, 0.00311817, 0.279186, 0.422161, 0.423721, 0.282304, 0., 0.282569],
                [0.284128, 0.00311817, 0.279186, 0.422161, 0.423721, 0.282304, 0., 0.282569]
            ]
        elif dropThruster:
            trueVector = [
                [0.563596,0,0.27922,0.421408,0.421408,0.27922,0],
                [0.563596,0,0.27922,0.421408,0.421408,0.27922,0]
            ]
        else:
            trueVector = [
                [0.281798, 0., 0.27922, 0.421408, 0.421408, 0.27922, 0., 0.281798],
                [0.281798, 0., 0.27922, 0.421408, 0.421408, 0.27922, 0., 0.281798]
            ]

    print trueVector


    # compare the module results to the truth values
    accuracy = 1e-6
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
                 False,           # useDVThruster
                 False,           # use COM offset
                 False,           # drop thruster(s)
                 False            # drop control axis
    )
