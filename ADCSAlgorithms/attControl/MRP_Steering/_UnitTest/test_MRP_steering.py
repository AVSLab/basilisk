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
import sys, os, inspect
import matplotlib.pyplot as plt
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import alg_contain
import unitTestSupport                  # general support file with common unit test functions
import MRP_Steering                     # import the module that is to be tested
import sunSafePoint                     # import module(s) that creates the needed input message declaration
import simple_nav                       # import module(s) that creates the needed input message declaration
import vehicleConfigData                # import module(s) that creates the needed input message declaration
import rwNullSpace

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_mrp_steering_tracking(show_plots):
    [testResults, testMessage] = mrp_steering_tracking(show_plots)
    assert testResults < 1, testMessage


def mrp_steering_tracking(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                        # create empty list to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = unitTestSupport.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = MRP_Steering.MRP_SteeringConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        MRP_Steering.Update_MRP_Steering,
                                        MRP_Steering.SelfInit_MRP_Steering,
                                        MRP_Steering.CrossInit_MRP_Steering,
                                        MRP_Steering.Reset_MRP_Steering)
    moduleWrap.ModelTag = "MRP_Steering"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputGuidName  = "inputGuidName"
    moduleConfig.inputNavName = "inputNavName"
    moduleConfig.inputVehicleConfigDataName  = "vehicleConfigName"
    moduleConfig.wheelSpeedsName  = "reactionwheel_speeds"
    moduleConfig.outputDataName = "outputName"
    

    moduleConfig.K1 = 0.15
    moduleConfig.K3 = 1.0
    moduleConfig.Ki = 0.01
    moduleConfig.P = 150.0
    moduleConfig.numRWAs = 4
    moduleConfig.omega_max = 1.5*unitTestSupport.D2R
    moduleConfig.integralLimit = 2./moduleConfig.Ki * 0.1
    SimulationBaseClass.SetCArray([.1,.1,.1,.1],        # set RW spin inertia Js values
                                  'double',
                                  moduleConfig.JsList)
    SimulationBaseClass.SetCArray([
                                1,0,0,
                                0,1,0,
                                0,0,1,
                                0.5773502691896258, 0.5773502691896258, 0.5773502691896258
                                ],        # set RW spin axes unit vector g_s
                                  'double',
                                  moduleConfig.GsMatrix)
                             

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.
    #   attGuidOut Message:
    inputMessageSize = 12*8                            # 4x3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputGuidName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    guidCmdData = sunSafePoint.attGuidOut()             # Create a structure for the input message
    sigma_BR = [0.3, -0.5, 0.7]
    SimulationBaseClass.SetCArray(sigma_BR,
                                  'double',
                                  guidCmdData.sigma_BR)
    omega_BR_B = [0.010, -0.020, 0.015]
    SimulationBaseClass.SetCArray(omega_BR_B,
                                  'double',
                                  guidCmdData.omega_BR_B)
    omega_RN_B = [-0.02, -0.01, 0.005]
    SimulationBaseClass.SetCArray(omega_RN_B,
                                  'double',
                                  guidCmdData.omega_RN_B)
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    SimulationBaseClass.SetCArray(domega_RN_B,
                                  'double',
                                  guidCmdData.domega_RN_B)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputGuidName,
                                          inputMessageSize,
                                          0,
                                          guidCmdData)

    # NavStateOut Message:
    inputMessageSize = 18*8                             # 6x3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputNavName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    NavStateOutData = simple_nav.NavStateOut()          # Create a structure for the input message
    sigma_BN = [0.25, -0.45, 0.75]
    SimulationBaseClass.SetCArray(sigma_BN,
                                  'double',
                                  NavStateOutData.sigma_BN)
    omega_BN_B = [-0.015, -0.012, 0.005]
    SimulationBaseClass.SetCArray(omega_BN_B,
                                  'double',
                                  NavStateOutData.omega_BN_B)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputNavName,
                                          inputMessageSize,
                                          0,
                                          NavStateOutData)

    # wheelSpeeds Message
    inputMessageSize = 36*8                               # 36 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    rwSpeedMessage = rwNullSpace.RWSpeedData()
    Omega = [10.0, 25.0, 50.0, 100.0];
    SimulationBaseClass.SetCArray(Omega,
                                  'double',
                                  rwSpeedMessage.wheelSpeeds)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          0,
                                          rwSpeedMessage)
    
    
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

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()
    
    moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds) 
    
    unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))
    
    # set the filtered output truth states
    trueVector = [
               [3.51929003225847,-5.043242796061465,3.475469832430654]
              ,[3.51929003225847,-5.043242796061465,3.475469832430654]
              ,[3.519426975703996,-5.043376767319374,3.475597633355197]
              ,[3.51929003225847,-5.043242796061465,3.475469832430654]
              ,[3.519426975703996,-5.043376767319374,3.475597633355197]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName +
                               " unit test at t=" + str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) +
                               "sec \n")

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    #   print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED: " + moduleWrap.ModelTag

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_mrp_steering_tracking(False)