#
#   Unit Test Script
#   Module Name:        velocityPoint
#   Author:             Mar Cols
#   Creation Date:      January 22, 2016
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
import velocityPoint                        # import the module that is to be tested
import simple_nav                       # import module(s) that creates the needed input message declaration
import spice_interface


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_velocityPoint(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = velocityPointTestFunction(show_plots)
    assert testResults < 1, testMessage


def velocityPointTestFunction(show_plots):
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
    testProcessRate = unitTestSupport.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = velocityPoint.velocityPointConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        velocityPoint.Update_velocityPoint,
                                        velocityPoint.SelfInit_velocityPoint,
                                        velocityPoint.CrossInit_velocityPoint)
    moduleWrap.ModelTag = "velocityPoint"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputNavDataName  = "inputNavName"
    moduleConfig.inputCelMessName = "inputCelName"
    moduleConfig.outputDataName = "outputName"


    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    #
    #   Navigation Input Message
    #
    inputNavMessageSize = 18*8                             # 6x3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputNavDataName,
                                          inputNavMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    NavStateOutData = simple_nav.NavStateOut()          # Create a structure for the input message
    r_BN_N = [500., 500., 1000.]
    SimulationBaseClass.SetCArray(r_BN_N,
                                  'double',
                                  NavStateOutData.r_BN_N)
    v_BN_N = [10., 10., 0.]
    SimulationBaseClass.SetCArray(v_BN_N,
                                  'double',
                                  NavStateOutData.v_BN_N)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputNavDataName,
                                          inputNavMessageSize,
                                          0,
                                          NavStateOutData)
    #
    #   Spice Input Message
    #
    inputCelMessageSize = (1 + 2*(3) + (3*3))*8 + 4 + 1*64 # Size of SpicePlanetState:
                                                           # (1, 2*v3, M33) doubles[8]
                                                           # 1 int[4]
                                                           # 1 char[1] with maxLength = 64

    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputCelMessName,
                                          inputCelMessageSize,
                                          2)  
    CelBodyData = spice_interface.SpicePlanetState()
    planetPos = [-500.,-500., 0.]
    SimulationBaseClass.SetCArray(planetPos,
                                  'double',
                                  CelBodyData.PositionVector)
    planetVel = [0., 0., 0.]
    SimulationBaseClass.SetCArray(planetVel,
                                  'double',
                                  CelBodyData.VelocityVector)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputCelMessName,
                                          inputCelMessageSize,
                                          0,
                                          CelBodyData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(1.))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    #
    # check sigma_RN
    #
    moduleOutputName = "sigma_RN"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))
    # set the filtered output truth states
    trueVector = [
               [ -0.16367330847620223, -0.39514232112172260, -0.16367330847620221 ] ,
               [ -0.16367330847620223, -0.39514232112172260, -0.16367330847620221 ] ,
               [ -0.16367330847620223, -0.39514232112172260, -0.16367330847620221 ]
               ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) +
                                "sec\n")
    #
    # check omega_RN_N
    #
    moduleOutputName = "omega_RN_N"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))
    # set the filtered output truth states
    trueVector = [
               [ -0.00383553448372837, 0.00383553448372837, 0.00000000000000000 ] ,
               [ -0.00383553448372837, 0.00383553448372837, 0.00000000000000000 ] ,
               [ -0.00383553448372837, 0.00383553448372837, 0.00000000000000000 ]
               ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) +
                                "sec\n")
    #
    # check domega_RN_N
    #
    moduleOutputName = "domega_RN_N"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))
    # set the filtered output truth states
    trueVector = [
               [ 0.00001786539057109, -0.00001786539057109, 0.00000000000000000 ],
               [ 0.00001786539057109, -0.00001786539057109, 0.00000000000000000 ],
               [ 0.00001786539057109, -0.00001786539057109, 0.00000000000000000 ]
               ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) +
                                "sec\n")
    
    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
#    if show_plots:
#        # plot a sample variable.
#        plt.figure(1)
#        plt.plot(variableState[:,0]*unitTestSupport.NANO2SEC, variableState[:,1], label='Sample Variable')
#        plt.legend(loc='upper left')
#        plt.xlabel('Time [s]')
#        plt.ylabel('Variable Description [unit]')
#        plt.show()

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_velocityPoint(False)
