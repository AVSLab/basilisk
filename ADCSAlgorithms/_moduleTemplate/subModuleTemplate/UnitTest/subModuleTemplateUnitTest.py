#
#   Unit Test Script
#   Module Name:        subModuleTemplate
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import unittest
import sys, os, inspect
import matplotlib.pyplot as plt
import numpy
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
import subModuleTemplate                # import the module that is to be tested
import MRP_Steering                     # import module(s) that creates the needed input message declaration


#   Given this function a unique name, typically based on the module name
class subModuleTemplateTest(unittest.TestCase):

    #def setUp(self):
        # add any code there that needs to be run once before the testing routine(s)

    #def tearDown(self):
        # this is needed if multiple unit test scripts are run
        # this create a fresh and consistent simulation environment for each test run
        # unitTestSim.TotalSim.terminateSimulation()

    # uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
    # @unittest.skip("test_subModuleTemplate skipping")
    # uncomment this line if this test has an expected failure, adjust message as needed
    # @unittest.expectedFailure("message")
    def test_subModuleTemplate(self):       # provide a unique test method name, starting with test_
        testFailCount = 0                       # zero unit test result counter
        testResults = ""                        # create empty array to store test log messages
        unitTaskName = "unitTask"               # arbitrary name (don't change)
        unitProcessName = "TestProcess"         # arbitrary name (don't change)

        #   Create a sim module as an empty container
        unitTestSim = SimulationBaseClass.SimBaseClass()
        unitTestSim.TotalSim.terminateSimulation()          # this is needed if multiple unit test scripts are run
                                                            # this creates a fresh and consistent simulation environment for each test run

        #   Create test thread
        testProcessRate = unitTestSupport.sec2nano(0.5)     # update process rate update time
        testProc = unitTestSim.CreateNewProcess(unitProcessName)
        testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


        #   Construct algorithm and associated C++ container
        moduleConfig = subModuleTemplate.subModuleTemplateConfig()                          # update with current values
        moduleWrap = alg_contain.AlgContain(moduleConfig,
                                            subModuleTemplate.Update_subModuleTemplate,     # update with current values
                                            subModuleTemplate.SelfInit_subModuleTemplate,   # update with current values
                                            subModuleTemplate.CrossInit_subModuleTemplate)  # update with current values
        moduleWrap.ModelTag = "subModuleTemplate"           # update python name of test module

        #   Add test module to runtime call list
        unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

        #   Initialize the test module configuration data
        moduleConfig.inputDataName  = "sampleInput"         # update with current values
        moduleConfig.outputDataName = "sampleOutput"        # update with current values
        moduleConfig.dummy = 1                              # update module parameter with required values
        vector = [1., 2., 3.]
        SimulationBaseClass.SetCArray(vector,
                                      'double',
                                      moduleConfig.dumVector)


        #   Create input message and size it because the regular creator of that message
        #   is not part of the test.
        inputMessageSize = 3*8                              # 3 doubles
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                              moduleConfig.inputDataName,
                                              inputMessageSize,
                                              2)            # number of buffers (leave at 2 as default, don't make zero)

        inputMessageData = MRP_Steering.vehControlOut()     # Create a structure for the input message
        sampleInputMessageVariable = [1.0, -0.5, 0.7]       # Set up a list as a 3-vector
        SimulationBaseClass.SetCArray(sampleInputMessageVariable,           # specify message variable
                                      'double',                             # specify message variable type
                                      inputMessageData.torqueRequestBody)   # write torque request to input message
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputDataName,
                                              inputMessageSize,
                                              0,
                                              inputMessageData)             # write data into the simulator

        #   Setup logging on the test module output message so that we get all the writes to it
        unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)
        variableName = "dummy"                              # name the module variable to be logged
        unitTestSim.AddVariableForLogging(moduleWrap.ModelTag + "." + variableName, testProcessRate)

        #   Need to call the self-init and cross-init methods
        unitTestSim.InitializeSimulation()

        #   Set the simulation time.
        #   NOTE: the total simulation time may be longer than this value.  The simulation is stopped
        #         at the next logging event on or after the simulation end time
        unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(1.))        # seconds to stop simulation

        #   Begin the simulation time run set above
        unitTestSim.ExecuteSimulation()

        #   This pulls the actual data log from the simulation run.
        #   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
        moduleOutputName = "outputVector"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                        range(3))
        variableState = unitTestSim.GetLogVariableData(moduleWrap.ModelTag + "." + variableName)


        #   set the filtered output truth states
        trueVector = [
                   [1.0, -0.5, 0.7],
                   [1.0, -0.5, 0.7],
                   [1.0, -0.5, 0.7]
                   ]

        #   compare the module results to the truth values
        accuracy = 1e-12
        for i in range(0,len(trueVector)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
                testFailCount += 1
                testMessage = "FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) + "sec"
                print testMessage
                testResults += testMessage + "\n"

            # check a scalar double value
            if not unitTestSupport.isDoubleEqual(variableState[i],2.0,accuracy):
                testFailCount += 1
                testMessage = "FAILED: " + moduleWrap.ModelTag + " Module failed " + variableName + " unit test at t=" \
                    + str(variableState[i,0]*unitTestSupport.NANO2SEC) + "sec"
                print testMessage
                testResults += testMessage + "\n"



        #   plot a sample variable.
        #   NOTE: the plots are only shown if the argument -plot is provided when the script is called
        plt.figure(1)
        plt.plot(variableState[:,0]*unitTestSupport.NANO2SEC, variableState[:,1], label='Sample Variable')
        plt.legend(loc='upper left')
        plt.xlabel('Time [s]')
        plt.ylabel('Variable Description [unit]')



        #   Note that we can continue to step the simulation however we feel like.
        #   Just because we stop and query data does not mean everything has to stop for good
        unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(0.6))    # run an additional 0.6 seconds
        unitTestSim.ExecuteSimulation()


        #   If the argument "-plot" is passed along, plot all figures
        inputArgs = sys.argv
        if len(inputArgs) > 1:
           if inputArgs[1] == '-plot':
              plt.show()

        # each test method requires a single assert method to be called
        # this check below just makes sure no sub-test failures were found
        self.assertLess(testFailCount, 1, testResults)





    # it is possible to have multiple tests being called.  Here is some boiler plate code to
    # setup another test message
    # To create a test use the naming nomenclature "test_your_method"
    #def test_example_method(self):
        # each test resolves a single boolean question
        # this is done by using one of the many 'assert' methods
        # available.
        # self.assertEqual(0, 0, 'Place holder assesrtion failure message')





#
#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   authmatically executes the runUnitTest() method
#
if __name__ == "__main__":
    unittest.main()
