#
#   Unit Test Script
#   Module Name:        subModuleTemplate
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#
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



testFailCount = 0                       # zero unit test result counter
unitTaskName = "unitTask"               # arbitrary name (don't change)
unitProcessName = "TestProcess"         # arbitrary name (don't change)

#   Create a sim module as an empty container
unitTestSim = SimulationBaseClass.SimBaseClass()

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
unitTestSim.AddVariableForLogging(moduleWrap.ModelTag + ".dummy", testProcessRate)

#   Need to call the self-init and cross-init methods
unitTestSim.InitializeSimulation()

#   Step the simulation to 3*process rate so 4 total steps including zero
unitTestSim.ConfigureStopTime(unitTestSupport.sec2nano(1.1))        # seconds to stop simulation
unitTestSim.ExecuteSimulation()

#   This pulls the actual data log from the simulation run.
#   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
moduleOutputName = "outputVector"
moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                range(3))
variableName = "dummy"
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
        print "FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" \
            + str(moduleOutput[i,0]*unitTestSupport.NANO2SEC) + "sec"

    # check a scalar double value
    if not unitTestSupport.isDoubleEqual(dummyState[i],2.0,accuracy):
        testFailCount += 1
        print "FAILED: " + moduleWrap.ModelTag + " Module failed " + variableName + " unit test at t=" \
            + str(dummyState[i,0]*unitTestSupport.NANO2SEC) + "sec"




#   plot a sample variable
plt.figure(1)
plt.plot(dummyState[:,0]*unitTestSupport.NANO2SEC, dummyState[:,1], label='Sample Variable')
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

#   print out success message if no error were found
if testFailCount == 0:
    print   "Unit Test: " + moduleWrap.ModelTag + "() passed"

#   exit and return the number of errors found
sys.exit(testFailCount)
