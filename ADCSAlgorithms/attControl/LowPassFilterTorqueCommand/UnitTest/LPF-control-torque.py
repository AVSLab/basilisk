#
#   Unit Test Script
#   Module Name:        LowPassFilterTorqueCommand
#   Author:             Hanspeter Schaub
#   Creation Date:      December 9, 2015
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

def sec2nano(time):                     # function to convert seconds to an integer nanoseconds value
    return int(time*1E9)


#   Import all of the modules that we are going to call in this simulation
import MessagingAccess
import SimulationBaseClass
import sim_model
import alg_contain
import LowPassFilterTorqueCommand       # import the module that is to be tested
import MRP_Steering                     # import a sample module that creates the neede input message declaration

#   zero all unit test result gather variables
TestResults = {}
testFailCount = 0
unitTaskName = "unitTask"               # arbitrary name (don't change)
unitProcessName = "TestProcess"         # arbitrary name (don't change)

#   Create a sim module as an empty container
unitTestSim = SimulationBaseClass.SimBaseClass()

#   Create test thread
testProcessRate = sec2nano(0.1)         # 10 Hz process rate, python time is a nano-seconds integer value
testProc = unitTestSim.CreateNewProcess(unitProcessName)
testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


#   Construct algorithm and associated C++ container
moduleConfig = LowPassFilterTorqueCommand.lowPassFilterTorqueCommandConfig()
moduleWrap = alg_contain.AlgContain(moduleConfig,
                                    LowPassFilterTorqueCommand.Update_LowPassFilterTorqueCommand,
                                    LowPassFilterTorqueCommand.SelfInit_LowPassFilterTorqueCommand,
                                    LowPassFilterTorqueCommand.CrossInit_LowPassFilterTorqueCommand)
moduleWrap.ModelTag = "testModule"

#   Add test module to runtime call list
unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

#   Initialize the test module configuration data
moduleConfig.InputDataName  = "controlTorqueRaw"
moduleConfig.OutputDataName = "controlTorqueFiltered"
moduleConfig.h = 0.5            #   [s]     control time step
moduleConfig.wc = 0.10          #   [rad/s] continous time critical filter frequency
moduleConfig.priorTime = 0      #           indidate filter is called for the first time


#   Create input message and size it because the regular creator of that message
#   is not part of the test.
inputMessageSize = 3*8          # 3 doubles
unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                      moduleConfig.InputDataName,
                                      inputMessageSize,
                                      2)        # number of buffers (leave at 2 as default, don't make zero)

inputMessageData = MRP_Steering.vehControlOut() #Create a structure for the input message
torqueRequest = [1.0, 0.0, 0.0]                 # Set up a list as a 3-vector
SimulationBaseClass.SetCArray(torqueRequest,                        # specify message variable
                              'double',                             # specify message variable type
                              inputMessageData.torqueRequestBody)   # Write torque request to input message
unitTestSim.TotalSim.WriteMessageData(moduleConfig.InputDataName,
                                      inputMessageSize,
                                      0,
                                      inputMessageData) #write data into the simulator

#   Setup logging on the test module output message so that we get all the writes to it
unitTestSim.TotalSim.logThisMessage(moduleConfig.OutputDataName, testProcessRate)

#   Need to call the self-init and cross-init methods
unitTestSim.InitializeSimulation()

#   Step the simulation to 3*process rate so 4 total steps including zero
unitTestSim.ConfigureStopTime(sec2nano(0.3))    # 0.3 second stop time
unitTestSim.ExecuteSimulation()

#   This pulls the actual data log from the simulation run.
#   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
filteredTorque = unitTestSim.pullMessageLogData(moduleConfig.OutputDataName + '.torqueRequestBody',
                                                range(3))
print filteredTorque

#   Note that we can continue to step the simulation however we feel like.
#   Just because we stop and query data does not mean everything has to stop for good
unitTestSim.ConfigureStopTime(sec2nano(0.6))    # run an additional 0.6 seconds
unitTestSim.ExecuteSimulation()

##Initialize input data for above message
#cssDataMsg = sim_model.new_doubleArray(32)
#i=0
#while(i<32):
#   sim_model.doubleArray_setitem(cssDataMsg, i, 0.0)   
#   i += 1
#
#angleFailCriteria = 17.5*math.pi/180.0 #Get 95% effective charging in this case
#numActiveFailCriteria = 0.000001 #basically zero
#
##Log the output message as well as the internal numACtiveCss variables
#unitTestSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
#unitTestSim.AddVariableForLogging("CSSWlsEst.numActiveCss", int(1E8))

#Initia test is all of the principal body axes
#TestVectors = [[-1.0, 0.0, 0.0],
#               [0.0, -1.0, 0.0],
#               [1.0, 0.0, 0.0],
#               [0.0, 1.0, 0.0],
#               [0.0, 0.0, -1.0],
#               [0.0, 0.0, 1.0]]

#   Initialize test and then step through all of the test vectors in a loop
#TotalSim.InitializeSimulation()
#stepCount = 0
#logLengthPrev = 0
#testFailCount = 0
#truthData = []
#for testVec in TestVectors:
#    if(stepCount > 1): #Doing this to test permutations and get code coverage
#        CSSWlsEstFSWConfig.UseWeights = True
#    nextRows = []
#    #Get observation data based on sun pointing and CSS orientation data
#    cssDataList = createCosList(testVec, CSSOrientationList)
#    i=0
#    #Updating C-arrays is handled like this.  Kind of clunky, but so is C.
#    while(i < len(cssDataList)):
#       sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
#       i += 1
#    #Write in the observation data to the input message
#    TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
#       cssDataMsg);
#    #Increment the stop time to new termination value
#    TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
#    #Execute simulation to current stop time
#    TotalSim.ExecuteSimulation()
#    stepCount += 1
#    #Pull logged data out into workspace for analysis
#    sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
#       'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
#    numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
#    sHatEstUse = sHatEst[logLengthPrev:, :] #Only data for this subtest
#    numActiveUse = numActive[logLengthPrev+1:, :] #Only data for this subtest
#
#    #Check failure criteria and add test failures
#    testFailCount += checksHatAccuracy(testVec, sHatEstUse, angleFailCriteria, 
#       TotalSim)
#    testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
#       numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
#    #Pop truth state onto end of array for plotting purposes
#    currentRow = [sHatEstUse[0, 0]]
#    currentRow.extend(testVec)
#    truthData.append(currentRow)
#    currentRow = [sHatEstUse[-1, 0]]
#    currentRow.extend(testVec)
#    truthData.append(currentRow)
#    logLengthPrev = sHatEst.shape[0]
#
##Hand construct case where we get low coverage (2 valid sensors)
#LonVal = 0.0
#LatVal = 40.68*math.pi/180.0
#doubleTestVec = [math.sin(LatVal), math.cos(LatVal)*math.sin(LonVal), 
#   math.cos(LatVal)*math.cos(LonVal)]
#cssDataList = createCosList(doubleTestVec, CSSOrientationList)
#i=0
#while(i < len(cssDataList)):
#   sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
#   i += 1
#
##Write in double coverage conditions and ensure that we get correct outputs
#TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
#   cssDataMsg);
#TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
#TotalSim.ExecuteSimulation()
#stepCount += 1
#sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
#   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
#numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
#sHatEstUse = sHatEst[logLengthPrev:, :]
#numActiveUse = numActive[logLengthPrev+1:, :]
#logLengthPrev = sHatEst.shape[0]
#currentRow = [sHatEstUse[0, 0]]
#currentRow.extend(doubleTestVec)
#truthData.append(currentRow)
#currentRow = [sHatEstUse[-1, 0]]
#currentRow.extend(doubleTestVec)
#truthData.append(currentRow)
#
##Check test criteria again
#testFailCount += checksHatAccuracy(doubleTestVec, sHatEstUse, angleFailCriteria,
#    TotalSim)
#testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
#   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
#
##Same test as above, but zero first element to get to a single coverage case
#sim_model.doubleArray_setitem(cssDataMsg, 0, 0.0)
#TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
#   cssDataMsg);
#TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
#TotalSim.ExecuteSimulation()
#stepCount += 1
#numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
#numActiveUse = numActive[logLengthPrev+1:, :]
#sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
#   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
#sHatEstUse = sHatEst[logLengthPrev+1:, :]
#logLengthPrev = sHatEst.shape[0]
#testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
#   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
#currentRow = [sHatEstUse[0, 0]]
#currentRow.extend(doubleTestVec)
#truthData.append(currentRow)
#currentRow = [sHatEstUse[-1, 0]]
#currentRow.extend(doubleTestVec)
#truthData.append(currentRow)
#
##Same test as above, but zero first and fourth elements to get to zero coverage
#sim_model.doubleArray_setitem(cssDataMsg, 0, 0.0)
#sim_model.doubleArray_setitem(cssDataMsg, 3, 0.0)
#TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
#   cssDataMsg);
#TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
#TotalSim.ExecuteSimulation()
#numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
#numActiveUse = numActive[logLengthPrev:, :]
#logLengthPrev = sHatEst.shape[0]
#testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
#   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
#
##Format data for plotting
#truthData = numpy.array(truthData)
#sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
#   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
#numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
#
#plt.figure(1)
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,1], label='x-Sun')
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,2], label='y-Sun' )
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,3], label='z-Sun' )
#plt.legend(loc='upper left')
#plt.xlabel('Time (s)')
#plt.ylabel('Unit Component (--)')
#
#plt.figure(2)
#plt.plot(numActive[:,0]*1.0E-9, numActive[:,1])
#plt.xlabel('Time (s)')
#plt.ylabel('Number Active CSS (--)')
#
#plt.figure(3)
#plt.subplot(3, 1, 1)
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,1], label='Est')
#plt.plot(truthData[:,0]*1.0E-9, truthData[:,1], 'r--', label='Truth')
#plt.xlabel('Time (s)')
#plt.ylabel('X Component (--)')
#plt.legend(loc='best')
#plt.subplot(3, 1, 2)
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,2], label='Est')
#plt.plot(truthData[:,0]*1.0E-9, truthData[:,2], 'r--', label='Truth')
#plt.xlabel('Time (s)')
#plt.ylabel('Y Component (--)')
#plt.subplot(3, 1, 3)
#plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,3], label='Est')
#plt.plot(truthData[:,0]*1.0E-9, truthData[:,3], 'r--', label='Truth')
#plt.xlabel('Time (s)')
#plt.ylabel('Z Component (--)')
#

inputArgs = sys.argv
if len(inputArgs) > 1:
   if inputArgs[1] == 'True':
      plt.show()

sys.exit(testFailCount)
