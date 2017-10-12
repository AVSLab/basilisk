''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#Very simple simulation.  Just sets up and calls the SPICE interface.  Could 
#be the basis for a unit test of SPICE
import sys, os, inspect
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import logging
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('FswAlgorithms')



#Import all of the modules that we are going to call in this simulation
from Basilisk.utilities import MessagingAccess
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
from Basilisk.simulation import alg_contain
from Basilisk.fswAlgorithms import cssWlsEst


# Function that takes a sun pointing vector and array of CSS normal vectors and 
# returns the measurements associated with those normal vectors.
def createCosList(sunPointVec, sensorPointList):
   outList = []
   for sensorPoint in sensorPointList:
      outList.append(numpy.dot(sunPointVec, sensorPoint))
   return outList

# Method that checks that all of the numActive outputs from the data array 
# that are greater than threshold thresh are consistent with the values in 
# measVec
def checkNumActiveAccuracy(measVec, numActiveUse, numActiveFailCriteria, thresh):
    numActivePred = 0
    testFailCount = 0
    #Iterate through measVec and find all valid signals
    for i in range(0,32):
        obsVal = sim_model.doubleArray_getitem(measVec, i)
        if(obsVal > thresh):
            numActivePred += 1

    #Iterate through the numActive array and sum up all numActive estimates
    numActiveTotal = numpy.array([0])
    j=0
    while j < numActiveUse.shape[0]:
        numActiveTotal += numActiveUse[j, 1:]
        j += 1
    numActiveTotal /= j #Mean number of numActive
    #If we violate the test criteria, increment the failure count and alert user
    if(abs(numActiveTotal[0] - numActivePred) > numActiveFailCriteria):
        testFailCount += 1
        errorString = "Active number failure for count of: "
        errorString += str(numActivePred)
        logging.error(errorString)
    return testFailCount
    
#This method takes the sHat estimate output by the estimator and compares that 
#against the actual sun vector passed in as an argument.  If it doesn't match 
#to the specified tolerance, increment failure counter and alert the user
def checksHatAccuracy(testVec, sHatEstUse, angleFailCriteria, TotalSim) :
    j=0
    testFailCount = 0
    sHatTotal = numpy.array([0.0, 0.0, 0.0])
    #Sum up all of the sHat estimates from the execution
    while j < sHatEstUse.shape[0]:
        sHatTotal += sHatEstUse[j, 1:]
        j += 1
    sHatTotal /= j #mean sHat estimate
    #This logic is to protect cases where the dot product numerically breaks acos
    dot_value = numpy.dot(sHatTotal, testVec)
    if(abs(dot_value > 1.0)):
       dot_value -= 2.0*(dot_value -math.copysign(1.0, dot_value))

    #If we violate the failure criteria, increment failure count and alert user
    if (abs(math.acos(dot_value)) > angleFailCriteria):
        testFailCount += 1
        errorString = "Angle fail criteria violated for test vector:"
        errorString += str(testVec).strip('[]') + "\n"
        errorString += "Criteria violation of: "
        errorString += str(abs(math.acos(numpy.dot(sHatTotal, testVec))))
        logging.error(errorString)
    return testFailCount

TestResults = {}

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass() 
#Create test thread
testProc = TotalSim.CreateNewProcess("TestProcess")
testProc.addTask(TotalSim.CreateNewTask("wlsEstTestTask", int(1E8)))

#Construct algorithm and associated C++ container
CSSWlsEstFSWConfig = cssWlsEst.CSSWLSConfig()
CSSWlsWrap = alg_contain.AlgContain(CSSWlsEstFSWConfig,
   cssWlsEst.Update_cssWlsEst, cssWlsEst.SelfInit_cssWlsEst,
   cssWlsEst.CrossInit_cssWlsEst)
CSSWlsWrap.ModelTag = "CSSWlsEst"

#Add module to runtime call list
TotalSim.AddModelToTask("wlsEstTestTask", CSSWlsWrap, CSSWlsEstFSWConfig)

#Initialize the WLS estimator configuration data
CSSWlsEstFSWConfig.InputDataName = "css_data_aggregate"
CSSWlsEstFSWConfig.OutputDataName = "css_wls_est"
CSSWlsEstFSWConfig.UseWeights = False
CSSWlsEstFSWConfig.SensorUseThresh = 0.15

CSSConfigElement = cssWlsEst.SingleCSSConfig()
CSSConfigElement.CBias = 1.0
CSSConfigElement.cssNoiseStd = 0.2
CSSOrientationList = [
   [0.70710678118654746, -0.5, 0.5],
   [0.70710678118654746, -0.5, -0.5],
   [0.70710678118654746, 0.5, -0.5],
   [0.70710678118654746, 0.5, 0.5],
   [-0.70710678118654746, 0, 0.70710678118654757],
   [-0.70710678118654746, 0.70710678118654757, 0.0],
   [-0.70710678118654746, 0, -0.70710678118654757],
   [-0.70710678118654746, -0.70710678118654757, 0.0],
]
i=0
#Initializing a 2D double array is hard with SWIG.  That's why there is this 
#layer between the above list and the actual C variables.
for CSSHat in CSSOrientationList:
   # SimulationBaseClass.SetCArray(CSSHat, 'double', CSSConfigElement.nHatBdy)
   CSSConfigElement.nHatBdy = CSSHat
   # cssWlsEst.CSSWlsConfigArray_setitem( CSSWlsEstFSWConfig.CSSData, i,
   #    CSSConfigElement)
   CSSWlsEstFSWConfig.CSSData[i] = CSSConfigElement
   i += 1

#Create input message and size it because the regular creator of that message 
#is not part of the test.
TotalSim.TotalSim.CreateNewMessage("TestProcess", CSSWlsEstFSWConfig.InputDataName, 8*8, 2)

#Initialize input data for above message
cssDataMsg = sim_model.new_doubleArray(32)
i=0
while(i<32):
   sim_model.doubleArray_setitem(cssDataMsg, i, 0.0)   
   i += 1

angleFailCriteria = 17.5*math.pi/180.0 #Get 95% effective charging in this case
numActiveFailCriteria = 0.000001 #basically zero

#Log the output message as well as the internal numACtiveCss variables
TotalSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
TotalSim.AddVariableForLogging("CSSWlsEst.numActiveCss", int(1E8))

#Initia test is all of the principal body axes
TestVectors = [[-1.0, 0.0, 0.0],
               [0.0, -1.0, 0.0],
               [1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [0.0, 0.0, -1.0],
               [0.0, 0.0, 1.0]]

#Initialize test and then step through all of the test vectors in a loop
TotalSim.InitializeSimulation()
stepCount = 0
logLengthPrev = 0
testFailCount = 0
truthData = []
for testVec in TestVectors:
    if(stepCount > 1): #Doing this to test permutations and get code coverage
        CSSWlsEstFSWConfig.UseWeights = True
    nextRows = []
    #Get observation data based on sun pointing and CSS orientation data
    cssDataList = createCosList(testVec, CSSOrientationList)
    i=0
    #Updating C-arrays is handled like this.  Kind of clunky, but so is C.
    while(i < len(cssDataList)):
       sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
       i += 1
    #Write in the observation data to the input message
    TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
       cssDataMsg)
    #Increment the stop time to new termination value
    TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
    #Execute simulation to current stop time
    TotalSim.ExecuteSimulation()
    stepCount += 1
    #Pull logged data out into workspace for analysis
    sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
       'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
    numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
    sHatEstUse = sHatEst[logLengthPrev:, :] #Only data for this subtest
    numActiveUse = numActive[logLengthPrev+1:, :] #Only data for this subtest

    #Check failure criteria and add test failures
    testFailCount += checksHatAccuracy(testVec, sHatEstUse, angleFailCriteria, 
       TotalSim)
    testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
       numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
    #Pop truth state onto end of array for plotting purposes
    currentRow = [sHatEstUse[0, 0]]
    currentRow.extend(testVec)
    truthData.append(currentRow)
    currentRow = [sHatEstUse[-1, 0]]
    currentRow.extend(testVec)
    truthData.append(currentRow)
    logLengthPrev = sHatEst.shape[0]

#Hand construct case where we get low coverage (2 valid sensors)
LonVal = 0.0
LatVal = 40.68*math.pi/180.0
doubleTestVec = [math.sin(LatVal), math.cos(LatVal)*math.sin(LonVal), 
   math.cos(LatVal)*math.cos(LonVal)]
cssDataList = createCosList(doubleTestVec, CSSOrientationList)
i=0
while(i < len(cssDataList)):
   sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
   i += 1

#Write in double coverage conditions and ensure that we get correct outputs
TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
   cssDataMsg);
TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
TotalSim.ExecuteSimulation()
stepCount += 1
sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
sHatEstUse = sHatEst[logLengthPrev:, :]
numActiveUse = numActive[logLengthPrev+1:, :]
logLengthPrev = sHatEst.shape[0]
currentRow = [sHatEstUse[0, 0]]
currentRow.extend(doubleTestVec)
truthData.append(currentRow)
currentRow = [sHatEstUse[-1, 0]]
currentRow.extend(doubleTestVec)
truthData.append(currentRow)

#Check test criteria again
testFailCount += checksHatAccuracy(doubleTestVec, sHatEstUse, angleFailCriteria,
    TotalSim)
testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)

#Same test as above, but zero first element to get to a single coverage case
sim_model.doubleArray_setitem(cssDataMsg, 0, 0.0)
TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
   cssDataMsg)
TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
TotalSim.ExecuteSimulation()
stepCount += 1
numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
numActiveUse = numActive[logLengthPrev+1:, :]
sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
sHatEstUse = sHatEst[logLengthPrev+1:, :]
logLengthPrev = sHatEst.shape[0]
testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
currentRow = [sHatEstUse[0, 0]]
currentRow.extend(doubleTestVec)
truthData.append(currentRow)
currentRow = [sHatEstUse[-1, 0]]
currentRow.extend(doubleTestVec)
truthData.append(currentRow)

#Same test as above, but zero first and fourth elements to get to zero coverage
sim_model.doubleArray_setitem(cssDataMsg, 0, 0.0)
sim_model.doubleArray_setitem(cssDataMsg, 3, 0.0)
TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
   cssDataMsg);
TotalSim.ConfigureStopTime(int((stepCount+1)*1E9))
TotalSim.ExecuteSimulation()
numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")
numActiveUse = numActive[logLengthPrev:, :]
logLengthPrev = sHatEst.shape[0]
testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)

#Format data for plotting
truthData = numpy.array(truthData)
sHatEst = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
   'CSSWlsEstOut', int(stepCount*10), TotalSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
numActive = TotalSim.GetLogVariableData("CSSWlsEst.numActiveCss")

plt.figure(1)
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,1], label='x-Sun')
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,2], label='y-Sun' )
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,3], label='z-Sun' )
plt.legend(loc='upper left')
plt.xlabel('Time (s)')
plt.ylabel('Unit Component (--)')

plt.figure(2)
plt.plot(numActive[:,0]*1.0E-9, numActive[:,1])
plt.xlabel('Time (s)')
plt.ylabel('Number Active CSS (--)')

plt.figure(3)
plt.subplot(3, 1, 1)
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,1], label='Est')
plt.plot(truthData[:,0]*1.0E-9, truthData[:,1], 'r--', label='Truth')
plt.xlabel('Time (s)')
plt.ylabel('X Component (--)')
plt.legend(loc='best')
plt.subplot(3, 1, 2)
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,2], label='Est')
plt.plot(truthData[:,0]*1.0E-9, truthData[:,2], 'r--', label='Truth')
plt.xlabel('Time (s)')
plt.ylabel('Y Component (--)')
plt.subplot(3, 1, 3)
plt.plot(sHatEst[:,0]*1.0E-9, sHatEst[:,3], label='Est')
plt.plot(truthData[:,0]*1.0E-9, truthData[:,3], 'r--', label='Truth')
plt.xlabel('Time (s)')
plt.ylabel('Z Component (--)')

inputArgs = sys.argv
if len(inputArgs) > 1:
   if inputArgs[1] == 'True':
      plt.show()

sys.exit(testFailCount)
