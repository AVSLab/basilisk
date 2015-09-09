#Very simple simulation.  Just sets up and calls the SPICE interface.  Could 
#be the basis for a unit test of SPICE
import sys, os
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import logging
sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')

#Import all of the modules that we are going to call in this simulation
import cssWlsEst
import MessagingAccess
import SimulationBaseClass
import sim_model
import alg_contain


def createCosList(sunPointVec, sensorPointList):
   outList = []
   for sensorPoint in sensorPointList:
      outList.append(numpy.dot(sunPointVec, sensorPoint))
   return outList

def checkNumActiveAccuracy(measVec, numActiveUse, numActiveFailCriteria, thresh):
    numActivePred = 0
    testFailCount = 0
    for i in range(0,32):
        obsVal = sim_model.doubleArray_getitem(measVec, i)
        if(obsVal > thresh):
            numActivePred += 1

    numActiveTotal = numpy.array([0])
    j=0
    while j < numActiveUse.shape[0]:
        numActiveTotal += numActiveUse[j, 1:]
        j += 1

    numActiveTotal /= j
    if(abs(numActiveTotal[0] - numActivePred) > numActiveFailCriteria):
        testFailCount += 1
        errorString = "Active number failure for count of: "
        errorString += str(numActivePred)
        logging.error(errorString)
    return testFailCount
    

def checksHatAccuracy(testVec, sHatEstUse, angleFailCriteria, TotalSim) :
    j=0
    testFailCount = 0
    sHatTotal = numpy.array([0.0, 0.0, 0.0])
    while j < sHatEstUse.shape[0]:
        sHatTotal += sHatEstUse[j, 1:]
        j += 1

    sHatTotal /= j
    dot_value = numpy.dot(sHatTotal, testVec)
    if(abs(dot_value > 1.0)):
       dot_value -= 2.0*(dot_value -math.copysign(1.0, dot_value))
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
TotalSim.CreateNewThread("wlsEstTestThread", int(1E8))

CSSWlsEstFSWConfig = cssWlsEst.CSSWLSConfig()
CSSWlsWrap = alg_contain.AlgContain(CSSWlsEstFSWConfig,
   cssWlsEst.Update_cssWlsEst, cssWlsEst.SelfInit_cssWlsEst,
   cssWlsEst.CrossInit_cssWlsEst)
CSSWlsWrap.ModelTag = "CSSWlsEst"

TotalSim.AddModelToThread("wlsEstTestThread", CSSWlsWrap, CSSWlsEstFSWConfig)

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
for CSSHat in CSSOrientationList:
   SimulationBaseClass.SetCArray(CSSHat, 'double', CSSConfigElement.nHatBdy)
   cssWlsEst.CSSWlsConfigArray_setitem( CSSWlsEstFSWConfig.CSSData, i,
      CSSConfigElement)
   i += 1

TotalSim.TotalSim.CreateNewMessage(CSSWlsEstFSWConfig.InputDataName, 8*32, 2)
cssDataMsg = sim_model.new_doubleArray(32)
i=0
while(i<32):
   sim_model.doubleArray_setitem(cssDataMsg, i, 0.0)   
   i += 1

angleFailCriteria = 17.5*math.pi/180.0 #Get 95% effective charging in this case
numActiveFailCriteria = 0.000001 #basically zero

TotalSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
TotalSim.AddVariableForLogging("CSSWlsEst.numActiveCss", int(1E8))

testLength = 1.0
TestVectors = [[-1.0, 0.0, 0.0],
               [0.0, -1.0, 0.0],
               [1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [0.0, 0.0, -1.0],
               [0.0, 0.0, 1.0]]

TotalSim.InitializeSimulation()
stepCount = 0
logLengthPrev = 0
testFailCount = 0
truthData = []
for testVec in TestVectors:
    if(stepCount > 1):
        CSSWlsEstFSWConfig.UseWeights = True
    nextRows = []
    cssDataList = createCosList(testVec, CSSOrientationList)
    i=0
    while(i < len(cssDataList)):
       sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
       i += 1
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

    testFailCount += checksHatAccuracy(testVec, sHatEstUse, angleFailCriteria, 
       TotalSim)
    testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
       numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)
    currentRow = [sHatEstUse[0, 0]]
    currentRow.extend(testVec)
    truthData.append(currentRow)
    currentRow = [sHatEstUse[-1, 0]]
    currentRow.extend(testVec)
    truthData.append(currentRow)
    logLengthPrev = sHatEst.shape[0]

LonVal = 0.0
LatVal = 40.68*math.pi/180.0
doubleTestVec = [math.sin(LatVal), math.cos(LatVal)*math.sin(LonVal), 
   math.cos(LatVal)*math.cos(LonVal)]
cssDataList = createCosList(doubleTestVec, CSSOrientationList)
i=0
while(i < len(cssDataList)):
   sim_model.doubleArray_setitem(cssDataMsg, i, cssDataList[i])
   i += 1


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

testFailCount += checksHatAccuracy(doubleTestVec, sHatEstUse, angleFailCriteria,
    TotalSim)
testFailCount += checkNumActiveAccuracy(cssDataMsg, numActiveUse, 
   numActiveFailCriteria, CSSWlsEstFSWConfig.SensorUseThresh)

sim_model.doubleArray_setitem(cssDataMsg, 0, 0.0)
TotalSim.TotalSim.WriteMessageData(CSSWlsEstFSWConfig.InputDataName, 8*8, 0,
   cssDataMsg);
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

plt.show()
