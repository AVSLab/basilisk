import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import EMMSim
import matplotlib.pyplot as plt
import ctypes
import math
import MessagingAccess
import sim_model
import numpy
import logging

def checkCSSEstAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh):
   i=0
   j=0
   accuracyFailCounter = 0
   while(i<FSWsHat.shape[0]):
      while(DataCSSTruth[j, 0] < FSWsHat[i,0]):
         j += 1
      timePrevDelta = DataCSSTruth[j-1, 0] - FSWsHat[i,0]
      timeNowDelta = DataCSSTruth[j, 0] - FSWsHat[i,0]
      indexUse = j-1 if abs(timePrevDelta) < abs(timeNowDelta) else j
      dot_val = numpy.dot(DataCSSTruth[indexUse, 1:], FSWsHat[i,1:])
      if(abs(dot_val) > 1.0):
         dot_val = 1.0
      angCheck = math.acos(dot_val)
      if(angCheck > CSSEstAccuracyThresh):
         errorString = "CSS accuracy failure for value of: "
         errorString += str(angCheck*180.0/math.pi)
         logging.error(errorString)
         accuracyFailCounter += 1
      i += 1
   return accuracyFailCounter

def checkSlewAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh, 
   slewFinishTime, desiredSunBdy):

   dtTruth = (DataCSSTruth[10, 0] - DataCSSTruth[9, 0])*1.0E-9
   counterStart = int(slewFinishTime/dtTruth)
   controlFailCounter = 0
   while(counterStart < DataCSSTruth.shape[0]):
      dot_val = numpy.dot(DataCSSTruth[counterStart, 1:], desiredSunBdy)
      if(dot_val > 1.0):
         dot_val = 1.0
      if(math.acos(dot_val) > CSSEstAccuracyThresh):
         controlFailCounter += 1
         errorString = "Safe mode control failure for value of: "
         errorString += str(math.acos(dot_val)*180.0/math.pi)
         logging.error(errorString)
      counterStart += 1
   return controlFailCounter

TheEMMSim = EMMSim.EMMSim()
TheEMMSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E8))
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E8))
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
TheEMMSim.TotalSim.logThisMessage("solar_array_sun_bore", int(1E9)) #solar array boresight angles
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E8))
TheEMMSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E8))


TheEMMSim.VehOrbElemObject.CurrentElem.a = 188767262.18*1000.0;
TheEMMSim.VehOrbElemObject.CurrentElem.e = 0.207501;
TheEMMSim.VehOrbElemObject.CurrentElem.i = 0.0;
TheEMMSim.VehOrbElemObject.CurrentElem.Omega = 0.0;
TheEMMSim.VehOrbElemObject.CurrentElem.omega = 0.0;
TheEMMSim.VehOrbElemObject.CurrentElem.f = 70.0*math.pi/180.0
#Convert those OEs to cartesian
TheEMMSim.VehOrbElemObject.Elements2Cartesian()
PosVec = ctypes.cast(TheEMMSim.VehOrbElemObject.r_N.__long__(), 
   ctypes.POINTER(ctypes.c_double))
VelVec = ctypes.cast(TheEMMSim.VehOrbElemObject.v_N.__long__(), 
  ctypes.POINTER(ctypes.c_double))
TheEMMSim.VehDynObject.PositionInit = sim_model.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
TheEMMSim.VehDynObject.VelocityInit = sim_model.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])

TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(30*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.modeRequest = 'safeMode'
TheEMMSim.ConfigureStopTime(int(60*120*1E9))
TheEMMSim.ExecuteSimulation()

FSWsHat = TheEMMSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
numCSSActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')
solarArrayMiss = TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")

CSSEstAccuracyThresh = 17.5*math.pi/180.0
accuracyFailCounter = checkCSSEstAccuracy(DataCSSTruth, FSWsHat, 
   CSSEstAccuracyThresh)

slewFinishTime = 150.0
desiredSunBdy = [0.0, 0.0, 1.0]
controlFailCounter =  checkSlewAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh,
   slewFinishTime, desiredSunBdy)

plt.figure(1)
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'g', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'r', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')

plt.figure(2)
plt.plot(numCSSActive[:,0]*1.0E-9, numCSSActive[:,1])

plt.figure(3)
plt.plot(solarArrayMiss[:,0]*1.0E-9, solarArrayMiss[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('Solar Array Miss (d)')

if(len(sys.argv) > 1):
   if(sys.argv[1] == 'True'):
      plt.show()

#sys.exit(accuracyFailCounter + controlFailCounter)
