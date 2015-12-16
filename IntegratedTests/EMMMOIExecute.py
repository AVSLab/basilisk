import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import matplotlib
matplotlib.use('TkAgg')
import EMMSim
import matplotlib.pyplot as plt
import ctypes
import math
import MessagingAccess
import sim_model
import numpy
import logging
import attMnvrPoint
import SimulationBaseClass

TheEMMSim = EMMSim.EMMSim()

TheEMMSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E9))
TheEMMSim.TotalSim.logThisMessage("nom_att_guid_out", int(1E9))
TheEMMSim.TotalSim.logThisMessage("dv_thruster_cmds", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E10))
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E10))
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E10))
TheEMMSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E10))
TheEMMSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10))
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E10))
TheEMMSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E10))
TheEMMSim.AddVectorForLogging('attMnvrPoint.sigmaCmd_BR', 'double', 0, 2, int(1E10))
TheEMMSim.AddVectorForLogging('attMnvrPoint.omegaCmd_BR_B', 'double', 0, 2, int(1E10))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E10))
TheEMMSim.AddVariableForLogging('dvGuidance.burnExecuting', int(1E9))
TheEMMSim.AddVariableForLogging('dvGuidance.burnComplete', int(1E9))


TheEMMSim.VehDynObject.GravData[0].IsCentralBody = False
TheEMMSim.VehDynObject.GravData[0].IsDisplayBody = False
TheEMMSim.VehDynObject.GravData[2].IsCentralBody = True
TheEMMSim.VehDynObject.GravData[2].IsDisplayBody = True
TheEMMSim.VehOrbElemObject.mu = TheEMMSim.MarsGravBody.mu

TheEMMSim.VehDynObject.baseCoMInit[0] = 0.05
TheEMMSim.VehOrbElemObject.CurrentElem.a = -5253.512142852398*1000.0;
TheEMMSim.VehOrbElemObject.CurrentElem.e = 1.737401863285942;
TheEMMSim.VehOrbElemObject.CurrentElem.i = 0.07676135277528215;
TheEMMSim.VehOrbElemObject.CurrentElem.Omega = 1.966373184970122;
TheEMMSim.VehOrbElemObject.CurrentElem.omega = 2.990805413783388;
TheEMMSim.VehOrbElemObject.CurrentElem.f = -1.980693284448144
#Convert those OEs to cartesian
TheEMMSim.VehOrbElemObject.Elements2Cartesian()
PosVec = ctypes.cast(TheEMMSim.VehOrbElemObject.r_N.__long__(), 
   ctypes.POINTER(ctypes.c_double))
VelVec = ctypes.cast(TheEMMSim.VehOrbElemObject.v_N.__long__(), 
  ctypes.POINTER(ctypes.c_double))
TheEMMSim.VehDynObject.PositionInit = sim_model.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
TheEMMSim.VehDynObject.VelocityInit = sim_model.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])
TheEMMSim.VehOrbElemObject.Cartesian2Elements()

fLocal = TheEMMSim.VehOrbElemObject.CurrentElem.f
aLocal = TheEMMSim.VehOrbElemObject.CurrentElem.a
eLocal = TheEMMSim.VehOrbElemObject.CurrentElem.e
hLocal = 2.0 * math.atanh(math.sqrt((eLocal - 1.0) / (eLocal + 1.0)) * math.tan(fLocal / 2.0));
tmT = math.sqrt(-aLocal*aLocal*aLocal/TheEMMSim.VehOrbElemObject.mu)
tmT *= (TheEMMSim.VehOrbElemObject.CurrentElem.e*math.sinh(hLocal) - hLocal)
tmT *= -1.0
totalBurnTime = 37.25*60
TheEMMSim.VehOrbElemObject.CurrentElem.f = 0.0
TheEMMSim.VehOrbElemObject.Elements2Cartesian()
posArray = numpy.array([PosVec[0], PosVec[1], PosVec[2]])
velArray = numpy.array([VelVec[0], VelVec[1], VelVec[2]])
hVec = numpy.cross(posArray, velArray)
hVec = hVec/numpy.linalg.norm(hVec)

vStartBurn = numpy.array([4341.2071605098472,-920.87582716674796,-280.6012005272342])
#dvCmd = velArray*-1.0
dvCmd = vStartBurn*-1.0
dvCmd = dvCmd/numpy.linalg.norm(dvCmd)
dvCmd *= 1050.0
print tmT
SimulationBaseClass.SetCArray(dvCmd, 'double', TheEMMSim.dvGuidanceData.dvInrtlCmd)
SimulationBaseClass.SetCArray(hVec, 'double', TheEMMSim.dvGuidanceData.dvRotAxis)
TheEMMSim.dvGuidanceData.dvRotMag = 0.0004048673159899027
TheEMMSim.dvGuidanceData.burnStartTime = int((int(tmT - 2.0/3.0*totalBurnTime))*1E9)
print TheEMMSim.dvGuidanceData.burnStartTime*1.0E-9

TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(30*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.modeRequest = 'safeMode'
TheEMMSim.ConfigureStopTime(int(60*30*1*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.modeRequest = 'sunPoint'
TheEMMSim.ConfigureStopTime(int(TheEMMSim.dvGuidanceData.burnStartTime-600*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.modeRequest = 'DVPrep' #Note that this will maneuver to burn, burn, and go back to sunPoint
TheEMMSim.updateTaskPeriod("DynamicsTask", int(2.5E7))
TheEMMSim.attMnvrPointData.propagateReference = 0;
TheEMMSim.ConfigureStopTime(int(60*60*5*1E9))
TheEMMSim.ExecuteSimulation()

FSWsHat = TheEMMSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
numCSSActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')
attMnvrCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.sigmaCmd_BR')
bodyRateCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.omegaCmd_BR_B')
FSWControlOut = TheEMMSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))
vehInertia = TheEMMSim.pullMessageLogData("spacecraft_mass_props.InertiaTensor", range(9))
bodyRateObs =  TheEMMSim.GetLogVariableData('VehicleDynamicsData.omega')
DataSigma = TheEMMSim.pullMessageLogData("inertial_state_output.sigma", range(3))
DataDV = TheEMMSim.pullMessageLogData("inertial_state_output.TotalAccumDVBdy", range(3))
thrustLog = TheEMMSim.pullMessageLogData("dv_thruster_cmds.effectorRequest", range(6))
semiMajor = TheEMMSim.pullMessageLogData("OrbitalElements.a")
posMag = TheEMMSim.pullMessageLogData("OrbitalElements.rmag")
radApo = TheEMMSim.pullMessageLogData("OrbitalElements.rApoap")
radPeri = TheEMMSim.pullMessageLogData("OrbitalElements.rPeriap")
burnActive = TheEMMSim.GetLogVariableData('dvGuidance.burnExecuting')
burnComplete = TheEMMSim.GetLogVariableData('dvGuidance.burnComplete')
sigmaError = TheEMMSim.pullMessageLogData('nom_att_guid_out.sigma_BR', range(3))

CSSEstAccuracyThresh = 17.5*math.pi/180.0
plt.figure(1)
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'g', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'r', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')

plt.figure(2)
plt.plot(numCSSActive[:,0]*1.0E-9, numCSSActive[:,1])

plt.figure(3)
plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,1], 'b--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,1], 'b')
plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,2], 'r--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,2], 'r')
plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,3], 'g--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,3], 'g')

plt.figure(4)
plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,1])
plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,2])
plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,3])

plt.figure(5)
plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,1], 'b--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,1], 'b')
plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,2], 'r--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,2], 'r')
plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,3], 'g--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,3], 'g')

plt.figure(6)
plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,1])
plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,2])
plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,3])

plt.figure(7)
plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,1])
plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,5])
plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,9])

plt.figure(8)
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,1])
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,2])
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,3])
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,4])
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,5])
plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,6])

plt.figure(9)
plt.plot(semiMajor[:,0]*1.0E-9, semiMajor[:,1])

plt.figure(10)
plt.plot(posMag[:,0]*1.0E-9, posMag[:,1])

plt.figure(11)
plt.plot(radApo[:,0]*1.0E-9, radApo[:,1])

plt.figure(12)
plt.plot(radPeri[:,0]*1.0E-9, radPeri[:,1])

plt.figure(13)
plt.plot(burnActive[:,0]*1.0E-9, burnActive[:,1])
plt.plot(burnComplete[:,0]*1.0E-9, burnComplete[:,1], 'g--')

plt.figure(14)
plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,1])
plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,2])
plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,3])



if(len(sys.argv) > 1):
   if(sys.argv[1] == 'True'):
      plt.show()

#sys.exit()
