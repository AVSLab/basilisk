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

TheEMMSim.TotalSim.CreateNewMessage("att_cmd_output", 6*8, 2)

TheEMMSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E9))
TheEMMSim.TotalSim.logThisMessage("dv_thruster_cmds", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E10))
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E10))
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E10))
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E10))
TheEMMSim.TotalSim.logThisMessage("sun_safe_control_request", int(1E10))
TheEMMSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10))
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E10))
TheEMMSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E10))
TheEMMSim.AddVectorForLogging('attMnvrPoint.sigmaCmd', 'double', 0, 2, int(1E10))
TheEMMSim.AddVectorForLogging('attMnvrPoint.bodyRateCmd', 'double', 0, 2, int(1E10))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E10))
#TheEMMSim.AddVariableForLogging('DVThrusterDynamics.objProps.Mass', int(1E9))
#TheEMMSim.AddVariableForLogging('DVThrusterDynamics.mDotTotal', int(1E7))

TheEMMSim.VehDynObject.GravData[0].IsCentralBody = False
TheEMMSim.VehDynObject.GravData[0].IsDisplayBody = False
#TheEMMSim.VehDynObject.GravData[0].mu = 0.0
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
print TheEMMSim.VehOrbElemObject.CurrentElem.a
print TheEMMSim.VehOrbElemObject.CurrentElem.e
print TheEMMSim.VehOrbElemObject.CurrentElem.i
print TheEMMSim.VehOrbElemObject.CurrentElem.f
fLocal = TheEMMSim.VehOrbElemObject.CurrentElem.f
aLocal = TheEMMSim.VehOrbElemObject.CurrentElem.a
eLocal = TheEMMSim.VehOrbElemObject.CurrentElem.e
hLocal = 2.0 * math.atanh(math.sqrt((eLocal - 1.0) / (eLocal + 1.0)) * math.tan(fLocal / 2.0));
print hLocal
tmT = math.sqrt(-aLocal*aLocal*aLocal/TheEMMSim.VehOrbElemObject.mu)
tmT *= (TheEMMSim.VehOrbElemObject.CurrentElem.e*math.sinh(hLocal) - hLocal)
print tmT
TheEMMSim.VehOrbElemObject.CurrentElem.f = 0.0
TheEMMSim.VehOrbElemObject.Elements2Cartesian()
posArray = numpy.array([PosVec[0], PosVec[1], PosVec[2]])
velArray = numpy.array([VelVec[0], VelVec[1], VelVec[2]])

xaxis = posArray/numpy.linalg.norm(posArray)
zaxis = numpy.cross(posArray, velArray)
zaxis = zaxis/numpy.linalg.norm(zaxis)
yaxis = numpy.cross(zaxis, xaxis)
yaxis = yaxis/numpy.linalg.norm(yaxis)

transMatBase = numpy.vstack((xaxis, yaxis, zaxis))
hillTransMat = numpy.vstack(([1, 0, 0], [0, 0, -1], [0, 1, 0]))
totalTransMat = numpy.dot(hillTransMat, transMatBase)

transList = numpy.ndarray.tolist(totalTransMat[0,:])
transList.extend(numpy.ndarray.tolist(totalTransMat[1,:]))
transList.extend(numpy.ndarray.tolist(totalTransMat[2,:]))

Carray = sim_model.new_doubleArray(9)
MrpArray = sim_model.new_doubleArray(3)
SimulationBaseClass.SetCArray(transList, 'double', Carray)
sim_model.C2MRP(Carray, MrpArray)
MrpCommand = []
for index in range(3):
    MrpCommand.append(sim_model.doubleArray_getitem(MrpArray, index))

print transList
#TheEMMSim.VehOrbElemObject.mu = TheEMMSim.SunGravBody.mu

TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(60*30*1*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.disableThread("sunSafeFSWThread")
TheEMMSim.enableThread("vehicleAttMnvrFSWThread")
attMsgUse = [0.1, 0.3, -0.4]
CmdMessage = attMnvrPoint.attCmdOut()
SimulationBaseClass.SetCArray(MrpCommand, 'double', 
      CmdMessage.sigma_BR)
TheEMMSim.TotalSim.WriteMessageData("att_cmd_output", 6*8, 
    TheEMMSim.TotalSim.CurrentNanos, CmdMessage);
TheEMMSim.ConfigureStopTime(int(8184*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.disableThread("vehicleAttMnvrFSWThread")
TheEMMSim.enableThread("vehicleDVMnvrFSWThread")
TheEMMSim.ConfigureStopTime(int((8184+37.25*60)*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.disableThread("vehicleDVMnvrFSWThread")
TheEMMSim.enableThread("vehicleAttMnvrFSWThread")
TheEMMSim.ConfigureStopTime(int(60*60*140*1E9))
TheEMMSim.ExecuteSimulation()

FSWsHat = TheEMMSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
numCSSActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')
attMnvrCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.sigmaCmd')
bodyRateCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.bodyRateCmd')
FSWControlOut = TheEMMSim.pullMessageLogData("sun_safe_control_request.accelRequestBody", range(3))
vehInertia = TheEMMSim.pullMessageLogData("spacecraft_mass_props.InertiaTensor", range(9))
bodyRateObs =  TheEMMSim.GetLogVariableData('VehicleDynamicsData.omega')
DataSigma = TheEMMSim.pullMessageLogData("inertial_state_output.sigma", range(3))
DataDV = TheEMMSim.pullMessageLogData("inertial_state_output.TotalAccumDVBdy", range(3))
thrustLog = TheEMMSim.pullMessageLogData("dv_thruster_cmds.effectorRequest", range(6))
semiMajor = TheEMMSim.pullMessageLogData("OrbitalElements.a")
posMag = TheEMMSim.pullMessageLogData("OrbitalElements.rmag")
radApo = TheEMMSim.pullMessageLogData("OrbitalElements.rApoap")
radPeri = TheEMMSim.pullMessageLogData("OrbitalElements.rPeriap")

#dvConsumption = TheEMMSim.GetLogVariableData("DVThrusterDynamics.objProps.Mass")
#dvConsumption = TheEMMSim.GetLogVariableData("DVThrusterDynamics.mDotTotal")

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

if(len(sys.argv) > 1):
   if(sys.argv[1] == 'True'):
      plt.show()

#sys.exit()
