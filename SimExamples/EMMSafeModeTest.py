import EMMSim
import thruster_dynamics
import matplotlib.pyplot as plt
import ctypes
import math

TheEMMSim = EMMSim.EMMSim()
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.GPSSeconds', int(1E8))
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.J2000Current', int(1E8))
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.GPSWeek', int(1E8))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.a', int(1E8))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.e', int(1E8))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.i', int(1E8))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.f', int(1E8))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.sigma', 'double', 0, 2,  int(1E8))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E8))
#TheEMMSim.AddVariableForLogging('SpiceObject.GPSSeconds', int(1E8))
#TheEMMSim.AddVariableForLogging('SpiceObject.J2000Current', int(1E8))
#TheEMMSim.AddVariableForLogging('SpiceObject.GPSWeek', int(1E8))
#TheEMMSim.AddVariableForLogging('VehOrbElemObject.CurrentElem.a', int(1E8))
#TheEMMSim.AddVariableForLogging('VehOrbElemObject.CurrentElem.e', int(1E8))
#TheEMMSim.AddVariableForLogging('VehOrbElemObject.CurrentElem.i', int(1E8))
#TheEMMSim.AddVariableForLogging('VehOrbElemObject.CurrentElem.f', int(1E8))
#TheEMMSim.AddVectorForLogging('VehDynObject.sigma', 'double', 0, 2,  int(1E8))
#TheEMMSim.AddVectorForLogging('VehDynObject.omega', 'double', 0, 2,  int(1E8))
TheEMMSim.AddVariableForLogging('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor', int(1E8))
TheEMMSim.AddVectorForLogging('CSSWlsEst.OutputData.sHatBdy', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E8))
TheEMMSim.AddVariableForLogging('CSSPyramid1HeadA.ScaledValue', int(1E8))
TheEMMSim.AddVariableForLogging('sunSafePoint.sunAngleErr', int(1E8))
TheEMMSim.AddVectorForLogging('sunSafePoint.attOut.sigma_BR', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('sunSafePoint.attOut.omega_BR', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('sunSafeControl.controlOut.accelRequestBody', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('sunSafeACS.cmdRequests.effectorRequest', 'double', 0, 7, int(1E8))
TheEMMSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E8))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.r_N', 'double', 0, 2,  int(1E8))

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
TheEMMSim.VehDynObject.PositionInit = thruster_dynamics.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
TheEMMSim.VehDynObject.VelocityInit = thruster_dynamics.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])


TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(60*60.0*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.ACSThrusterDynObject.NewThrustCmds[0] = 0.1
TheEMMSim.ConfigureStopTime(int(60*1440.0*1E9))
TheEMMSim.ExecuteSimulation()


DataSemi = TheEMMSim.GetLogVariableData('VehicleOrbitalElements.CurrentElem.a')
DatasunSafeSigma = TheEMMSim.GetLogVariableData('sunSafePoint.attOut.sigma_BR')
DatasunSafeOmega = TheEMMSim.GetLogVariableData('sunSafePoint.attOut.omega_BR')
DataCSSFSW = TheEMMSim.GetLogVariableData('CSSWlsEst.OutputData.sHatBdy')
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
Dataomega = TheEMMSim.GetLogVariableData('VehicleDynamicsData.omega')
Datacontrol = TheEMMSim.GetLogVariableData('sunSafeControl.controlOut.accelRequestBody')
Dataeffector = TheEMMSim.GetLogVariableData('sunSafeACS.cmdRequests.effectorRequest')
DataSigma = TheEMMSim.GetLogVariableData('VehicleDynamicsData.sigma')
DataSunSen = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.ScaledValue')
DataActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')
DataPosition = TheEMMSim.GetLogVariableData('VehicleDynamicsData.r_N')

print DataCSSTruth[DataCSSTruth.shape[0]-1, :]
print Datacontrol[Datacontrol.shape[0]-1, :]
print DataSigma[DataSigma.shape[0]-1, :]
print DataCSSFSW[DataCSSFSW.shape[0]-1, :]
print DataSunSen[DataSunSen.shape[0]-1, :]
print DataPosition[DataPosition.shape[0]-1, :]
print Dataomega[Dataomega.shape[0]-1, :]
#
plt.figure(1)
plt.plot(DataSemi[:,0], DataSemi[:,1] )
#
plt.figure(2)
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,1] )
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,2] )
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,3] )
#
plt.figure(3)
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,1], 'b', Dataomega[:,0], Dataomega[:,1], 'b--' )
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,2] , 'g' , Dataomega[:,0], Dataomega[:,2], 'g--' )
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,3], 'r', Dataomega[:,0], Dataomega[:,3], 'r--'  )
##
plt.figure(4)
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,1], 'b', DataCSSTruth[:,0], DataCSSTruth[:,1], 'b--')
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,2], 'g', DataCSSTruth[:,0], DataCSSTruth[:,2], 'g--')
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,3], 'r', DataCSSTruth[:,0], DataCSSTruth[:,3], 'r--')
#
plt.figure(5)
plt.plot(Datacontrol[:,0], Datacontrol[:,1] )
plt.plot(Datacontrol[:,0], Datacontrol[:,2] )
plt.plot(Datacontrol[:,0], Datacontrol[:,3] )
plt.figure(6)
plt.plot(Dataeffector[:,0], Dataeffector[:,1] )
plt.plot(Dataeffector[:,0], Dataeffector[:,2] )
plt.plot(Dataeffector[:,0], Dataeffector[:,3] )
plt.plot(Dataeffector[:,0], Dataeffector[:,4] )
plt.plot(Dataeffector[:,0], Dataeffector[:,5] )
plt.plot(Dataeffector[:,0], Dataeffector[:,6] )
plt.plot(Dataeffector[:,0], Dataeffector[:,7] )
plt.plot(Dataeffector[:,0], Dataeffector[:,8] )
#
plt.figure(7)
plt.plot(DataActive[:,0], DataActive[:,1] )

plt.figure(8)
plt.plot(DataSigma[:,0], DataSigma[:,1] )
plt.plot(DataSigma[:,0], DataSigma[:,2] )
plt.plot(DataSigma[:,0], DataSigma[:,3] )
#
plt.show()
