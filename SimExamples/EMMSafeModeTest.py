import EMMSim
import thruster_dynamics
import matplotlib.pyplot as plt
import ctypes
import math

TheEMMSim = EMMSim.EMMSim()
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.GPSSeconds', int(1E9))
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.J2000Current', int(1E9))
TheEMMSim.AddVariableForLogging('SpiceInterfaceData.GPSWeek', int(1E9))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.a', int(1E9))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.e', int(1E9))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.i', int(1E9))
TheEMMSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.f', int(1E9))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.sigma', 'double', 0, 2,  int(1E9))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E9))
TheEMMSim.AddVariableForLogging('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor', int(1E9))
TheEMMSim.AddVectorForLogging('CSSWlsEst.OutputData.sHatBdy', 'double', 0, 2, int(1E9))
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E9))
TheEMMSim.AddVariableForLogging('sunSafePoint.sunAngleErr', int(1E9))
TheEMMSim.AddVectorForLogging('sunSafePoint.attOut.sigma_BR', 'double', 0, 2, int(1E9))
TheEMMSim.AddVectorForLogging('sunSafePoint.attOut.omega_BR', 'double', 0, 2, int(1E9))

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
TheEMMSim.ConfigureStopTime(int(60*1.0*1E9))

TheEMMSim.ExecuteSimulation()

TheEMMSim.ACSThrusterDynObject.NewThrustCmds = thruster_dynamics.DoubleVector([1.0, 0, 0, 0, 0, 0, 0, 0])

TheEMMSim.ConfigureStopTime(int(60*20.0*1E9))
TheEMMSim.ExecuteSimulation()

TheEMMSim.ACSThrusterDynObject.NewThrustCmds[0] = 1.0

TheEMMSim.ConfigureStopTime(int(60*50.0*1E9))
TheEMMSim.ExecuteSimulation()

DataSemi = TheEMMSim.GetLogVariableData('VehicleOrbitalElements.CurrentElem.a')
DatasunSafeSigma = TheEMMSim.GetLogVariableData('sunSafePoint.attOut.sigma_BR')
DatasunSafeOmega = TheEMMSim.GetLogVariableData('sunSafePoint.attOut.omega_BR')
DataCSSFSW = TheEMMSim.GetLogVariableData('CSSWlsEst.OutputData.sHatBdy')
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
Dataomega = TheEMMSim.GetLogVariableData('VehicleDynamicsData.omega')


plt.figure(1)
plt.plot(DataSemi[:,0], DataSemi[:,1] )

plt.figure(2)
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,1] )
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,2] )
plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,3] )

plt.figure(3)
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,1], 'b', Dataomega[:,0], Dataomega[:,1], 'b--' )
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,2] , 'g' , Dataomega[:,0], Dataomega[:,2], 'g--' )
plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,3], 'r', Dataomega[:,0], Dataomega[:,3], 'r--'  )

plt.figure(4)
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,1], 'b', DataCSSTruth[:,0], DataCSSTruth[:,1], 'b--')
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,2], 'g', DataCSSTruth[:,0], DataCSSTruth[:,2], 'g--')
plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,3], 'r', DataCSSTruth[:,0], DataCSSTruth[:,3], 'r--')

plt.show()
