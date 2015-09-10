import sys, os
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')
import EMMSim
import thruster_dynamics
import matplotlib.pyplot as plt
import ctypes
import math
import MessagingAccess
import sim_model

TheEMMSim = EMMSim.EMMSim()
TheEMMSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E8))
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E8))
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
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
TheEMMSim.VehDynObject.PositionInit = thruster_dynamics.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
TheEMMSim.VehDynObject.VelocityInit = thruster_dynamics.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])


TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(60*120*1E9))
TheEMMSim.ExecuteSimulation()

FSWsHat = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
   'CSSWlsEstOut', 7200*2, TheEMMSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
numCSSActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')


plt.figure(1)
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'g', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'r', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')

plt.figure(2)
plt.plot(numCSSActive[:,0]*1.0E-9, numCSSActive[:,1])

plt.show()
