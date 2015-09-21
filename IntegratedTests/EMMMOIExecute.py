import sys, os
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')
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

TheEMMSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E8))
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E8))
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E8))
TheEMMSim.TotalSim.logThisMessage("sun_safe_control_request", int(1E8))
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E8))
TheEMMSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E8))
TheEMMSim.AddVectorForLogging('attMnvrPoint.sigmaCmd', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('attMnvrPoint.bodyRateCmd', 'double', 0, 2, int(1E8))
TheEMMSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E8))

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
TheEMMSim.ConfigureStopTime(int(60*60*1*1E9))
TheEMMSim.ExecuteSimulation()
TheEMMSim.disableThread("sunSafeFSWThread")
TheEMMSim.enableThread("vehicleAttMnvrFSWThread")
attMsgUse = [0.1, 0.3, -0.4]
CmdMessage = attMnvrPoint.attCmdOut()
SimulationBaseClass.SetCArray(attMsgUse, 'double', 
      CmdMessage.sigma_BR)
TheEMMSim.TotalSim.WriteMessageData("att_cmd_output", 6*8, 
    TheEMMSim.TotalSim.CurrentNanos, CmdMessage);
TheEMMSim.ConfigureStopTime(int(60*60*4*1E9))
TheEMMSim.ExecuteSimulation()

FSWsHat = MessagingAccess.obtainMessageVector("css_wls_est", 'cssWlsEst',
   'CSSWlsEstOut', 3600*2, TheEMMSim.TotalSim, 'sHatBdy', 'double', 0, 2, sim_model.logBuffer)
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
numCSSActive = TheEMMSim.GetLogVariableData('CSSWlsEst.numActiveCss')
attMnvrCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.sigmaCmd')
bodyRateCmd = TheEMMSim.GetLogVariableData('attMnvrPoint.bodyRateCmd')
FSWControlOut = MessagingAccess.obtainMessageVector("sun_safe_control_request", 'sunSafeControl',
   'vehControlOut', 7200*4, TheEMMSim.TotalSim, 'accelRequestBody', 'double', 0, 2, sim_model.logBuffer)
bodyRateObs =  TheEMMSim.GetLogVariableData('VehicleDynamicsData.omega')
DataSigma = MessagingAccess.obtainMessageVector("inertial_state_output", 'six_dof_eom',
   'OutputStateData', 7200*2, TheEMMSim.TotalSim, 'sigma', 'double', 0, 2, sim_model.logBuffer)

CSSEstAccuracyThresh = 17.5*math.pi/180.0
#accuracyFailCounter = checkCSSEstAccuracy(DataCSSTruth, FSWsHat, 
#   CSSEstAccuracyThresh)
#
#slewFinishTime = 150.0
#desiredSunBdy = [0.0, 0.0, 1.0]
#controlFailCounter =  checkSlewAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh,
#   slewFinishTime, desiredSunBdy)
#
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



if(len(sys.argv) > 1):
   if(sys.argv[1] == 'True'):
      plt.show()

sys.exit()
