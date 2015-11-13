import sys, os, inspect #Don't worry about this, standard stuff plus file discovery
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import matplotlib #plotting package
matplotlib.use('TkAgg') #Plotter dies quietly on OSX sometimes with default
import EMMSim #The simulation
import matplotlib.pyplot as plt #shorthand makes plotting cmds easier
import ctypes #This is to allow you to set pointers in your c-code
import math #Isn't it obvious?
import sim_model #Sometimes need utilities out of here
import numpy #Lets us interact with nump objects which work like matlab arrays
import logging #Handy logging feature for test issues
import SimulationBaseClass #Need this to access some pointer manipulation

#Instantiate a copy of the EMM vehicle/FSW
TheEMMSim = EMMSim.EMMSim()

#Log a handful of messages to examine vehicle performance
TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E10)) #inertial states
TheEMMSim.TotalSim.logThisMessage("att_cmd_output", int(1E10)) #inertial states
TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E10)) #orbital elements
TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E10)) #FSW weighted least squares sun-vector
TheEMMSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10)) #spacecraft mass properties
TheEMMSim.TotalSim.logThisMessage("solar_array_sun_bore", int(1E10)) #solar array boresight angles
TheEMMSim.TotalSim.logThisMessage("high_gain_earth_bore", int(1E10)) #solar array boresight angles
TheEMMSim.TotalSim.logThisMessage("instrument_mars_bore", int(1E10)) #solar array boresight angles
TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E10))

#Setup a time in the science orbit well past our transition to science
TheEMMSim.SpiceObject.UTCCalInit = "2021 June 15, 00:00:00.0"

#Reset the dynamics to change the central body from Sun (index 0) to Mars (index 2)
TheEMMSim.VehDynObject.GravData[0].IsCentralBody = False
TheEMMSim.VehDynObject.GravData[0].IsDisplayBody = False
TheEMMSim.VehDynObject.GravData[2].IsCentralBody = True
TheEMMSim.VehDynObject.GravData[2].IsDisplayBody = True
TheEMMSim.VehOrbElemObject.mu = TheEMMSim.MarsGravBody.mu #central body of OE object

#Setting a dry mass CoM offset to be conservative
TheEMMSim.VehDynObject.baseCoMInit[0] = 0.05 #meters

#These mass properties are built assuming that we have already emptied the tank
#inserting into Mars orbit.  I'm assuming that we have 40% of the prop remaining 
#because that feels right.  Fix that!
DVpropCM = [0.0, 0.0, 1.0] #Rough center of mass for the prop tank
DVpropMass = (812.3-40)*0.4 #A theoretical "heavy" prop mass
DVpropRadius = 46.0/2.0/3.2808399/12.0 #Radius of propellant tank
sphereInerita = 2.0/5.0*DVpropMass*DVpropRadius*DVpropRadius #assume it is a sphere because why not?
DVInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita] #Inertia tensor
#now go and set the actual simulation variables with the calculated values
TheEMMSim.DVThrusterDynObject.objProps.Mass = DVpropMass
SimulationBaseClass.SetCArray(DVpropCM, 'double', TheEMMSim.DVThrusterDynObject.objProps.CoM)
SimulationBaseClass.SetCArray(DVInertia, 'double', TheEMMSim.DVThrusterDynObject.objProps.InertiaTensor)

#General place in the science orbit.  These parameters were made up without access 
#to any textbook or even the internet.  They are probably wrong.
TheEMMSim.VehOrbElemObject.CurrentElem.a = 39900000.0; #Should be close I think
TheEMMSim.VehOrbElemObject.CurrentElem.e = 0.01; #wrong.  Look this up
TheEMMSim.VehOrbElemObject.CurrentElem.i = 25.1*math.pi/180.0; #I think we're inclined this amount
TheEMMSim.VehOrbElemObject.CurrentElem.Omega = 0.0; #Zero because it is somewhat arbitrary
TheEMMSim.VehOrbElemObject.CurrentElem.omega = 0.0; #Zero because it is somewhat arbitrary
TheEMMSim.VehOrbElemObject.CurrentElem.f = -1.0*math.pi/180.0 #Gets us to periapsis 1q minutes in
#Convert those OEs to cartesian.  This is python-y.  Don't worry about it if it frightens you.
TheEMMSim.VehOrbElemObject.Elements2Cartesian()
PosVec = ctypes.cast(TheEMMSim.VehOrbElemObject.r_N.__long__(), 
   ctypes.POINTER(ctypes.c_double))
VelVec = ctypes.cast(TheEMMSim.VehOrbElemObject.v_N.__long__(), 
  ctypes.POINTER(ctypes.c_double))
TheEMMSim.VehDynObject.PositionInit = sim_model.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
TheEMMSim.VehDynObject.VelocityInit = sim_model.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])
TheEMMSim.VehOrbElemObject.Cartesian2Elements()

#Initialize simulation and free-drift for 30 seconds to let everything populate
TheEMMSim.InitializeSimulation()
TheEMMSim.ConfigureStopTime(int(30*1E9))
TheEMMSim.ExecuteSimulation()
#Command the FSW to go into safe mode and advance to ~ periapsis
TheEMMSim.modeRequest = 'safeMode'
TheEMMSim.ConfigureStopTime(int(60*11*1*1E9))
TheEMMSim.ExecuteSimulation()
#Take the vehicle into sun pointing mode and begin the science sequencing
TheEMMSim.modeRequest = 'sunPoint'
TheEMMSim.ConfigureStopTime(int(60*60*1*1E9))
TheEMMSim.ExecuteSimulation()
#Take the vehicle into earth pointing mode and begin the science sequencing
TheEMMSim.modeRequest = 'earthPoint'
TheEMMSim.ConfigureStopTime(int(60*60*2*1E9))
TheEMMSim.ExecuteSimulation()
#Take the vehicle into mars pointing mode and begin the science sequencing
TheEMMSim.modeRequest = 'marsPoint'
TheEMMSim.ConfigureStopTime(int(60*60*3*1E9))
TheEMMSim.ExecuteSimulation()

#Simulation complete.  Pull off a selected set of values from the variable logs
semiMajor = TheEMMSim.pullMessageLogData("OrbitalElements.a")
posMag = TheEMMSim.pullMessageLogData("OrbitalElements.rmag")
radApo = TheEMMSim.pullMessageLogData("OrbitalElements.rApoap")
radPeri = TheEMMSim.pullMessageLogData("OrbitalElements.rPeriap")
trueAnom = TheEMMSim.pullMessageLogData("OrbitalElements.f")
solarArrayMiss = TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")
highGainMiss = TheEMMSim.pullMessageLogData("high_gain_earth_bore.missAngle")
instrumentMiss = TheEMMSim.pullMessageLogData("instrument_mars_bore.missAngle")
DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
sigmaTruth = TheEMMSim.pullMessageLogData('inertial_state_output.sigma', range(3))
sigmaCMD = TheEMMSim.pullMessageLogData('att_cmd_output.sigma_BR', range(3))

#Plot true anomaly for the simulation
plt.figure(1)
plt.plot(trueAnom[:,0]*1.0E-9, trueAnom[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('True Anomaly (d)')

plt.figure(2)
plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')
plt.xlabel('Time (s)')
plt.ylabel('Structural Frame Sun Vector (-)')

plt.figure(3)
plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,1], 'b', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,1], 'b--')
plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,2], 'g', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,2], 'g--')
plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,3], 'r', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,3], 'r--')
plt.xlabel('Time (s)')
plt.ylabel('Attitude MRP (-)')

plt.figure(4)
plt.plot(solarArrayMiss[:,0]*1.0E-9, solarArrayMiss[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('Solar Array Miss (d)')

plt.figure(5)
plt.plot(highGainMiss[:,0]*1.0E-9, highGainMiss[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('High Gain Miss (d)')

plt.figure(6)
plt.plot(instrumentMiss[:,0]*1.0E-9, instrumentMiss[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('Instrument Nadir Miss (d)')


#If requested, generate plots
if(len(sys.argv) > 1):
   if(sys.argv[1] == 'True'):
      plt.show()

