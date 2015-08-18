#Import some architectural stuff that we will probably always use
import sys, os
import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
#Point the path to the module storage area
sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')

#import all of the modules that we are using for this run
import star_tracker
import spice_interface
import sys_model_thread
import sim_model
import six_dof_eom
import orb_elem_convert
import thruster_dynamics
import coarse_sun_sensor
import ctypes
import numpy
import math
import csv
import copy
import cssComm
import alg_contain
import imu_sensor
import cssWlsEst
import sunSafePoint
import vehicleConfigData
import imuComm

#Using these gravity files to set gravity coefficients (might be overkill)
EarthGravFile = os.environ['SIMULATION_BASE']+'/External/LocalGravData/GGM03S.txt'
MarsGravFile = os.environ['SIMULATION_BASE']+'/External/LocalGravData/GGM2BData.txt'

import MessagingAccess
import SimulationBaseClass

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass()
TotalSim.CreateNewThread("DynamicsThread", int(1E7))
TotalSim.CreateNewThread("FSWThread", int(1E8))

LocalConfigData = vehicleConfigData.vehicleConfigData()
Tstr2Bdy = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
SimulationBaseClass.SetCArray(Tstr2Bdy, 'double', LocalConfigData.T_str2body)
TotalSim.TotalSim.CreateNewMessage("adcs_config_data", 8*9+4, 2)
TotalSim.TotalSim.WriteMessageData("adcs_config_data", 8*9+4, 0, LocalConfigData)

#Now initialize the modules that we are using.  I got a little better as I went along
SpiceObject = spice_interface.SpiceInterface()
SpiceObject.ModelTag = "SpiceInterfaceData"
SpiceObject.SPICEDataPath = os.environ['SIMULATION_BASE'] + '/External/EphemerisData/'
SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
SpiceObject.OutputBufferCount = 10000
SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars", "sun"])
TotalSim.AddModelToThread("DynamicsThread", SpiceObject)

#Not really a star tracker.  Just a garbage test object.  Throw it away
StObject1 = star_tracker.StarTracker()
StObject1.NoiseSigma = star_tracker.DoubleVector([1.0, 2.0, 3.0])
StObject1.ModelTag = "StarTracker1"
TotalSim.AddModelToThread("DynamicsThread", StObject1)

#This is a module set up to model a coarse sun sensor.
#Note the re-use between different instances of the modules.  
#Handy but not required.
CSSNoiseStd = 0.001 #Standard deviation of white noise
CSSNoiseBias = 0.0 #Constant bias
CSSscaleFactor = 500.0E-6 #Scale factor (500 mu-amps) for max measurement
CSSFOV = 90.0*math.pi/180.0 #90 degree field of view
CSSKellyFactor = 0.1 #Used to get the curve shape correct for output

#Platform 1 is forward, platform 2 is back notionally
CSSPlatform1YPR = [0.0, 0.0, 0.0]
CSSPlatform2YPR = [180.0*math.pi/180.0, 0.0, 0.0]

#Initialize one sensor by hand and then init the rest off of it
CSSPyramid1HeadA = coarse_sun_sensor.CoarseSunSensor()
CSSPyramid1HeadA.ModelTag = "CSSPyramid1HeadA"
CSSPyramid1HeadA.SenBias = CSSNoiseBias
CSSPyramid1HeadA.SenNoiseStd = CSSNoiseStd
CSSPyramid1HeadA.setStructureToPlatformDCM(CSSPlatform1YPR[0], 
   CSSPlatform1YPR[1], CSSPlatform1YPR[2])
CSSPyramid1HeadA.scaleFactor = CSSscaleFactor
CSSPyramid1HeadA.fov = CSSFOV
CSSPyramid1HeadA.KellyFactor = CSSKellyFactor
#This is the hook in to the messaging system.  Everyone needs a unique name
CSSPyramid1HeadA.OutputDataMsg = "coarse_sun_data_pyramid1_headA" 

#Initialize the rest of Pyramid 1 off of the first one and change names
CSSPyramid1HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
CSSPyramid1HeadB.ModelTag = "CSSPyramid1HeadB"
CSSPyramid1HeadB.OutputDataMsg = "coarse_sun_data_pyramid1_headB"
CSSPyramid1HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
CSSPyramid1HeadC.ModelTag = "CSSPyramid1HeadC"
CSSPyramid1HeadC.OutputDataMsg = "coarse_sun_data_pyramid1_headC"
CSSPyramid1HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
CSSPyramid1HeadD.ModelTag = "CSSPyramid1HeadD"
CSSPyramid1HeadD.OutputDataMsg = "coarse_sun_data_pyramid1_headD"

#Set up the sun sensor orientation information
#Maybe we should add the method call to the SelfInit of the CSS module
CSSPyramid1HeadA.theta = 0.0
CSSPyramid1HeadA.phi = 45.0*math.pi/180.0
CSSPyramid1HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid1HeadB.theta = 45.0*math.pi/180.0
CSSPyramid1HeadB.phi = 0.0
CSSPyramid1HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid1HeadC.theta = 0.0
CSSPyramid1HeadC.phi = -45.0*math.pi/180.0
CSSPyramid1HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid1HeadD.theta = -45.0*math.pi/180.0
CSSPyramid1HeadD.phi = 0.0
CSSPyramid1HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

#Schedule the first pyramid on the simulated sensor thread
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid1HeadA)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid1HeadB)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid1HeadC)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid1HeadD)

#Only difference between the sensors on the new pyramid are the names and the 
# DCM of the base platform
CSSPyramid2HeadA = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
CSSPyramid2HeadA.ModelTag = "CSSPyramid2HeadA"
CSSPyramid2HeadA.OutputDataMsg = "coarse_sun_data_pyramid2_headA"
CSSPyramid2HeadA.setStructureToPlatformDCM(CSSPlatform2YPR[0], 
   CSSPlatform2YPR[1], CSSPlatform2YPR[2])
CSSPyramid2HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid2HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadB)
CSSPyramid2HeadB.ModelTag = "CSSPyramid2HeadB"
CSSPyramid2HeadB.OutputDataMsg = "coarse_sun_data_pyramid2_headB"
CSSPyramid2HeadB.setStructureToPlatformDCM(CSSPlatform2YPR[0], 
   CSSPlatform2YPR[1], CSSPlatform2YPR[2])
CSSPyramid2HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid2HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadC)
CSSPyramid2HeadC.ModelTag = "CSSPyramid2HeadC"
CSSPyramid2HeadC.OutputDataMsg = "coarse_sun_data_pyramid2_headC"
CSSPyramid2HeadC.setStructureToPlatformDCM(CSSPlatform2YPR[0], 
   CSSPlatform2YPR[1], CSSPlatform2YPR[2])
CSSPyramid2HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

CSSPyramid2HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadD)
CSSPyramid2HeadD.ModelTag = "CSSPyramid2HeadD"
CSSPyramid2HeadD.OutputDataMsg = "coarse_sun_data_pyramid2_headD"
CSSPyramid2HeadD.setStructureToPlatformDCM(CSSPlatform2YPR[0], 
   CSSPlatform2YPR[1], CSSPlatform2YPR[2])
CSSPyramid2HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

#Schedule the second pyramid on the simulated sensor thread
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid2HeadA)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid2HeadB)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid2HeadC)
TotalSim.AddModelToThread("DynamicsThread", CSSPyramid2HeadD)

#Configure the IMU sensor. The ctypes ugliness is so that we can access a regular 
#array from SWIG.  We could easily hide that nastiness from the user in python.
RotBiasValue = 0.0;
RotNoiseStd = 0.000001;
TransBiasValue = 0.0
TransNoiseStd = 1.0E-6
IMUSensor = imu_sensor.ImuSensor()
IMUSensor.SensorPosStr = imu_sensor.DoubleVector([1.5, 0.1, 0.1])
IMUSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
RotBiasVec = ctypes.cast(IMUSensor.senRotBias.__long__(), ctypes.POINTER(ctypes.c_double))
RotNoiseVec = ctypes.cast(IMUSensor.senRotNoiseStd.__long__(), ctypes.POINTER(ctypes.c_double))
TransBiasVec = ctypes.cast(IMUSensor.senTransBias.__long__(), ctypes.POINTER(ctypes.c_double))
TransNoiseVec = ctypes.cast(IMUSensor.senTransNoiseStd.__long__(), ctypes.POINTER(ctypes.c_double))
i=0
while(i<3):
   RotBiasVec[i] = RotBiasValue
   RotNoiseVec[i] = RotNoiseStd
   TransBiasVec[i] = TransBiasValue
   TransNoiseVec[i] = TransNoiseStd
   i+=1

TotalSim.AddModelToThread("DynamicsThread", IMUSensor)

#I only created one thruster for now and set it up as a test point.  
#Note that the thrusters are a standard vector of configuration information per 
#thruster.  This should allow us to easily set up all of the thrusters like I 
#did earlier with the sun sensors
ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()
ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
Thruster1 = thruster_dynamics.ThrusterConfigData()
Thruster1.ThrusterLocation = thruster_dynamics.DoubleVector([2.0, 1.125, 0.0])
Thruster1.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, -math.cos(45.0*math.pi/180.0), math.sin(45.0*math.pi/180.0)])
Thruster1.MaxThrust = 0.9
ACSThrusterDynObject.ThrusterData = thruster_dynamics.ThrusterConfigVector([Thruster1])
TotalSim.AddModelToThread("DynamicsThread", ACSThrusterDynObject)

#Gravity.  The sun is pretty cut and dried.  I kind of went overboard with the 
#Earth and Mars.
VehDynObject = six_dof_eom.SixDofEOM()
SunGravBody = six_dof_eom.GravityBodyData()
SunGravBody.BodyMsgName = "sun_planet_data"
SunGravBody.mu = 132712440023.310*1000*1000*1000 #meters!
SunGravBody.IsCentralBody = True
SunGravBody.UseJParams = False

#So for Earth and MArs, I pulled some authoritative spherical harmonics models 
#for both bodies.  Then I wrote this basic parser to go and get the mu and 
#J parameters out of those files.  Kind of neat, but a little bit of overkill.
#I was really just testing to make sure that I could interface arbitrary pieces 
#of data at the python level without forcing changes into the source code.
EarthGravBody = six_dof_eom.GravityBodyData()
JParamsSelect = [2, 3, 4, 5, 6]
csvfile = open(EarthGravFile, 'rb')
csvreader = csv.reader(csvfile)
FirstLine = True
NextJindex = 0
AllJParams = []
for row in csvreader:
   if(FirstLine == True):
      EarthGravBody.mu = float(row[1])
      FirstLine = False
   elif(int(row[0]) == JParamsSelect[NextJindex]):
      LocalJParam = -math.sqrt(2*JParamsSelect[NextJindex]+1)*float(row[2])
      AllJParams.append(LocalJParam)
      NextJindex += 1
      if(NextJindex >= len(JParamsSelect)):
         break


EarthGravBody.BodyMsgName = "earth_planet_data"
EarthGravBody.IsCentralBody = False
EarthGravBody.UseJParams = False
EarthGravBody.JParams = six_dof_eom.DoubleVector(AllJParams)

MarsGravBody = six_dof_eom.GravityBodyData()
csvfile = open(MarsGravFile, 'rb')
csvreader = csv.reader(csvfile)
FirstLine = True
NextJindex = 0
AllJParams = []
for row in csvreader:
   if(FirstLine == True):
      MarsGravBody.mu = float(row[1])
      FirstLine = False
   elif(int(row[0]) == JParamsSelect[NextJindex]):
      LocalJParam = -math.sqrt(2*JParamsSelect[NextJindex]+1)*float(row[2])
      AllJParams.append(LocalJParam)
      NextJindex += 1
      if(NextJindex >= len(JParamsSelect)):
         break

MarsGravBody.BodyMsgName = "mars_planet_data"
MarsGravBody.IsCentralBody = False
MarsGravBody.UseJParams = False
MarsGravBody.JParams = six_dof_eom.DoubleVector(AllJParams)

#Assemble the actual vehicle dynamics.  Note how the interface to the standard 
#vectors is pretty clean.  Just have to call the vector init method basically.
#Also note that you can do basic (or really complex) math in the config set
VehDynObject.ModelTag = "VehicleDynamicsData"
VehDynObject.PositionInit = six_dof_eom.DoubleVector([2.342211275644610E+07*1000.0, -1.503236698659483E+08*1000.0, -1.786319594218582E+04*1000.0])
VehDynObject.VelocityInit = six_dof_eom.DoubleVector([2.896852053342327E+01*1000.0,  4.386175246767674E+00*1000.0, -3.469168621992313E-04*1000.0])
VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.0, 0.0, 0.0])
VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.0, 0.0, 0.0])
VehDynObject.MassInit = 1400.0
VehDynObject.InertiaInit = six_dof_eom.DoubleVector([1000, 0.0, 0.0, 
                                                     0.0, 1000.0, 0.0,
                                                     0.0, 0.0, 1000.0])
VehDynObject.T_Str2BdyInit = six_dof_eom.DoubleVector([1.0, 0.0, 0.0, 
                                                     0.0, 1.0, 0.0,
                                                     0.0, 0.0, 1.0])
VehDynObject.CoMInit = six_dof_eom.DoubleVector([1.0, 0.0, 0.0])
#Add the three gravity bodies in to the simulation
VehDynObject.AddGravityBody(SunGravBody)
VehDynObject.AddGravityBody(EarthGravBody)
VehDynObject.AddGravityBody(MarsGravBody)
#Here is where the thruster dynamics are attached/scheduled to the overall 
#vehicle dynamics.  Anything that is going to impact the dynamics of the vehicle 
# should be one of these body effectors I think.
VehDynObject.AddBodyEffector(ACSThrusterDynObject)
TotalSim.AddModelToThread("DynamicsThread", VehDynObject)

#Setting up orbital element computation in the run.  Note that I use it for ICs 
#first local to python and then add it as a scheduled job.  That in and of itself
# is really cool.
VehOrbElemObject = orb_elem_convert.OrbElemConvert()
VehOrbElemObject.ModelTag = "VehicleOrbitalElements"
VehOrbElemObject.CurrentElem.a = 188767262.18*1000.0;
VehOrbElemObject.CurrentElem.e = 0.207501;
VehOrbElemObject.CurrentElem.i = 0.0;
VehOrbElemObject.CurrentElem.Omega = 0.0;
VehOrbElemObject.CurrentElem.omega = 0.0;
VehOrbElemObject.CurrentElem.f = 70.0*math.pi/180.0
VehOrbElemObject.mu = SunGravBody.mu
#Convert those OEs to cartesian
VehOrbElemObject.Elements2Cartesian()
#Set dynamics inputs based on the computed cartesian data.  All happens in python.
PosVec = ctypes.cast(VehOrbElemObject.r_N.__long__(), ctypes.POINTER(ctypes.c_double))
VelVec = ctypes.cast(VehOrbElemObject.v_N.__long__(), ctypes.POINTER(ctypes.c_double))
AttVec = ctypes.cast(VehDynObject.sigma.__long__(), ctypes.POINTER(ctypes.c_double))
RateVec = ctypes.cast(VehDynObject.omega.__long__(), ctypes.POINTER(ctypes.c_double))
VehDynObject.PositionInit = six_dof_eom.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
VehDynObject.VelocityInit = six_dof_eom.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])
TotalSim.AddModelToThread("DynamicsThread", VehOrbElemObject)

#Configure a FSW algorithm.  First module is just the c-level of the algorithm
CSSDecodeFSWConfig = cssComm.CSSConfigData()
CSSDecodeFSWConfig.NumSensors = 8
CSSDecodeFSWConfig.MaxSensorValue = 500E-6
CSSDecodeFSWConfig.OutputDataName = "css_data_aggregate"
ChebyList = [-1.734963346951471e+06,   3.294117146099591e+06,
        -2.816333294617512e+06,   2.163709942144332e+06,
        -1.488025993860025e+06,   9.107359382775769e+05,
        -4.919712500291216e+05,   2.318436583511218e+05,
        -9.376105045529010e+04,   3.177536873430168e+04,
        -8.704033370738143e+03,   1.816188108176300e+03,
        -2.581556805090373e+02,   1.888418924282780e+01]
CSSDecodeFSWConfig.ChebyCount = len(ChebyList)
ChebyVec = ctypes.cast(CSSDecodeFSWConfig.KellyCheby.__long__(), ctypes.POINTER(ctypes.c_double))
i=0;
for elem in ChebyList:
   ChebyVec[i] = elem
   i+=1
CurrentName = cssComm.SensorMsgNameCarrier()
#Arrays of c-strings are hard for SWIG/python.  Required a bit of care.
SensorListUse = [
   "coarse_sun_data_pyramid1_headA",
   "coarse_sun_data_pyramid1_headB",
   "coarse_sun_data_pyramid1_headC",
   "coarse_sun_data_pyramid1_headD",
   "coarse_sun_data_pyramid2_headA",
   "coarse_sun_data_pyramid2_headB",
   "coarse_sun_data_pyramid2_headC",
   "coarse_sun_data_pyramid2_headD"
   ]

i=0
for Name in SensorListUse:
   CurrentName.SensorMsgName = Name
   cssComm.SensorNameArray_setitem(CSSDecodeFSWConfig.SensorList, i, CurrentName)
   i += 1

#Then the c algorithm needs to be wrapped with a C++ wrapper so that we can 
#treat it like the simulation modules created above.  This is probably 
#boilerplate for every module that we have to add.  Might want to make it simpler. 
CSSAlgWrap = alg_contain.AlgContain()
CSSAlgWrap.UseData(CSSDecodeFSWConfig)
CSSAlgWrap.UseUpdate(cssComm.Update_cssProcessTelem)
CSSAlgWrap.UseSelfInit(cssComm.SelfInit_cssProcessTelem)
CSSAlgWrap.UseCrossInit(cssComm.CrossInit_cssProcessTelem)
TotalSim.AddModelToThread("FSWThread", CSSAlgWrap)

IMUCommData = imuComm.IMUConfigData()
IMUCommData.InputDataName = "imu_meas_data"
IMUCommData.InputPropsName = "adcs_config_data"
IMUCommData.OutputDataName = "parsed_imu_data"
platform2str = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
SimulationBaseClass.SetCArray(Tstr2Bdy, 'double', IMUCommData.platform2StrDCM)
IMUAlgWrap = alg_contain.AlgContain()
IMUAlgWrap.UseData(IMUCommData)
IMUAlgWrap.UseUpdate(imuComm.Update_imuProcessTelem)
IMUAlgWrap.UseSelfInit(imuComm.SelfInit_imuProcessTelem)
IMUAlgWrap.UseCrossInit(imuComm.CrossInit_imuProcessTelem)
TotalSim.AddModelToThread("FSWThread", IMUAlgWrap)


CSSWlsEstFSWConfig = cssWlsEst.CSSWLSConfig()
CSSWlsEstFSWConfig.InputDataName = "css_data_aggregate"
CSSWlsEstFSWConfig.OutputDataName = "css_wls_est"
CSSWlsEstFSWConfig.UseWeights = True
CSSWlsEstFSWConfig.SensorUseThresh = 0.1

CSSConfigElement = cssWlsEst.SingleCSSConfig()
CSSConfigElement.CBias = 1.0
CSSConfigElement.cssNoiseStd = 0.2
CSSOrientationList = [
   [0.70710678118654757, 0, 0.70710678118654746],
   [0.70710678118654757, 0.70710678118654746, 0],
   [0.70710678118654757, 0, -0.70710678118654746],
   [0.70710678118654757, -0.70710678118654746, 0],
   [-0.70710678118654757, 0.0, 0.70710678118654746],
   [-0.70710678118654768, -0.70710678118654735, 0],
   [-0.70710678118654757, 0.0, -0.70710678118654746],
   [-0.70710678118654746, 0.70710678118654757, 0]
   ]
i=0
PointVec = ctypes.cast(CSSConfigElement.nHatBdy.__long__(), ctypes.POINTER(ctypes.c_double))
for CSSHat in CSSOrientationList:
   PointVec[0] = CSSHat[0]
   PointVec[1] = CSSHat[1]
   PointVec[2] = CSSHat[2]
   cssWlsEst.CSSWlsConfigArray_setitem(CSSWlsEstFSWConfig.CSSData, i, 
      CSSConfigElement)
   i += 1

CSSWlsAlgWrap = alg_contain.AlgContain()
CSSWlsAlgWrap.ModelTag = "CSSWlsEst"
CSSWlsAlgWrap.UseData(CSSWlsEstFSWConfig)
CSSWlsAlgWrap.UseUpdate(cssWlsEst.Update_cssWlsEst)
CSSWlsAlgWrap.UseSelfInit(cssWlsEst.SelfInit_cssWlsEst)
CSSWlsAlgWrap.UseCrossInit(cssWlsEst.CrossInit_cssWlsEst)
TotalSim.AddModelToThread("FSWThread", CSSWlsAlgWrap, CSSWlsEstFSWConfig)

sunSafePointData = sunSafePoint.sunSafePointConfig()
sunSafePointData.outputDataName = "sun_safe_att_err"
sunSafePointData.inputSunVecName = "css_wls_est"
sunSafePointData.inputIMUDataName = "parsed_imu_data"
sunSafePointData.minUnitMag = 0.95
PointVec = ctypes.cast(sunSafePointData.sHatBdyCmd.__long__(), ctypes.POINTER(ctypes.c_double))
PointVec[0] = 1.0
PointVec[1] = 0.0
PointVec[2] = 0.0
sunSafePointWrap = alg_contain.AlgContain()
sunSafePointWrap.ModelTag = "sunSafePoint"
sunSafePointWrap.UseData(sunSafePointData)
sunSafePointWrap.UseUpdate(sunSafePoint.Update_sunSafePoint)
sunSafePointWrap.UseSelfInit(sunSafePoint.SelfInit_sunSafePoint)
sunSafePointWrap.UseCrossInit(sunSafePoint.CrossInit_sunSafePoint)
TotalSim.AddModelToThread("FSWThread", sunSafePointWrap, sunSafePointData)

TotalSim.AddVariableForLogging('SpiceInterfaceData.GPSSeconds', int(1E9))
TotalSim.AddVariableForLogging('SpiceInterfaceData.J2000Current', int(1E9))
TotalSim.AddVariableForLogging('SpiceInterfaceData.GPSWeek', int(1E9))
TotalSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.a', int(1E9))
TotalSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.e', int(1E9))
TotalSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.i', int(1E9))
TotalSim.AddVariableForLogging('VehicleOrbitalElements.CurrentElem.f', int(1E9))
TotalSim.AddVectorForLogging('VehicleDynamicsData.sigma', 'double', 0, 2,  int(1E9))
TotalSim.AddVariableForLogging('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor', int(1E9))
TotalSim.AddVectorForLogging('CSSWlsEst.OutputData.sHatBdy', 'double', 0, 2, int(1E9))
TotalSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E9))
TotalSim.AddVariableForLogging('sunSafePoint.sunAngleErr', int(1E9))
TotalSim.AddVectorForLogging('sunSafePoint.attOut.sigma_BR', 'double', 0, 2, int(1E9))
TotalSim.AddVectorForLogging('sunSafePoint.attOut.omega_BR', 'double', 0, 2, int(1E9))
TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(int(60*1.0*1E9))

TotalSim.ExecuteSimulation()

ACSThrusterDynObject.NewThrustCmds = thruster_dynamics.DoubleVector([1.0])

TotalSim.ConfigureStopTime(int(60*20.0*1E9))
TotalSim.ExecuteSimulation()
ACSThrusterDynObject.NewThrustCmds[0] = 1.0

TotalSim.ConfigureStopTime(int(60*50.0*1E9))
TotalSim.ExecuteSimulation()

#DataSemi = TotalSim.GetLogVariableData('VehicleOrbitalElements.CurrentElem.a')
#DataSigma = TotalSim.GetLogVariableData('VehicleDynamicsData.sigma')
##DataThrust = TotalSim.GetLogVariableData('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor')
#DataCSSFSW = TotalSim.GetLogVariableData('CSSWlsEst.OutputData.sHatBdy')
#DataCSSTruth = TotalSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
#DatasunSafeAng = TotalSim.GetLogVariableData('sunSafePoint.sunAngleErr')
#DatasunSafeSigma = TotalSim.GetLogVariableData('sunSafePoint.attOut.sigma_BR')
#DatasunSafeOmega = TotalSim.GetLogVariableData('sunSafePoint.attOut.omega_BR')
#
#plt.figure(1)
#plt.plot(DataSemi[:,0], DataSemi[:,1] )
#
#plt.figure(2)
#plt.plot(DataSigma[:,0], DataSigma[:,1] )
#plt.plot(DataSigma[:,0], DataSigma[:,2] )
#plt.plot(DataSigma[:,0], DataSigma[:,3] )
#
##plt.figure(3)
##plt.plot(DataThrust[:,0], DataThrust[:,1], 'bx')
#
#plt.figure(4)
#plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,1], 'b', DataCSSTruth[:,0], DataCSSTruth[:,1], 'b--')
#plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,2], 'g', DataCSSTruth[:,0], DataCSSTruth[:,2], 'g--')
#plt.plot(DataCSSFSW[:,0], DataCSSFSW[:,3], 'r', DataCSSTruth[:,0], DataCSSTruth[:,3], 'r--')
#
#plt.figure(5)
#plt.plot(DatasunSafeAng[:,0], DatasunSafeAng[:,1] )
#
#plt.figure(6)
#plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,1] )
#plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,2] )
#plt.plot(DatasunSafeSigma[:,0], DatasunSafeSigma[:,3] )
#
#plt.figure(7)
#plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,1] )
#plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,2] )
#plt.plot(DatasunSafeOmega[:,0], DatasunSafeOmega[:,3] )
#
#
#plt.show()
