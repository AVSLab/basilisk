#Import some architectural stuff that we will probably always use
import sys, os
#Simulation base class is needed because we inherit from it
import SimulationBaseClass
import RigidBodyKinematics
import numpy

#import regular python objects that we need
import math
import csv
import copy
#Vehicle dynamics and avionics models
import spice_interface
import sys_model_task
import sim_model
import six_dof_eom
import orb_elem_convert
import thruster_dynamics
import coarse_sun_sensor
import imu_sensor
import simple_nav
import bore_ang_calc
#FSW algorithms that we want to call
import cssComm
import alg_contain
import vehicleConfigData
import cssWlsEst
import sunSafePoint
import imuComm
import MRP_Steering
import sunSafeACS
import attMnvrPoint
import dvAttEffect
import dvGuidance
import attRefGen
import celestialBodyPoint
import clock_synch

class AVSSim(SimulationBaseClass.SimBaseClass):
 def __init__(self):
   #Create a sim module as an empty container
   SimulationBaseClass.SimBaseClass.__init__(self)
   self.modeRequest = 'None'
   self.fswProc = self.CreateNewProcess("FSWProcess")
   self.dynProc = self.CreateNewProcess("DynamicsProcess")
   self.dyn2FSWInterface = sim_model.SysInterface()
   self.fsw2DynInterface = sim_model.SysInterface()
   self.dyn2FSWInterface.addNewInterface("DynamicsProcess", "FSWProcess")
   self.fsw2DynInterface.addNewInterface("FSWProcess", "DynamicsProcess")
   self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
   self.fswProc.addInterfaceRef(self.fsw2DynInterface)
   self.dynProc.addTask(self.CreateNewTask("SynchTask", int(1E8)))
   self.dynProc.addTask(self.CreateNewTask("DynamicsTask", int(1E8)))
   self.fswProc.addTask(self.CreateNewTask("sunSafeFSWTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("sunPointTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("earthPointTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("marsPointTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("vehicleDVPrepFSWTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("vehicleAttMnvrFSWTask", int(5E8)))
   self.fswProc.addTask(self.CreateNewTask("vehicleDVMnvrFSWTask", int(5E8)))
   self.LocalConfigData = vehicleConfigData.vehicleConfigData()
   self.SpiceObject = spice_interface.SpiceInterface()
   self.InitCSSHeads()
   #Schedule the first pyramid on the simulated sensor Task
   self.IMUSensor = imu_sensor.ImuSensor()
   self.ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()
   self.DVThrusterDynObject = thruster_dynamics.ThrusterDynamics()
   self.VehDynObject = six_dof_eom.SixDofEOM()
   self.VehOrbElemObject = orb_elem_convert.OrbElemConvert()
   self.SimpleNavObject = simple_nav.SimpleNav()
   self.solarArrayBore = bore_ang_calc.BoreAngCalc();
   self.highGainBore = bore_ang_calc.BoreAngCalc();
   self.instrumentBore = bore_ang_calc.BoreAngCalc();
   self.clockSynchData = clock_synch.ClockSynch();
   self.InitAllDynObjects()
   self.disableTask("SynchTask")
   self.AddModelToTask("SynchTask", self.clockSynchData)
   self.AddModelToTask("DynamicsTask", self.SpiceObject)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid1HeadA)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid1HeadB)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid1HeadC)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid1HeadD)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid2HeadA)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid2HeadB)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid2HeadC)
   self.AddModelToTask("DynamicsTask", self.CSSPyramid2HeadD)
   self.AddModelToTask("DynamicsTask", self.IMUSensor)
   self.AddModelToTask("DynamicsTask", self.ACSThrusterDynObject)
   self.AddModelToTask("DynamicsTask", self.DVThrusterDynObject)
   self.AddModelToTask("DynamicsTask", self.VehDynObject)
   self.AddModelToTask("DynamicsTask", self.VehOrbElemObject)
   self.AddModelToTask("DynamicsTask", self.SimpleNavObject)
   self.AddModelToTask("DynamicsTask", self.solarArrayBore)
   self.AddModelToTask("DynamicsTask", self.instrumentBore)
   self.AddModelToTask("DynamicsTask", self.highGainBore)

   self.CSSDecodeFSWConfig = cssComm.CSSConfigData()
   self.CSSAlgWrap = alg_contain.AlgContain(self.CSSDecodeFSWConfig, 
      cssComm.Update_cssProcessTelem, cssComm.SelfInit_cssProcessTelem, 
      cssComm.CrossInit_cssProcessTelem)
   self.CSSAlgWrap.ModelTag = "cssSensorDecode"

   self.IMUCommData = imuComm.IMUConfigData()
   self.IMUCommWrap = alg_contain.AlgContain(self.IMUCommData, 
      imuComm.Update_imuProcessTelem, imuComm.SelfInit_imuProcessTelem, 
      imuComm.CrossInit_imuProcessTelem)
   self.IMUCommWrap.ModelTag = "imuSensorDecode"

   self.CSSWlsEstFSWConfig = cssWlsEst.CSSWLSConfig()
   self.CSSWlsWrap = alg_contain.AlgContain(self.CSSWlsEstFSWConfig,
      cssWlsEst.Update_cssWlsEst, cssWlsEst.SelfInit_cssWlsEst, 
      cssWlsEst.CrossInit_cssWlsEst)
   self.CSSWlsWrap.ModelTag = "CSSWlsEst"
   
   self.sunSafePointData = sunSafePoint.sunSafePointConfig()
   self.sunSafePointWrap = alg_contain.AlgContain(self.sunSafePointData,
      sunSafePoint.Update_sunSafePoint, sunSafePoint.SelfInit_sunSafePoint,
      sunSafePoint.CrossInit_sunSafePoint)
   self.sunSafePointWrap.ModelTag = "sunSafePoint"

   self.MRP_SteeringSafeData = MRP_Steering.MRP_SteeringConfig()
   self.MRP_SteeringWrap = alg_contain.AlgContain(self.MRP_SteeringSafeData,
      MRP_Steering.Update_MRP_Steering, MRP_Steering.SelfInit_MRP_Steering,
      MRP_Steering.CrossInit_MRP_Steering)
   self.MRP_SteeringWrap.ModelTag = "MRP_Steering"
 
   self.sunSafeACSData = sunSafeACS.sunSafeACSConfig()
   self.sunSafeACSWrap = alg_contain.AlgContain(self.sunSafeACSData,
      sunSafeACS.Update_sunSafeACS, sunSafeACS.SelfInit_sunSafeACS,
      sunSafeACS.CrossInit_sunSafeACS)
   self.sunSafeACSWrap.ModelTag = "sunSafeACS"

   self.attMnvrPointData = attRefGen.attRefGenConfig()
   self.attMnvrPointWrap = alg_contain.AlgContain(self.attMnvrPointData,
      attRefGen.Update_attRefGen, attRefGen.SelfInit_attRefGen,
      attRefGen.CrossInit_attRefGen)
   self.attMnvrPointWrap.ModelTag = "attMnvrPoint" 

   self.MRP_SteeringRWAData = MRP_Steering.MRP_SteeringConfig()
   self.MRP_SteeringRWAWrap = alg_contain.AlgContain(self.MRP_SteeringRWAData,
      MRP_Steering.Update_MRP_Steering, MRP_Steering.SelfInit_MRP_Steering,
      MRP_Steering.CrossInit_MRP_Steering)
   self.MRP_SteeringRWAData.ModelTag = "MRP_SteeringRWA"
   
   self.MRP_SteeringMOIData = MRP_Steering.MRP_SteeringConfig()
   self.MRP_SteeringMOIWrap = alg_contain.AlgContain(self.MRP_SteeringMOIData,
                                                    MRP_Steering.Update_MRP_Steering, MRP_Steering.SelfInit_MRP_Steering,
                                                    MRP_Steering.CrossInit_MRP_Steering)
   self.MRP_SteeringMOIWrap.ModelTag = "MRP_SteeringMOI"
   
   self.dvGuidanceData = dvGuidance.dvGuidanceConfig()
   self.dvGuidanceWrap = alg_contain.AlgContain(self.dvGuidanceData,
                                                 dvGuidance.Update_dvGuidance, dvGuidance.SelfInit_dvGuidance,
                                                 dvGuidance.CrossInit_dvGuidance)
   self.dvGuidanceWrap.ModelTag = "dvGuidance"
   
   self.dvAttEffectData = dvAttEffect.dvAttEffectConfig()
   self.dvAttEffectWrap = alg_contain.AlgContain(self.dvAttEffectData,
        dvAttEffect.Update_dvAttEffect, dvAttEffect.SelfInit_dvAttEffect,
        dvAttEffect.CrossInit_dvAttEffect)
   self.dvAttEffectWrap.ModelTag = "dvAttEffect"
   
   self.sunPointData = celestialBodyPoint.celestialBodyPointConfig()
   self.sunPointWrap = alg_contain.AlgContain(self.sunPointData,
       celestialBodyPoint.Update_celestialBodyPoint, celestialBodyPoint.SelfInit_celestialBodyPoint,
       celestialBodyPoint.CrossInit_celestialBodyPoint)
   self.sunPointWrap.ModelTag = "sunPoint"

   self.earthPointData = celestialBodyPoint.celestialBodyPointConfig()
   self.earthPointWrap = alg_contain.AlgContain(self.earthPointData,
       celestialBodyPoint.Update_celestialBodyPoint, celestialBodyPoint.SelfInit_celestialBodyPoint,
       celestialBodyPoint.CrossInit_celestialBodyPoint)
   self.earthPointWrap.ModelTag = "earthPoint"

   self.marsPointData = celestialBodyPoint.celestialBodyPointConfig()
   self.marsPointWrap = alg_contain.AlgContain(self.marsPointData,
       celestialBodyPoint.Update_celestialBodyPoint, celestialBodyPoint.SelfInit_celestialBodyPoint,
       celestialBodyPoint.CrossInit_celestialBodyPoint)
   self.marsPointWrap.ModelTag = "marsPoint"

   self.InitAllFSWObjects()

   self.AddModelToTask("sunSafeFSWTask", self.CSSAlgWrap, self.CSSDecodeFSWConfig)
   self.AddModelToTask("sunSafeFSWTask", self.IMUCommWrap, self.IMUCommData)
   self.AddModelToTask("sunSafeFSWTask", self.CSSWlsWrap, self.CSSWlsEstFSWConfig)
   self.AddModelToTask("sunSafeFSWTask", self.sunSafePointWrap, 
      self.sunSafePointData)
   self.AddModelToTask("sunSafeFSWTask", self.MRP_SteeringWrap, 
      self.MRP_SteeringSafeData)
   self.AddModelToTask("sunSafeFSWTask", self.sunSafeACSWrap, 
      self.sunSafeACSData)

   self.AddModelToTask("vehicleAttMnvrFSWTask", self.attMnvrPointWrap, 
      self.attMnvrPointData)
   self.AddModelToTask("vehicleAttMnvrFSWTask", self.MRP_SteeringRWAWrap, 
      self.MRP_SteeringRWAData)
   self.AddModelToTask("vehicleAttMnvrFSWTask", self.sunSafeACSWrap, 
      self.sunSafeACSData)
      
   self.AddModelToTask("vehicleDVPrepFSWTask", self.dvGuidanceWrap,
                            self.dvGuidanceData)
      
   self.AddModelToTask("vehicleDVMnvrFSWTask", self.dvGuidanceWrap,
                            self.dvGuidanceData)
   self.AddModelToTask("vehicleDVMnvrFSWTask", self.attMnvrPointWrap,
                            self.attMnvrPointData)
   self.AddModelToTask("vehicleDVMnvrFSWTask", self.MRP_SteeringMOIWrap,
                            self.MRP_SteeringMOIData)
   self.AddModelToTask("vehicleDVMnvrFSWTask", self.dvAttEffectWrap,
                            self.dvAttEffectData)
                            
   self.AddModelToTask("sunPointTask", self.sunPointWrap,
       self.sunPointData)

   self.AddModelToTask("earthPointTask", self.earthPointWrap,
       self.earthPointData)

   self.AddModelToTask("marsPointTask", self.marsPointWrap,
       self.marsPointData)

   self.fswProc.disableAllTasks()
 
   self.createNewEvent("initiateSafeMode", int(1E9), True, ["self.modeRequest == 'safeMode'"],
                          ["self.fswProc.disableAllTasks()",
                           "self.enableTask('sunSafeFSWTask')"])
   self.createNewEvent("initiateSunPoint", int(1E9), True, ["self.modeRequest == 'sunPoint'"],
                         [ "self.fswProc.disableAllTasks()", "self.enableTask('sunPointTask')",
                           "self.enableTask('vehicleAttMnvrFSWTask')", "self.attMnvrPointData.mnvrActive = False"])
   self.createNewEvent("initiateEarthPoint", int(1E9), True, ["self.modeRequest == 'earthPoint'"],
                         ["self.fswProc.disableAllTasks()", "self.enableTask('vehicleAttMnvrFSWTask')",
                          "self.enableTask('earthPointTask')", "self.attMnvrPointData.mnvrActive = False"])
   self.createNewEvent("initiateMarsPoint", int(1E9), True, ["self.modeRequest == 'marsPoint'"],
                         ["self.fswProc.disableAllTasks()", "self.enableTask('vehicleAttMnvrFSWTask')",
                          "self.enableTask('marsPointTask')", "self.attMnvrPointData.mnvrActive = False",
                          "self.attMnvrPointData.mnvrComplete = False",
                          "self.activateNextRaster()", "self.setEventActivity('completeRaster', True)"])
   self.createNewEvent("initiateDVPrep", int(1E9), True, ["self.modeRequest == 'DVPrep'"],
                         ["self.fswProc.disableAllTasks()", "self.enableTask('vehicleAttMnvrFSWTask')",
                          "self.enableTask('vehicleDVPrepFSWTask')", "self.attMnvrPointData.mnvrActive = False",
                          "self.setEventActivity('startDV', True)"])
   self.createNewEvent("initiateDVMnvr", int(1E9), True, ["self.modeRequest == 'DVMnvr'"],
                         ["self.fswProc.disableAllTasks()", "self.enableTask('vehicleDVMnvrFSWTask')",
                          "self.setEventActivity('completeDV', True)"])
   self.createNewEvent("completeDV", int(1E8), False, ["self.dvGuidanceData.burnComplete != 0"],
                    ["self.fswProc.disableAllTasks()", "self.enableTask('vehicleAttMnvrFSWTask')",
                     "self.attMnvrPointData.mnvrActive = False",
                     "self.setEventActivity('initiateSunPoint', True)", "self.modeRequest = 'sunPoint'"])
   self.createNewEvent("startDV", int(1E8), False, ["self.dvGuidanceData.burnStartTime <= self.TotalSim.CurrentNanos"],
                    ["self.modeRequest = 'DVMnvr'", "self.setEventActivity('initiateDVMnvr', True)"])
   self.createNewEvent("mnvrToRaster", int(1E9), False, ["self.attMnvrPointData.mnvrComplete == 1"],
                       ["self.activateNextRaster()", "self.setEventActivity('completeRaster', True)"])
   self.createNewEvent("completeRaster", int(1E9), False, ["self.attMnvrPointData.mnvrComplete == 1"],
                       ["self.initializeRaster()"])

   rastAngRad = 50.0*math.pi/180.0
   self.asteriskAngles = [[rastAngRad, 0.0, 0.0],
                          [-rastAngRad, 0.0, 0.0],
                          [-rastAngRad/math.sqrt(2.0), 0.0, -rastAngRad/math.sqrt(2.0)],
                          [rastAngRad/math.sqrt(2.0), 0.0, rastAngRad/math.sqrt(2.0)],
                          [0.0, 0.0, rastAngRad],
                          [0.0, 0.0, -rastAngRad],
                          [rastAngRad/math.sqrt(2.0), 0.0, -rastAngRad/math.sqrt(2.0)],
                          [-rastAngRad/math.sqrt(2.0), 0.0, rastAngRad/math.sqrt(2.0)],
                          [0.0, 0.0, 0.0]]
                          
   rastAngRad = 11.0*math.pi/180.0
   discAngleRad = 16.5*1.6*math.pi/180.0
   rasterTime = 25.0*60.0 + 100.0
   discAngRate = 2.0*discAngleRad/rasterTime
   self.sideScanAngles = [ \
                            [rastAngRad, 0.0, -discAngleRad],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, -discAngleRad],
                            [0.0, 0.0, 0.0],
                            [-rastAngRad, 0.0, discAngleRad] \
                            ]
   self.sideScanRate = [ \
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, discAngRate]\
                    ]
   self.sideScanTimes = [ rasterTime, 200.0, rasterTime, 200.0, rasterTime]
   self.scanSelector = 0
   self.scanAnglesUse = self.asteriskAngles
   self.scanRate = self.sideScanRate
   self.rasterTimes = self.sideScanTimes
 def initializeRaster(self):
     if(self.scanSelector != 0):
         self.setEventActivity('mnvrToRaster', True)
     else:
         SimulationBaseClass.SetCArray([0.0, 0.0, 0.0], 'double', self.attMnvrPointData.mnvrScanRate)
         self.setEventActivity('initiateSunPoint', True)
         if(self.modeRequest != 'earthPoint'):
             self.modeRequest = 'sunPoint'
         self.setEventActivity('initiateMarsPoint', True)

 def activateNextRaster(self):
     basePointMatrix = numpy.array(self.baseMarsTrans)
     basePointMatrix = numpy.reshape(basePointMatrix, (3,3))
     offPointAngles = numpy.array(self.scanAnglesUse[self.scanSelector])
     newScanAngles = self.scanRate[self.scanSelector]
     self.attMnvrPointData.totalMnvrTime = self.rasterTimes[self.scanSelector]
     self.scanSelector += 1
     self.scanSelector = self.scanSelector % len(self.scanAnglesUse)
     offPointAngles = numpy.reshape(offPointAngles, (3,1))
     offMatrix = RigidBodyKinematics.Euler1232C(offPointAngles)
     newPointMatrix = numpy.dot(offMatrix, basePointMatrix)
     newPointMatrix = numpy.reshape(newPointMatrix, 9).tolist()
     SimulationBaseClass.SetCArray(newPointMatrix[0], 'double', self.marsPointData.TPoint2Bdy)
     SimulationBaseClass.SetCArray(newScanAngles, 'double', self.attMnvrPointData.mnvrScanRate)
     self.attMnvrPointData.mnvrActive = False
     self.attMnvrPointData.mnvrComplete = 0
     print "Current Raster"
     print [self.TotalSim.CurrentNanos, self.scanSelector]
     
 def InitializeSimulation(self):
   SimulationBaseClass.SimBaseClass.InitializeSimulation(self)
   self.dyn2FSWInterface.discoverAllMessages()
   self.fsw2DynInterface.discoverAllMessages()
   
 # 
 # Set the static spacecraft parameters 
 #
 def SetLocalConfigData(self):
    #
    BS = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    SimulationBaseClass.SetCArray(BS, 'double', self.LocalConfigData.BS)

    Inertia = [1000.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 800]  # kg * m^2
    SimulationBaseClass.SetCArray(Inertia, 'double', self.LocalConfigData.I)

    # adjust the message size by hand if needed 
    msgSize = 8*9 + 8*9 + 4 + 8;    # the last 8 bytes are a required padding for now
    self.TotalSim.CreateNewMessage("FSWProcess", "adcs_config_data", msgSize, 2)
    self.TotalSim.WriteMessageData("adcs_config_data", msgSize, 0, self.LocalConfigData)
   
   
 def SetSpiceObject(self):
   self.SpiceObject.ModelTag = "SpiceInterfaceData"
   self.SpiceObject.SPICEDataPath = self.simBasePath + '/External/EphemerisData/'
   self.SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
   self.SpiceObject.OutputBufferCount = 2
   self.SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars", "sun"])
   self.SpiceObject.referenceBase = "MARSIAU"
 def SetIMUSensor(self):
   RotBiasValue = 0.0;
   RotNoiseStdValue = 0.000001;
   TransBiasValue = 0.0
   TransNoiseStdValue = 1.0E-6
   self.IMUSensor = imu_sensor.ImuSensor()
   self.IMUSensor.SensorPosStr = imu_sensor.DoubleVector([1.5, 0.1, 0.1])
   self.IMUSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
   SimulationBaseClass.SetCArray([RotBiasValue, RotBiasValue, RotBiasValue], 
      'double', self.IMUSensor.senRotBias)
   SimulationBaseClass.SetCArray([RotNoiseStdValue, RotNoiseStdValue, RotNoiseStdValue], 
      'double', self.IMUSensor.senRotNoiseStd)
   SimulationBaseClass.SetCArray([TransBiasValue, TransBiasValue, TransBiasValue], 
      'double', self.IMUSensor.senTransBias)
   SimulationBaseClass.SetCArray([TransNoiseStdValue, TransNoiseStdValue, TransNoiseStdValue], 
      'double', self.IMUSensor.senTransNoiseStd)
   self.IMUSensor.accelLSB = 2.77E-4*9.80665
   self.IMUSensor.gyroLSB = 8.75E-3*math.pi/180.0
    
 def SetACSThrusterDynObject(self):
   self.ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
   Thruster1 = thruster_dynamics.ThrusterConfigData()
   Thruster1.ThrusterLocation = thruster_dynamics.DoubleVector([-0.86360, -0.82550,  1.79070])
   Thruster1.ThrusterDirection = thruster_dynamics.DoubleVector([1.0, 0.0, 0.0])
   Thruster1.MaxThrust = 0.9
   Thruster1.MinOnTime = 0.020;
   Thruster2 = thruster_dynamics.ThrusterConfigData()
   Thruster2.ThrusterLocation = thruster_dynamics.DoubleVector([-0.82550, -0.86360,  1.79070])
   Thruster2.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, 1.0, 0.0])
   Thruster2.MaxThrust = 0.9
   Thruster2.MinOnTime = 0.020;
   Thruster3 = thruster_dynamics.ThrusterConfigData()
   Thruster3.ThrusterLocation = thruster_dynamics.DoubleVector([0.82550,  0.86360,  1.79070])
   Thruster3.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, -1.0, 0.0])
   Thruster3.MaxThrust = 0.9
   Thruster3.MinOnTime = 0.020;
   Thruster4 = thruster_dynamics.ThrusterConfigData()
   Thruster4.ThrusterLocation = thruster_dynamics.DoubleVector([0.86360,  0.82550,  1.79070])
   Thruster4.ThrusterDirection = thruster_dynamics.DoubleVector([-1.0, 0.0, 0.0])
   Thruster4.MaxThrust = 0.9
   Thruster4.MinOnTime = 0.020;
   Thruster5 = thruster_dynamics.ThrusterConfigData()
   Thruster5.ThrusterLocation = thruster_dynamics.DoubleVector([-0.86360, -0.82550,  0.18288])
   Thruster5.ThrusterDirection = thruster_dynamics.DoubleVector([1.0, 0.0, 0.0])
   Thruster5.MaxThrust = 0.9
   Thruster5.MinOnTime = 0.020;
   Thruster6 = thruster_dynamics.ThrusterConfigData()
   Thruster6.ThrusterLocation = thruster_dynamics.DoubleVector([-0.82550, -0.86360,  0.18288])
   Thruster6.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, 1.0, 0.0])
   Thruster6.MaxThrust = 0.9
   Thruster6.MinOnTime = 0.020;
   Thruster7 = thruster_dynamics.ThrusterConfigData()
   Thruster7.ThrusterLocation = thruster_dynamics.DoubleVector([0.82550,  0.86360,  0.18288])
   Thruster7.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, -1.0, 0.0])
   Thruster7.MaxThrust = 0.9
   Thruster7.MinOnTime = 0.020;
   Thruster8 = thruster_dynamics.ThrusterConfigData()
   Thruster8.ThrusterLocation = thruster_dynamics.DoubleVector([0.86360,  0.82550,  0.18288])
   Thruster8.ThrusterDirection = thruster_dynamics.DoubleVector([-1.0, 0.0, 0.0])
   Thruster8.MaxThrust = 0.9
   Thruster8.MinOnTime = 0.020;
   self.ACSThrusterDynObject.ThrusterData = \
      thruster_dynamics.ThrusterConfigVector([Thruster1, Thruster2, Thruster3, 
         Thruster4, Thruster5, Thruster6, Thruster7, Thruster8])
   ACSpropCM = [0.0, 0.0, 1.2]
   ACSpropMass = 40 #Made up!!!!
   ACSpropRadius = 46.0/2.0/3.2808399/12.0
   sphereInerita = 2.0/5.0*ACSpropMass*ACSpropRadius*ACSpropRadius
   ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
   self.ACSThrusterDynObject.objProps.Mass = ACSpropMass
   SimulationBaseClass.SetCArray(ACSpropCM, 'double', self.ACSThrusterDynObject.objProps.CoM)
   SimulationBaseClass.SetCArray(ACSInertia, 'double', self.ACSThrusterDynObject.objProps.InertiaTensor)
   
 def SetDVThrusterDynObject(self):
    self.DVThrusterDynObject.ModelTag = "DVThrusterDynamics"
    self.DVThrusterDynObject.InputCmds = "dv_thruster_cmds"
    self.DVThrusterDynObject.OutputDataString = "dv_thruster_output"
    allThrusters = []
    dvRadius = 0.256
    DVIsp = 226.7
    maxThrust = 111.33
    minOnTime = 0.020
    i=0
    angleInc = math.radians(60.0)
    while i < 6:
       newThruster = thruster_dynamics.ThrusterConfigData()
       newThruster.ThrusterLocation = thruster_dynamics.DoubleVector(
           [dvRadius*math.cos(i*angleInc), dvRadius*math.sin(i*angleInc), 0.0])
       newThruster.ThrusterDirection = thruster_dynamics.DoubleVector([0.0, 0.0, -1.0])
       newThruster.MaxThrust = maxThrust
       newThruster.MinOnTime = minOnTime
       newThruster.steadyIsp = DVIsp
       allThrusters.append(newThruster)
       i+=1
 
    self.DVThrusterDynObject.ThrusterData = \
        thruster_dynamics.ThrusterConfigVector(allThrusters)

    DVpropCM = [0.0, 0.0, 1.0]
    DVpropMass = 812.3-40 #The 40 comes from the made up ACS number!
    DVpropRadius = 46.0/2.0/3.2808399/12.0
    sphereInerita = 2.0/5.0*DVpropMass*DVpropRadius*DVpropRadius
    DVInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
    self.DVThrusterDynObject.objProps.Mass = DVpropMass
    SimulationBaseClass.SetCArray(DVpropCM, 'double', self.DVThrusterDynObject.objProps.CoM)
    SimulationBaseClass.SetCArray(DVInertia, 'double', self.DVThrusterDynObject.objProps.InertiaTensor)
 def InitCSSHeads(self):
   #Note the re-use between different instances of the modules.  
   #Handy but not required.
   CSSNoiseStd = 0.001 #Standard deviation of white noise
   CSSNoiseBias = 0.0 #Constant bias
   CSSscaleFactor = 500.0E-6 #Scale factor (500 mu-amps) for max measurement
   CSSFOV = 90.0*math.pi/180.0 #90 degree field of view
   CSSKellyFactor = 0.1 #Used to get the curve shape correct for output

   #Platform 1 is forward, platform 2 is back notionally
   CSSPlatform1YPR = [-math.pi/2.0, -math.pi/4.0, -math.pi/2.0]
   CSSPlatform2YPR = [0.0, -math.pi/2.0, 0.0]

   #Initialize one sensor by hand and then init the rest off of it
   self.CSSPyramid1HeadA = coarse_sun_sensor.CoarseSunSensor()
   self.CSSPyramid1HeadA.ModelTag = "CSSPyramid1HeadA"
   self.CSSPyramid1HeadA.SenBias = CSSNoiseBias
   self.CSSPyramid1HeadA.SenNoiseStd = CSSNoiseStd
   self.CSSPyramid1HeadA.setStructureToPlatformDCM(CSSPlatform1YPR[0],
      CSSPlatform1YPR[1], CSSPlatform1YPR[2])
   self.CSSPyramid1HeadA.scaleFactor = CSSscaleFactor
   self.CSSPyramid1HeadA.fov = CSSFOV
   self.CSSPyramid1HeadA.KellyFactor = CSSKellyFactor
   self.CSSPyramid1HeadA.OutputDataMsg = "coarse_sun_data_pyramid1_headA"
   self.CSSPyramid1HeadB = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadA)
   self.CSSPyramid1HeadB.ModelTag = "CSSPyramid1HeadB"
   self.CSSPyramid1HeadB.OutputDataMsg = "coarse_sun_data_pyramid1_headB"
   self.CSSPyramid1HeadC = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadA)
   self.CSSPyramid1HeadC.ModelTag = "CSSPyramid1HeadC"
   self.CSSPyramid1HeadC.OutputDataMsg = "coarse_sun_data_pyramid1_headC"
   self.CSSPyramid1HeadD = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadA)
   self.CSSPyramid1HeadD.ModelTag = "CSSPyramid1HeadD"
   self.CSSPyramid1HeadD.OutputDataMsg = "coarse_sun_data_pyramid1_headD"

   #Set up the sun sensor orientation information
   #Maybe we should add the method call to the SelfInit of the CSS module
   self.CSSPyramid1HeadA.theta = 0.0
   self.CSSPyramid1HeadA.phi = 45.0*math.pi/180.0
   self.CSSPyramid1HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid1HeadB.theta = 90.0*math.pi/180.0
   self.CSSPyramid1HeadB.phi = 45.0*math.pi/180.0
   self.CSSPyramid1HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid1HeadC.theta = 180.0*math.pi/180.0
   self.CSSPyramid1HeadC.phi = 45.0*math.pi/180.0
   self.CSSPyramid1HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid1HeadD.theta = 270.0*math.pi/180.0
   self.CSSPyramid1HeadD.phi = 45*math.pi/180.0
   self.CSSPyramid1HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid2HeadA = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadA)
   self.CSSPyramid2HeadA.ModelTag = "CSSPyramid2HeadA"
   self.CSSPyramid2HeadA.OutputDataMsg = "coarse_sun_data_pyramid2_headA"
   self.CSSPyramid2HeadA.setStructureToPlatformDCM(CSSPlatform2YPR[0],
      CSSPlatform2YPR[1], CSSPlatform2YPR[2])
   self.CSSPyramid2HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid2HeadB = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadB)
   self.CSSPyramid2HeadB.ModelTag = "CSSPyramid2HeadB"
   self.CSSPyramid2HeadB.OutputDataMsg = "coarse_sun_data_pyramid2_headB"
   self.CSSPyramid2HeadB.setStructureToPlatformDCM(CSSPlatform2YPR[0],
      CSSPlatform2YPR[1], CSSPlatform2YPR[2])
   self.CSSPyramid2HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid2HeadC = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadC)
   self.CSSPyramid2HeadC.ModelTag = "CSSPyramid2HeadC"
   self.CSSPyramid2HeadC.OutputDataMsg = "coarse_sun_data_pyramid2_headC"
   self.CSSPyramid2HeadC.setStructureToPlatformDCM(CSSPlatform2YPR[0],
      CSSPlatform2YPR[1], CSSPlatform2YPR[2])
   self.CSSPyramid2HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

   self.CSSPyramid2HeadD = coarse_sun_sensor.CoarseSunSensor(self.CSSPyramid1HeadD)
   self.CSSPyramid2HeadD.ModelTag = "CSSPyramid2HeadD"
   self.CSSPyramid2HeadD.OutputDataMsg = "coarse_sun_data_pyramid2_headD"
   self.CSSPyramid2HeadD.setStructureToPlatformDCM(CSSPlatform2YPR[0],
      CSSPlatform2YPR[1], CSSPlatform2YPR[2])
   self.CSSPyramid2HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

 def SetVehDynObject(self):
   self.SunGravBody = six_dof_eom.GravityBodyData()
   self.SunGravBody.BodyMsgName = "sun_planet_data"
   self.SunGravBody.outputMsgName = "sun_display_frame_data"
   self.SunGravBody.mu = 1.32712440018E20 #meters!
   self.SunGravBody.IsCentralBody = True
   self.SunGravBody.IsDisplayBody = True
   self.SunGravBody.UseJParams = False

   JParamsSelect = [2, 3, 4, 5, 6]
   EarthGravFile = self.simBasePath + '/External/LocalGravData/GGM03S.txt'
   MarsGravFile = self.simBasePath +'/External/LocalGravData/GGM2BData.txt'
   
   self.EarthGravBody = six_dof_eom.GravityBodyData()
   self.EarthGravBody.BodyMsgName = "earth_planet_data"
   self.EarthGravBody.outputMsgName = "earth_display_frame_data"
   self.EarthGravBody.IsCentralBody = False
   self.EarthGravBody.UseJParams = False
   JParams = LoadGravFromFile(EarthGravFile, self.EarthGravBody, JParamsSelect)
   self.EarthGravBody.JParams = six_dof_eom.DoubleVector(JParams)   

   self.MarsGravBody = six_dof_eom.GravityBodyData()
   self.MarsGravBody.BodyMsgName = "mars_planet_data"
   self.MarsGravBody.outputMsgName = "mars_display_frame_data"
   self.MarsGravBody.IsCentralBody = False
   self.MarsGravBody.UseJParams = True
   JParams = LoadGravFromFile(MarsGravFile, self.MarsGravBody, JParamsSelect)
   self.MarsGravBody.JParams = six_dof_eom.DoubleVector(JParams)   
  
   self.VehDynObject.ModelTag = "VehicleDynamicsData"
   self.VehDynObject.PositionInit = six_dof_eom.DoubleVector([2.342211275644610E+07*1000.0, -1.503236698659483E+08*1000.0, -1.786319594218582E+04*1000.0])
   self.VehDynObject.VelocityInit = six_dof_eom.DoubleVector([2.896852053342327E+01*1000.0,  4.386175246767674E+00*1000.0, -3.469168621992313E-04*1000.0])
   self.VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.4, 0.2, 0.1])
   self.VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.0001, 0.0, 0.0])
   self.VehDynObject.baseMass = 1500.0 - 812.3
   self.VehDynObject.baseInertiaInit = six_dof_eom.DoubleVector([1000, 0.0, 0.0,
                                                             0.0, 800.0, 0.0,
                                                             0.0, 0.0, 800.0])
   self.VehDynObject.T_Str2BdyInit = six_dof_eom.DoubleVector([1.0, 0.0, 0.0,
                                                               0.0, 1.0, 0.0,
                                                               0.0, 0.0, 1.0])
   self.VehDynObject.baseCoMInit = six_dof_eom.DoubleVector([0.0, 0.0, 1.0])
   #Add the three gravity bodies in to the simulation
   self.VehDynObject.AddGravityBody(self.SunGravBody)
   self.VehDynObject.AddGravityBody(self.EarthGravBody)
   self.VehDynObject.AddGravityBody(self.MarsGravBody)
   #Here is where the thruster dynamics are attached/scheduled to the overall 
   #vehicle dynamics.  Anything that is going to impact the dynamics of the vehicle 
   # should be one of these body effectors I think.
   self.VehDynObject.addThrusterSet(self.ACSThrusterDynObject)
   self.VehDynObject.addThrusterSet(self.DVThrusterDynObject)

 def SetVehOrbElemObject(self):
   self.VehOrbElemObject.ModelTag = "VehicleOrbitalElements"
   self.VehOrbElemObject.mu = self.SunGravBody.mu
 
 def SetsolarArrayBore(self):
   self.solarArrayBore.ModelTag = "solarArrayBoresight"
   self.solarArrayBore.StateString = "inertial_state_output"
   self.solarArrayBore.celBodyString = "sun_display_frame_data"
   self.solarArrayBore.OutputDataString = "solar_array_sun_bore"
   SimulationBaseClass.SetCArray([0.0, 0.0, 1.0], 'double',
                                 self.solarArrayBore.strBoreVec)
 def SethighGainBore(self):
    self.highGainBore.ModelTag = "highGainBoresight"
    self.highGainBore.StateString = "inertial_state_output"
    self.highGainBore.celBodyString = "earth_display_frame_data"
    self.highGainBore.OutputDataString = "high_gain_earth_bore"
    angSin = math.sin(23.0*math.pi/180.0)
    angCos = math.cos(23.0*math.pi/180.0)
    SimulationBaseClass.SetCArray([0.0, -angSin, angCos], 'double',
                      self.highGainBore.strBoreVec)

 def SetinstrumentBore(self):
    self.instrumentBore.ModelTag = "instrumentBoresight"
    self.instrumentBore.StateString = "inertial_state_output"
    self.instrumentBore.celBodyString = "mars_display_frame_data"
    self.instrumentBore.OutputDataString = "instrument_mars_bore"
    SimulationBaseClass.SetCArray([0.0, 1.0, 0.0], 'double',
       self.instrumentBore.strBoreVec)
 
 def SetSimpleNavObject(self):
   self.SimpleNavObject.ModelTag = "SimpleNavigation"
   PMatrix = [0.0]*18*18
   PMatrix[0*18+0] = PMatrix[1*18+1] = PMatrix[2*18+2] = 10.0; #Position
   PMatrix[3*18+3] = PMatrix[4*18+4] = PMatrix[5*18+5] = 0.05; #Velocity
   PMatrix[6*18+6] = PMatrix[7*18+7] = PMatrix[8*18+8] = 1.0/3600.0*math.pi/180.0; #Attitude (sigma!)
   PMatrix[9*18+9] = PMatrix[10*18+10] = PMatrix[11*18+11] = 0.001*math.pi/180.0; #Attitude rate
   PMatrix[12*18+12] = PMatrix[13*18+13] = PMatrix[14*18+14] = 0.1*math.pi/180.0; #Sun vector
   PMatrix[15*18+15] = PMatrix[16*18+16] = PMatrix[17*18+17] = 0.003; #Accumulated DV
   errorBounds = [1000.0, 1000.0, 1000.0, #Position
                  1.0, 1.0, 1.0, #Velocity
                  1.6E-2*math.pi/180.0, 1.6E-2*math.pi/180.0, 1.6E-2*math.pi/180.0, #Attitude
                  0.001*math.pi/180.0, 0.001*math.pi/180.0, 0.001*math.pi/180.0, #Attitude Rate
                  5.0*math.pi/180.0, 5.0*math.pi/180.0, 5.0*math.pi/180.0, #Sun vector
                  0.053, 0.053, 0.053] #Accumulated DV
   self.SimpleNavObject.walkBounds = sim_model.DoubleVector(errorBounds)
   self.SimpleNavObject.PMatrix = sim_model.DoubleVector(PMatrix)
   self.SimpleNavObject.crossTrans = True
   self.SimpleNavObject.crossAtt = False
 def SetclockSynchData(self):
   self.clockSynchData.ModelTag = "ClockSynchModel"
   self.clockSynchData.accelFactor = 1.0
   self.clockSynchData.clockOutputName = "clock_synch_data"
   self.clockSynchData.outputBufferCount = 2
 def SetCSSDecodeFSWConfig(self):
   self.CSSDecodeFSWConfig.NumSensors = 8
   self.CSSDecodeFSWConfig.MaxSensorValue = 500E-6
   self.CSSDecodeFSWConfig.OutputDataName = "css_data_aggregate"
   ChebyList = [-1.734963346951471e+06,   3.294117146099591e+06,
           -2.816333294617512e+06,   2.163709942144332e+06,
           -1.488025993860025e+06,   9.107359382775769e+05,
           -4.919712500291216e+05,   2.318436583511218e+05,
           -9.376105045529010e+04,   3.177536873430168e+04,
           -8.704033370738143e+03,   1.816188108176300e+03,
           -2.581556805090373e+02,   1.888418924282780e+01]
   self.CSSDecodeFSWConfig.ChebyCount = len(ChebyList)
   SimulationBaseClass.SetCArray(ChebyList, 'double', 
      self.CSSDecodeFSWConfig.KellyCheby)
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
      cssComm.SensorNameArray_setitem(self.CSSDecodeFSWConfig.SensorList, i, 
         CurrentName)
      i += 1

 def SetIMUCommData(self):
   self.IMUCommData.InputDataName = "imu_meas_data"
   self.IMUCommData.InputPropsName = "adcs_config_data"
   self.IMUCommData.OutputDataName = "parsed_imu_data"
   platform2str = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
   SimulationBaseClass.SetCArray(platform2str, 'double', 
      self.IMUCommData.platform2StrDCM)

 def SetCSSWlsEstFSWConfig(self):
   self.CSSWlsEstFSWConfig.InputDataName = "css_data_aggregate"
   self.CSSWlsEstFSWConfig.OutputDataName = "css_wls_est"
   self.CSSWlsEstFSWConfig.UseWeights = True
   self.CSSWlsEstFSWConfig.SensorUseThresh = 0.1

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
      cssWlsEst.CSSWlsConfigArray_setitem(self.CSSWlsEstFSWConfig.CSSData, i,
         CSSConfigElement)
      i += 1

 def SetsunSafePoint(self):
   self.sunSafePointData.outputDataName = "sun_safe_att_err"
   self.sunSafePointData.inputSunVecName = "css_wls_est"
   self.sunSafePointData.inputIMUDataName = "parsed_imu_data"
   self.sunSafePointData.minUnitMag = 0.95
   SimulationBaseClass.SetCArray([0.0, 0.0, 1.0], 'double', 
      self.sunSafePointData.sHatBdyCmd)

 def SetMRP_Steering(self):
   self.MRP_SteeringSafeData.K1 = 0.15            # rad/sec
   self.MRP_SteeringSafeData.K3 = 1.0             # rad/sec
   self.MRP_SteeringSafeData.omega_max = 1.5*(math.pi/180.) # rad/sec
   self.MRP_SteeringSafeData.P = 150.0            # N*m*sec
   self.MRP_SteeringSafeData.Ki = -1.0             # N*m  - negative values turn off the integral feedback
   self.MRP_SteeringSafeData.integralLimit = 0.15 # rad
   self.MRP_SteeringSafeData.inputGuidName = "sun_safe_att_err"
   self.MRP_SteeringSafeData.inputVehicleConfigDataName = "adcs_config_data"
   self.MRP_SteeringSafeData.inputNavName = "simple_nav_output"
   self.MRP_SteeringSafeData.outputDataName = "controlTorqueRaw"

 def SetsunSafeACS(self):
   self.sunSafeACSData.inputControlName = "controlTorqueRaw"
   self.sunSafeACSData.thrData.outputDataName = "acs_thruster_cmds"
   self.sunSafeACSData.thrData.minThrustRequest = 0.1
   self.sunSafeACSData.thrData.numEffectors = 8
   self.sunSafeACSData.thrData.maxNumCmds = 2
   onTimeMap = [0.0, 1.0, 0.7, 
                 -1.0, 0.0, -0.7,
                 1.0, 0.0, -0.7,
                 0.0, -1.0, 0.7,
                 0.0, -1.0, 0.7,
                 1.0, 0.0, -0.7,
                 -1.0, 0.0, -0.7,
                 0.0, 1.0, 0.7]
   SimulationBaseClass.SetCArray(onTimeMap, 'double', 
      self.sunSafeACSData.thrData.thrOnMap)
 def SetattMnvrPoint(self):
   self.attMnvrPointData.inputNavStateName = "simple_nav_output" 
   self.attMnvrPointData.inputAttCmdName = "att_cmd_output"
   self.attMnvrPointData.outputDataName = "nom_att_guid_out"
   self.attMnvrPointData.zeroAngleTol = 1.0*math.pi/180.0
   self.attMnvrPointData.mnvrActive = 0
   self.attMnvrPointData.totalMnvrTime = 1000.0
   self.attMnvrPointData.propagateReference = 1

 def SetMRP_SteeringRWA(self):
   self.MRP_SteeringRWAData.K1 = 0.3         # rad/sec
   self.MRP_SteeringRWAData.K3 = 1.0           # rad/sec
   self.MRP_SteeringRWAData.omega_max = 1.5*(math.pi/180.) # rad/sec
   self.MRP_SteeringRWAData.P = 150.0            # N*m*sec
   self.MRP_SteeringRWAData.Ki = -1.0         # N*m  - negative values turn off the integral feedback
   self.MRP_SteeringRWAData.integralLimit = 0.0 # rad
   self.MRP_SteeringRWAData.inputGuidName = "nom_att_guid_out"
   self.MRP_SteeringRWAData.inputVehicleConfigDataName = "adcs_config_data"
   self.MRP_SteeringRWAData.inputNavName = "simple_nav_output"
   self.MRP_SteeringRWAData.outputDataName = "controlTorqueRaw"
 
 def SetMRP_SteeringMOI(self):
    self.MRP_SteeringMOIData.K1 = 0.9         # rad/sec
    self.MRP_SteeringMOIData.K3 = 3.0           # rad/sec
    self.MRP_SteeringMOIData.omega_max = 1.5*(math.pi/180.) # rad/sec
    self.MRP_SteeringMOIData.P = 200.0            # N*m*sec
    self.MRP_SteeringMOIData.Ki = 11.7         # N*m  - negative values turn off the integral feedback
    self.MRP_SteeringMOIData.integralLimit = 0.5 # rad
    self.MRP_SteeringMOIData.inputGuidName = "nom_att_guid_out"
    self.MRP_SteeringMOIData.inputVehicleConfigDataName = "adcs_config_data"
    self.MRP_SteeringMOIData.inputNavName = "simple_nav_output"
    self.MRP_SteeringMOIData.outputDataName = "controlTorqueRaw"
 
 def SetdvAttEffect(self):
   self.dvAttEffectData.inputControlName = "controlTorqueRaw"
   self.dvAttEffectData.numThrGroups = 2
   newThrGroup = dvAttEffect.ThrustGroupData()
   newThrGroup.outputDataName = "acs_thruster_cmds"
   newThrGroup.minThrustRequest = 0.1
   newThrGroup.numEffectors = 8
   newThrGroup.maxNumCmds = 1
   onTimeMap = [0.0, 0.0, 1.0,
                0.0, 0.0, -1.0,
                0.0, 0.0, -1.0,
                0.0, 0.0, 1.0,
                0.0, 0.0, 1.0,
                0.0, 0.0, -1.0,
                0.0, 0.0, -1.0,
                0.0, 0.0, 1.0]
   SimulationBaseClass.SetCArray(onTimeMap, 'double', newThrGroup.thrOnMap)
   dvAttEffect.ThrustGroupArray_setitem(self.dvAttEffectData.thrGroups, 0,
                                       newThrGroup)
   newThrGroup.numEffectors = 6
   newThrGroup.maxNumCmds = 6
   newThrGroup.nomThrustOn = 0.52
   
   newThrGroup.outputDataName = "dv_thruster_cmds"
   matMult = 0.7
   onTimeMap = [0.0, 0.1*matMult, 0.0,
                -0.0866*matMult, 0.05*matMult, 0.0,
                -0.0866*matMult, -0.05*matMult, 0.0,
                0.0, -0.1*matMult, 0.0,
                0.0866*matMult, -0.05*matMult, 0.0,
                0.0866*matMult, 0.05*matMult, 0.0]
   SimulationBaseClass.SetCArray(onTimeMap, 'double', newThrGroup.thrOnMap)
   dvAttEffect.ThrustGroupArray_setitem(self.dvAttEffectData.thrGroups, 1,
                                     newThrGroup)
 def SetdvGuidance(self):
   self.dvGuidanceData.outputDataName = "att_cmd_output"
   self.dvGuidanceData.inputNavDataName = "simple_nav_output"
   self.dvGuidanceData.inputMassPropName = "adcs_config_data"
   desiredBurnDir = [1.0, 0.0, 0.0]
   desiredOffAxis = [0.0, 1.0, 0.0]
   Tburn2Body = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]
   SimulationBaseClass.SetCArray(desiredBurnDir, 'double', self.dvGuidanceData.dvInrtlCmd)
   SimulationBaseClass.SetCArray(Tburn2Body, 'double', self.dvGuidanceData.Tburn2Bdy)
   SimulationBaseClass.SetCArray(desiredOffAxis, 'double', self.dvGuidanceData.dvRotAxis)
 
 def SetsunPoint(self):
   self.sunPointData.inputNavDataName = "simple_nav_output"
   self.sunPointData.inputCelMessName = "sun_display_frame_data"
   self.sunPointData.outputDataName = "att_cmd_output"
   self.sunPointData.inputSecMessName = "earth_display_frame_data"
   TsunVec2Body = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
   SimulationBaseClass.SetCArray(TsunVec2Body, 'double', self.sunPointData.TPoint2Bdy)

 def SetearthPoint(self):
   self.earthPointData.inputNavDataName = "simple_nav_output"
   self.earthPointData.inputCelMessName = "earth_display_frame_data"
   self.earthPointData.outputDataName = "att_cmd_output"
   self.earthPointData.inputSecMessName = "sun_display_frame_data"
   angSin = math.sin(23.0*math.pi/180.0)
   angCos = math.cos(23.0*math.pi/180.0)
   TearthVec2Body = [0.0, 0.0, -1.0, -angSin, angCos, 0.0, angCos, angSin, 0.0]
   SimulationBaseClass.SetCArray(TearthVec2Body, 'double', self.earthPointData.TPoint2Bdy)

 def SetmarsPoint(self):
   self.marsPointData.inputNavDataName = "simple_nav_output"
   self.marsPointData.inputCelMessName = "mars_display_frame_data"
   self.marsPointData.inputSecMessName = "sun_display_frame_data"
   self.marsPointData.outputDataName = "att_cmd_output"
   TmarsVec2Body = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
   #TmarsVec2Body = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
   self.baseMarsTrans = TmarsVec2Body
   SimulationBaseClass.SetCArray(TmarsVec2Body, 'double', self.marsPointData.TPoint2Bdy)

 def InitAllDynObjects(self):
   self.SetSpiceObject()
   self.SetIMUSensor()
   self.SetACSThrusterDynObject()
   self.SetDVThrusterDynObject()
   self.SetVehDynObject()
   self.SetVehOrbElemObject()
   self.SetSimpleNavObject()
   self.SetsolarArrayBore()
   self.SetinstrumentBore()
   self.SethighGainBore()
   self.SetclockSynchData()

 def InitAllFSWObjects(self):
   self.SetLocalConfigData()
   self.SetCSSDecodeFSWConfig()
   self.SetIMUCommData()
   self.SetCSSWlsEstFSWConfig()
   self.SetsunSafePoint()
   self.SetMRP_Steering()
   self.SetsunSafeACS()
   self.SetattMnvrPoint()
   self.SetMRP_SteeringRWA()
   self.SetMRP_SteeringMOI()
   self.SetdvAttEffect()
   self.SetdvGuidance()
   self.SetsunPoint()
   self.SetearthPoint()
   self.SetmarsPoint()

# def AddVariableForLogging(self, VarName, LogPeriod = 0):
#   i=0
#   SplitName = VarName.split('.')
#   Subname = '.'
#   Subname = Subname.join(SplitName[1:])
#   NoDotName = ''
#   NoDotName = NoDotName.join(SplitName)
#   NoDotName = NoDotName.translate(None, '[]')
#   #if SplitName[0] in self.NameReplace:
#   #   LogName = self.NameReplace[SplitName[0]] + '.' + Subname
#   if(VarName not in self.VarLogList):
#      RefFunctionString = 'def Get' + NoDotName + '(self):\n'
#      RefFunctionString += '   return self.'+ VarName
#      exec(RefFunctionString)
#      methodHandle = eval('Get' + NoDotName)
#      self.VarLogList[VarName] = SimulationBaseClass.LogBaseClass(VarName, LogPeriod,
#         methodHandle )
#
# def AddVectorForLogging(self, VarName, VarType, StartIndex, StopIndex=0, LogPeriod=0):
#   SplitName = VarName.split('.')
#   Subname = '.'
#   Subname = Subname.join(SplitName[1:])
#   NoDotName = ''
#   NoDotName = NoDotName.join(SplitName) 
#   NoDotName = NoDotName.translate(None, '[]')
#   #LogName = self.NameReplace[SplitName[0]] + '.' + Subname
#   if(VarName in self.VarLogList):
#      return
#   if(type(eval('self.'+VarName)).__name__ == 'SwigPyObject'):
#      RefFunctionString = 'def Get' + NoDotName + '(self):\n'
#      RefFunctionString += '   return ['
#      LoopTerminate = False
#      i=0
#      while not LoopTerminate:
#         RefFunctionString += 'sim_model.' + VarType + 'Array_getitem('
#         RefFunctionString += 'self.'+VarName + ', ' + str(StartIndex + i) + '),'
#         i+=1 
#         if(i > StopIndex-StartIndex):
#            LoopTerminate = True
#   else:
#      RefFunctionString = 'def Get' + NoDotName + '(self):\n'
#      RefFunctionString += '   return ['
#      LoopTerminate = False
#      i=0
#      while not LoopTerminate:
#         RefFunctionString += 'self.'+VarName + '[' +str(StartIndex+i) +'],'
#         i+=1
#         if(i > StopIndex-StartIndex):
#            LoopTerminate = True
#   RefFunctionString = RefFunctionString[:-1] + ']'
#   exec(RefFunctionString)
#   methodHandle = eval('Get' + NoDotName)
#   self.VarLogList[VarName] = SimulationBaseClass.LogBaseClass(VarName, LogPeriod,
#      methodHandle, StopIndex - StartIndex+1)

def LoadGravFromFile(FileName, GravBody, JParamsSelect):
   csvfile = open(FileName, 'rb')
   csvreader = csv.reader(csvfile)
   FirstLine = True
   NextJindex = 0
   AllJParams = []
   for row in csvreader:
      if(FirstLine == True):
         GravBody.mu = float(row[1])
         FirstLine = False
      elif(int(row[0]) == JParamsSelect[NextJindex]):
         LocalJParam = -math.sqrt(2*JParamsSelect[NextJindex]+1)*float(row[2])
         AllJParams.append(LocalJParam)
         NextJindex += 1
         if(NextJindex >= len(JParamsSelect)):
            break
   return(AllJParams)
