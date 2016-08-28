'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
# Import some architectural stuff that we will probably always use
import sys
import os
import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# This part definitely needs work.  Need to detect Basilisk somehow.
sys.path.append(path + '/../../Basilisk/PythonModules')
sys.path.append(path + '/../../Basilisk/modules')
# Simulation base class is needed because we inherit from it
import SimulationBaseClass
import RigidBodyKinematics as rbk
import numpy as np
import macros as mc

# import regular python objects that we need
import math
import csv
import copy

# Vehicle dynamics and avionics models
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
import reactionwheel_dynamics
# import radiation_pressure
# import parseSRPLookup
import star_tracker
# FSW algorithms that we want to call
import cssComm
import alg_contain
import vehicleConfigData
import cssWlsEst
import sunSafePoint
import imuComm
import stComm
import MRP_Steering
import MRP_Feedback
import MRP_PD
import PRV_Steering
import sunSafeACS
import dvAttEffect
import dvGuidance
import attRefGen
import celestialBodyPoint
import clock_synch
import rwNullSpace
import thrustRWDesat
import attitude_ukf
import boost_communication
import inertial3D
import hillPoint
import velocityPoint
import celestialTwoBodyPoint
import inertial3DSpin
import rasterManager
import eulerRotation
import attTrackingError
import simpleDeadband
import thrForceMapping
import rwMotorTorque

import simSetupRW                 # RW simulation setup utilties
import simSetupThruster           # Thruster simulation setup utilties
import fswSetupRW

class AVSSim(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        self.modeRequest = 'None'
        self.isUsingVisualization = False

        # Processes
        self.fswProc = self.CreateNewProcess("FSWProcess")
        self.dynProc = self.CreateNewProcess("DynamicsProcess")
        self.visProc = self.CreateNewProcess("VisProcess")
        # Process message interfaces.
        self.dyn2FSWInterface = sim_model.SysInterface()
        self.fsw2DynInterface = sim_model.SysInterface()
        self.dyn2VisInterface = sim_model.SysInterface()
        self.dyn2FSWInterface.addNewInterface("DynamicsProcess", "FSWProcess")
        self.fsw2DynInterface.addNewInterface("FSWProcess", "DynamicsProcess")
        self.dyn2VisInterface.addNewInterface("DynamicsProcess", "VisProcess")
        self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
        self.fswProc.addInterfaceRef(self.fsw2DynInterface)
        self.dynProc.addInterfaceRef(self.dyn2VisInterface)

        # Process task groups.
        self.dynProc.addTask(self.CreateNewTask("SynchTask", int(5E8)), 2000)
        self.dynProc.addTask(self.CreateNewTask("DynamicsTask", int(1E8)), 1000)

        # Flight software tasks.
        self.fswProc.addTask(self.CreateNewTask("initOnlyTask", int(1E10)), 1)
        self.fswProc.addTask(self.CreateNewTask("sunSafeFSWTask", int(5E8)), 999)

        self.fswProc.addTask(self.CreateNewTask("sunPointTask", int(5E8)), 106)
        self.fswProc.addTask(self.CreateNewTask("earthPointTask", int(5E8)), 105)
        self.fswProc.addTask(self.CreateNewTask("marsPointTask", int(5E8)), 104)
        self.fswProc.addTask(self.CreateNewTask("vehicleAttMnvrFSWTask", int(5E8)), 103)

        self.fswProc.addTask(self.CreateNewTask("vehicleDVPrepFSWTask", int(5E8)), 101)
        self.fswProc.addTask(self.CreateNewTask("vehicleDVMnvrFSWTask", int(5E8)), 100)
        self.fswProc.addTask(self.CreateNewTask("RWADesatTask", int(5E8)), 102)
        self.fswProc.addTask(self.CreateNewTask("sensorProcessing", int(5E8)), 210)
        self.fswProc.addTask(self.CreateNewTask("attitudeNav", int(5E8)), 209)

        # Guidance Tasks
        self.fswProc.addTask(self.CreateNewTask("inertial3DPointTask", int(5E8)), 128)
        self.fswProc.addTask(self.CreateNewTask("hillPointTask", int(5E8)), 126)
        self.fswProc.addTask(self.CreateNewTask("velocityPointTask", int(5E8)), 125)
        self.fswProc.addTask(self.CreateNewTask("celTwoBodyPointTask", int(5E8)), 124)
        self.fswProc.addTask(self.CreateNewTask("rasterMnvrTask", int(5E8)), 118)
        self.fswProc.addTask(self.CreateNewTask("eulerRotationTask", int(5E8)), 117)
        self.fswProc.addTask(self.CreateNewTask("inertial3DSpinTask", int(5E8)), 116)

        self.fswProc.addTask(self.CreateNewTask("attitudeControlMnvrTask", int(5E8)), 110)
        self.fswProc.addTask(self.CreateNewTask("feedbackControlMnvrTask", int(5E8)), 110)
        self.fswProc.addTask(self.CreateNewTask("attitudePRVControlMnvrTask", int(5E8)), 110)

        self.fswProc.addTask(self.CreateNewTask("simpleRWControlTask", int(5E8)), 111)


        # Spacecraft configuration data module.
        self.LocalConfigData = vehicleConfigData.vehicleConfigData()

        # Simulation modules
        self.SpiceObject = spice_interface.SpiceInterface()
        self.cssConstellation = coarse_sun_sensor.CSSConstellation()
        # Schedule the first pyramid on the simulated sensor Task
        self.IMUSensor = imu_sensor.ImuSensor()
        self.ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()
        self.DVThrusterDynObject = thruster_dynamics.ThrusterDynamics()
        self.VehDynObject = six_dof_eom.SixDofEOM()
        # self.radiationPressure = radiation_pressure.RadiationPressure()
        self.VehOrbElemObject = orb_elem_convert.OrbElemConvert()
        self.SimpleNavObject = simple_nav.SimpleNav()
        self.solarArrayBore = bore_ang_calc.BoreAngCalc()
        self.highGainBore = bore_ang_calc.BoreAngCalc()
        self.instrumentBore = bore_ang_calc.BoreAngCalc()
        self.clockSynchData = clock_synch.ClockSynch()
        self.rwDynObject = reactionwheel_dynamics.ReactionWheelDynamics()
        self.trackerA = star_tracker.StarTracker()
        self.InitAllDynObjects()

        # Add simulation modules to task groups.
        self.disableTask("SynchTask")
        self.AddModelToTask("SynchTask", self.clockSynchData, None, 100)
        self.AddModelToTask("DynamicsTask", self.SpiceObject, None, 202)
        self.AddModelToTask("DynamicsTask", self.cssConstellation, None, 108)
        self.AddModelToTask("DynamicsTask", self.IMUSensor, None, 100)
        # self.AddModelToTask("DynamicsTask", self.radiationPressure, None, 303)
        self.AddModelToTask("DynamicsTask", self.ACSThrusterDynObject, None, 302)
        self.AddModelToTask("DynamicsTask", self.DVThrusterDynObject, None, 301)
        self.AddModelToTask("DynamicsTask", self.rwDynObject, None, 300)
        self.AddModelToTask("DynamicsTask", self.VehDynObject, None, 201)
        self.AddModelToTask("DynamicsTask", self.VehOrbElemObject, None, 200)
        self.AddModelToTask("DynamicsTask", self.SimpleNavObject, None, 109)
        self.AddModelToTask("DynamicsTask", self.solarArrayBore, None, 110)
        self.AddModelToTask("DynamicsTask", self.instrumentBore, None, 111)
        self.AddModelToTask("DynamicsTask", self.highGainBore, None, 112)
        self.AddModelToTask("DynamicsTask", self.trackerA, None, 113)

        # Flight software modules.
        self.VehConfigData = vehicleConfigData.VehConfigInputData()
        self.VehConfigDataWrap = alg_contain.AlgContain(self.VehConfigData,
            vehicleConfigData.Update_vehicleConfigData, vehicleConfigData.SelfInit_vehicleConfigData,
            vehicleConfigData.CrossInit_vehicleConfigData)
        self.VehConfigDataWrap.ModelTag = "vehConfigData"


        self.CSSDecodeFSWConfig = cssComm.CSSConfigData()
        self.CSSAlgWrap = self.setModelDataWrap(self.CSSDecodeFSWConfig)
        self.CSSAlgWrap.ModelTag = "cssSensorDecode"

        self.IMUCommData = imuComm.IMUConfigData()
        self.IMUCommWrap = self.setModelDataWrap(self.IMUCommData)
        self.IMUCommWrap.ModelTag = "imuSensorDecode"

        self.STCommData = stComm.STConfigData()
        self.STCommWrap = self.setModelDataWrap(self.STCommData)
        self.STCommWrap.ModelTag = "stSensorDecode"

        self.CSSWlsEstFSWConfig = cssWlsEst.CSSWLSConfig()
        self.CSSWlsWrap = self.setModelDataWrap(self.CSSWlsEstFSWConfig)
        self.CSSWlsWrap.ModelTag = "CSSWlsEst"

        self.sunSafePointData = sunSafePoint.sunSafePointConfig()
        self.sunSafePointWrap = self.setModelDataWrap(self.sunSafePointData)
        self.sunSafePointWrap.ModelTag = "sunSafePoint"

        self.MRP_SteeringSafeData = MRP_Steering.MRP_SteeringConfig()
        self.MRP_SteeringWrap = self.setModelDataWrap(self.MRP_SteeringSafeData)
        self.MRP_SteeringWrap.ModelTag = "MRP_Steering"

        self.MRP_PDSafeData = MRP_PD.MRP_PDConfig()
        self.MRP_PDSafeWrap = self.setModelDataWrap(self.MRP_PDSafeData)
        self.MRP_PDSafeWrap.ModelTag = "MRP_PD"

        self.sunSafeACSData = sunSafeACS.sunSafeACSConfig()
        self.sunSafeACSWrap = self.setModelDataWrap(self.sunSafeACSData)
        self.sunSafeACSWrap.ModelTag = "sunSafeACS"

        self.AttUKF = attitude_ukf.STInertialUKF()

        self.attMnvrPointData = attRefGen.attRefGenConfig()
        self.attMnvrPointWrap = self.setModelDataWrap(self.attMnvrPointData)
        self.attMnvrPointWrap.ModelTag = "attMnvrPoint"

        self.MRP_SteeringRWAData = MRP_Steering.MRP_SteeringConfig()
        self.MRP_SteeringRWAWrap = self.setModelDataWrap(self.MRP_SteeringRWAData)
        self.MRP_SteeringRWAWrap.ModelTag = "MRP_SteeringRWA"
        
        self.MRP_FeedbackRWAData = MRP_Feedback.MRP_FeedbackConfig()
        self.MRP_FeedbackRWAWrap = self.setModelDataWrap(self.MRP_FeedbackRWAData)
        self.MRP_FeedbackRWAWrap.ModelTag = "MRP_FeedbackRWA"

        self.PRV_SteeringRWAData = PRV_Steering.PRV_SteeringConfig()
        self.PRV_SteeringRWAWrap = self.setModelDataWrap(self.PRV_SteeringRWAData)
        self.PRV_SteeringRWAWrap.ModelTag = "PRV_SteeringRWA"

        self.MRP_SteeringMOIData = MRP_Steering.MRP_SteeringConfig()
        self.MRP_SteeringMOIWrap = self.setModelDataWrap(self.MRP_SteeringMOIData)
        self.MRP_SteeringMOIWrap.ModelTag = "MRP_SteeringMOI"

        self.dvGuidanceData = dvGuidance.dvGuidanceConfig()
        self.dvGuidanceWrap = self.setModelDataWrap(self.dvGuidanceData)
        self.dvGuidanceWrap.ModelTag = "dvGuidance"

        self.dvAttEffectData = dvAttEffect.dvAttEffectConfig()
        self.dvAttEffectWrap = self.setModelDataWrap(self.dvAttEffectData)
        self.dvAttEffectWrap.ModelTag = "dvAttEffect"

        self.sunPointData = celestialBodyPoint.celestialBodyPointConfig()
        self.sunPointWrap = self.setModelDataWrap(self.sunPointData)
        self.sunPointWrap.ModelTag = "sunPoint"

        self.earthPointData = celestialBodyPoint.celestialBodyPointConfig()
        self.earthPointWrap = self.setModelDataWrap(self.earthPointData)
        self.earthPointWrap.ModelTag = "earthPoint"

        self.marsPointData = celestialBodyPoint.celestialBodyPointConfig()
        self.marsPointWrap = self.setModelDataWrap(self.marsPointData)
        self.marsPointWrap.ModelTag = "marsPoint"

        self.RWAMappingData = dvAttEffect.dvAttEffectConfig()
        self.RWAMappingDataWrap = self.setModelDataWrap(self.RWAMappingData)
        self.RWAMappingDataWrap.ModelTag = "RWAMappingData"

        self.RWANullSpaceData = rwNullSpace.rwNullSpaceConfig()
        self.RWANullSpaceDataWrap = self.setModelDataWrap(self.RWANullSpaceData)
        self.RWANullSpaceDataWrap.ModelTag = "RWNullSpace"

        self.thrustRWADesatData = thrustRWDesat.thrustRWDesatConfig()
        self.thrustRWADesatDataWrap = self.setModelDataWrap(self.thrustRWADesatData)
        self.thrustRWADesatDataWrap.ModelTag = "thrustRWDesat"

        # Guidance flight software modules.

        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = self.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = self.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        self.velocityPointData = velocityPoint.velocityPointConfig()
        self.velocityPointWrap = self.setModelDataWrap(self.velocityPointData)
        self.velocityPointWrap.ModelTag = "velocityPoint"

        self.celTwoBodyPointData = celestialTwoBodyPoint.celestialTwoBodyPointConfig()
        self.celTwoBodyPointWrap = self.setModelDataWrap(self.celTwoBodyPointData)
        self.celTwoBodyPointWrap.ModelTag = "celTwoBodyPoint"

        self.rasterManagerData = rasterManager.rasterManagerConfig()
        self.rasterManagerWrap = self.setModelDataWrap(self.rasterManagerData)
        self.rasterManagerWrap.ModelTag = "rasterManager"

        self.eulerRotationData = eulerRotation.eulerRotationConfig()
        self.eulerRotationWrap = self.setModelDataWrap(self.eulerRotationData)
        self.eulerRotationWrap.ModelTag = "eulerRotation"

        self.inertial3DSpinData = inertial3DSpin.inertial3DSpinConfig()
        self.inertial3DSpinWrap = self.setModelDataWrap(self.inertial3DSpinData)
        self.inertial3DSpinWrap.ModelTag = "inertial3DSpin"
        
        self.attTrackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.attTrackingErrorWrap = self.setModelDataWrap(self.attTrackingErrorData)
        self.attTrackingErrorWrap.ModelTag = "attTrackingError"

        self.simpleDeadbandData = simpleDeadband.simpleDeadbandConfig()
        self.simpleDeadbandWrap = self.setModelDataWrap(self.simpleDeadbandData)
        self.simpleDeadbandWrap.ModelTag = "simpleDeadband"
        
        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = self.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
        
        self.thrForceMappingData = thrForceMapping.thrForceMappingConfig()
        self.thrForceMappingWrap = self.setModelDataWrap(self.thrForceMappingData)
        self.thrForceMappingWrap.ModelTag = "thrForceMapping"

        # Initialize flight software modules.
        self.InitAllFSWObjects()
        self.AddModelToTask("initOnlyTask", self.VehConfigDataWrap, self.VehConfigData, 1)

        # Add flight software modules to task groups.
        self.AddModelToTask("sunSafeFSWTask", self.IMUCommWrap, self.IMUCommData, 10)
        self.AddModelToTask("sunSafeFSWTask", self.CSSAlgWrap, self.CSSDecodeFSWConfig, 9)
        self.AddModelToTask("sunSafeFSWTask", self.CSSWlsWrap, self.CSSWlsEstFSWConfig, 8)
        self.AddModelToTask("sunSafeFSWTask", self.sunSafePointWrap, self.sunSafePointData, 7)
        self.AddModelToTask("sunSafeFSWTask", self.simpleDeadbandWrap, self.simpleDeadbandData, 6)
        self.AddModelToTask("sunSafeFSWTask", self.MRP_PDSafeWrap, self.MRP_PDSafeData, 5)
        # self.AddModelToTask("sunSafeFSWTask", self.MRP_SteeringWrap, self.MRP_SteeringSafeData, 5)
        self.AddModelToTask("sunSafeFSWTask", self.sunSafeACSWrap, self.sunSafeACSData, 4)

        self.AddModelToTask("sensorProcessing", self.CSSAlgWrap, self.CSSDecodeFSWConfig, 9)
        self.AddModelToTask("sensorProcessing", self.IMUCommWrap, self.IMUCommData, 10)
        self.AddModelToTask("sensorProcessing", self.STCommWrap, self.STCommData, 11)

        self.AddModelToTask("attitudeNav", self.AttUKF, None, 10)

        self.AddModelToTask("vehicleAttMnvrFSWTask", self.attMnvrPointWrap, self.attMnvrPointData, 10)
        self.AddModelToTask("vehicleAttMnvrFSWTask", self.MRP_SteeringRWAWrap, self.MRP_SteeringRWAData, 9)
        self.AddModelToTask("vehicleAttMnvrFSWTask", self.RWAMappingDataWrap, self.RWAMappingData, 8)
        self.AddModelToTask("vehicleAttMnvrFSWTask", self.RWANullSpaceDataWrap, self.RWANullSpaceData, 7)

        self.AddModelToTask("vehicleDVPrepFSWTask", self.dvGuidanceWrap, self.dvGuidanceData)

        self.AddModelToTask("vehicleDVMnvrFSWTask", self.dvGuidanceWrap, self.dvGuidanceData, 10)
        self.AddModelToTask("vehicleDVMnvrFSWTask", self.attMnvrPointWrap, self.attMnvrPointData, 9)
        self.AddModelToTask("vehicleDVMnvrFSWTask", self.MRP_SteeringMOIWrap, self.MRP_SteeringMOIData, 8)
        self.AddModelToTask("vehicleDVMnvrFSWTask", self.dvAttEffectWrap, self.dvAttEffectData, 7)

        self.AddModelToTask("sunPointTask", self.sunPointWrap, self.sunPointData)
        self.AddModelToTask("earthPointTask", self.earthPointWrap, self.earthPointData)
        self.AddModelToTask("marsPointTask", self.marsPointWrap, self.marsPointData)

        self.AddModelToTask("RWADesatTask", self.thrustRWADesatDataWrap, self.thrustRWADesatData)

        # Mapping of Guidance Models to Guidance Tasks
        self.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 20)
        self.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 20)
        self.AddModelToTask("velocityPointTask", self.velocityPointWrap, self.velocityPointData, 20)
        self.AddModelToTask("celTwoBodyPointTask", self.celTwoBodyPointWrap, self.celTwoBodyPointData, 20)
        self.AddModelToTask("eulerRotationTask", self.eulerRotationWrap, self.eulerRotationData, 19)
        self.AddModelToTask("inertial3DSpinTask", self.inertial3DSpinWrap, self.inertial3DSpinData, 19)
        self.AddModelToTask("rasterMnvrTask", self.rasterManagerWrap, self.rasterManagerData, 19)
        self.AddModelToTask("rasterMnvrTask", self.eulerRotationWrap, self.eulerRotationData, 18)
        
        self.AddModelToTask("attitudeControlMnvrTask", self.attTrackingErrorWrap, self.attTrackingErrorData, 10)
        self.AddModelToTask("attitudeControlMnvrTask", self.MRP_SteeringRWAWrap, self.MRP_SteeringRWAData, 9)
        self.AddModelToTask("attitudeControlMnvrTask", self.RWAMappingDataWrap, self.RWAMappingData, 8)
        self.AddModelToTask("attitudeControlMnvrTask", self.RWANullSpaceDataWrap, self.RWANullSpaceData, 7)
        

        self.AddModelToTask("feedbackControlMnvrTask", self.attTrackingErrorWrap, self.attTrackingErrorData, 10)
        self.AddModelToTask("feedbackControlMnvrTask", self.MRP_FeedbackRWAWrap, self.MRP_FeedbackRWAData, 9)
        self.AddModelToTask("feedbackControlMnvrTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)
        
        self.AddModelToTask("attitudePRVControlMnvrTask", self.attTrackingErrorWrap, self.attTrackingErrorData, 10)
        self.AddModelToTask("attitudePRVControlMnvrTask", self.PRV_SteeringRWAWrap, self.PRV_SteeringRWAData, 9)
        self.AddModelToTask("attitudePRVControlMnvrTask", self.RWAMappingDataWrap, self.RWAMappingData, 8)
        self.AddModelToTask("attitudePRVControlMnvrTask", self.RWANullSpaceDataWrap, self.RWANullSpaceData, 7)

        self.AddModelToTask("simpleRWControlTask", self.attTrackingErrorWrap, self.attTrackingErrorData, 10)
        self.AddModelToTask("simpleRWControlTask", self.MRP_FeedbackRWAWrap, self.MRP_FeedbackRWAData, 9)
        self.AddModelToTask("simpleRWControlTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)


        # Disable all tasks in the FSW process
        self.fswProc.disableAllTasks()

        # RW Motor Torque Event
        self.createNewEvent("initiateSimpleRWControlEvent", int(1E9), True, ["self.modeRequest == 'rwMotorTorqueControl'"],
                            ["self.fswProc.disableAllTasks()"
                             , "self.enableTask('sensorProcessing')"
                             , "self.enableTask('velocityPointTask')"
                             , "self.enableTask('simpleRWControlTask')"
                             ])
        # Guidance Events
        self.createNewEvent("initiateInertial3DPoint", int(1E9), True, ["self.modeRequest == 'inertial3DPoint'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('inertial3DPointTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])

        self.createNewEvent("initiateHillPoint", int(1E9), True, ["self.modeRequest == 'hillPoint'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('hillPointTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])

        self.createNewEvent("initiateVelocityPoint", int(1E9), True, ["self.modeRequest == 'velocityPoint'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('velocityPointTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])

        self.createNewEvent("initiateCelTwoBodyPoint", int(1E9), True, ["self.modeRequest == 'celTwoBodyPoint'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('celTwoBodyPointTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])
        
        self.createNewEvent("initiateRasterMnvr", int(1E9), True, ["self.modeRequest == 'rasterMnvr'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('inertial3DPointTask')"
                                , "self.enableTask('rasterMnvrTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])
        
        self.createNewEvent("initiateEulerRotation", int(1E9), True, ["self.modeRequest == 'eulerRotation'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('hillPointTask')"
                                , "self.enableTask('eulerRotationTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])

        self.createNewEvent("initiateInertial3DSpin", int(1E9), True, ["self.modeRequest == 'inertial3DSpin'"],
                            ["self.fswProc.disableAllTasks()"
                                , "self.enableTask('sensorProcessing')"
                                , "self.enableTask('hillPointTask')"
                                , "self.enableTask('inertial3DSpinTask')"
                                , "self.enableTask('feedbackControlMnvrTask')"
                             ])

        self.createNewEvent("initiateSafeMode", int(1E9), True, ["self.modeRequest == 'safeMode'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sunSafeFSWTask')"])

        self.createNewEvent("initiateSunPoint", int(1E9), True, ["self.modeRequest == 'sunPoint'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('sunPointTask')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.ResetTask('vehicleAttMnvrFSWTask')"])

        self.createNewEvent("initiateEarthPoint", int(1E9), True, ["self.modeRequest == 'earthPoint'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.enableTask('earthPointTask')",
                             "self.ResetTask('vehicleAttMnvrFSWTask')"])

        self.createNewEvent("initiateMarsPoint", int(1E9), True, ["self.modeRequest == 'marsPoint'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.enableTask('marsPointTask')",
                             "self.ResetTask('vehicleAttMnvrFSWTask')",
                             "self.attMnvrPointData.mnvrComplete = False",
                             "self.activateNextRaster()",
                             "self.setEventActivity('completeRaster', True)"])

        self.createNewEvent("initiateDVPrep", int(1E9), True, ["self.modeRequest == 'DVPrep'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('attitudeNav')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.enableTask('vehicleDVPrepFSWTask')",
                             "self.ResetTask('vehicleAttMnvrFSWTask')",
                             "self.setEventActivity('startDV', True)"])

        self.createNewEvent("initiateDVMnvr", int(1E9), True, ["self.modeRequest == 'DVMnvr'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('attitudeNav')",
                             "self.enableTask('vehicleDVMnvrFSWTask')",
                             "self.setEventActivity('completeDV', True)"])

        self.createNewEvent("initiateRWADesat", int(1E9), True, ["self.modeRequest == 'rwaDesat'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('attitudeNav')",
                             "self.enableTask('sunPointTask')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.enableTask('RWADesatTask')",
                             "self.ResetTask('RWADesatTask')"])

        self.createNewEvent("completeDV", int(1E8), False, ["self.dvGuidanceData.burnComplete != 0"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('sensorProcessing')",
                             "self.enableTask('attitudeNav')",
                             "self.enableTask('vehicleAttMnvrFSWTask')",
                             "self.ResetTask('vehicleAttMnvrFSWTask')",
                             "self.setEventActivity('initiateSunPoint', True)",
                             "self.modeRequest = 'sunPoint'"])

        self.createNewEvent("startDV", int(1E8), False,
                            ["self.dvGuidanceData.burnStartTime <= self.TotalSim.CurrentNanos"],
                            ["self.modeRequest = 'DVMnvr'",
                             "self.setEventActivity('initiateDVMnvr', True)"])

    def initializeVisualization(self):
        openGLIO = boost_communication.OpenGLIO()
        for planetName in self.SpiceObject.PlanetNames:
            openGLIO.addPlanetMessageName(planetName)
        idx = 0
        for thruster in self.ACSThrusterDynObject.ThrusterData:
            openGLIO.addThrusterMessageName(idx)
            idx += 1
        idxRW = 0
        for rw in self.rwDynObject.ReactionWheelData:
            openGLIO.addRwMessageName(idxRW)
            idxRW += 1
        openGLIO.setIpAddress("127.0.0.1")
        openGLIO.spiceDataPath = self.simBasePath+'/External/EphemerisData/'
        openGLIO.setUTCCalInit(self.SpiceObject.UTCCalInit)
        openGLIO.setCelestialObject(13)
        self.visProc.addTask(self.CreateNewTask("visTask", int(1E8)))
        self.AddModelToTask("visTask", openGLIO)
        self.enableTask("SynchTask")


    def InitializeSimulation(self):
        if self.isUsingVisualization:
            self.initializeVisualization()
        SimulationBaseClass.SimBaseClass.InitializeSimulation(self)
        self.dyn2FSWInterface.discoverAllMessages()
        self.fsw2DynInterface.discoverAllMessages()
        self.dyn2VisInterface.discoverAllMessages()

    #
    # Set the static spacecraft parameters
    #
    def SetLocalConfigData(self):
        self.RWAGsMatrix = []
        self.RWAJsList = []
        i = 0
        rwElAngle = 45.0 * math.pi / 180.0
        rwClockAngle = 45.0 * math.pi / 180.0
        RWAlignScale = 1.0 / 25.0
        rwClass = vehicleConfigData.RWConstellation()
        rwClass.numRW = 4
        rwPointer = vehicleConfigData.RWConfigurationElement()
        while (i < rwClass.numRW):
            self.RWAGsMatrix.extend([-math.sin(rwElAngle) * math.sin(rwClockAngle),
                                -math.sin(rwElAngle) * math.cos(rwClockAngle), -math.cos(rwElAngle)])
            self.RWAJsList.extend([100.0 / (6000.0 / 60.0 * math.pi * 2.0)])
            rwPointer.gsHat_S = [-math.sin(rwElAngle) * math.sin(rwClockAngle),-math.sin(rwElAngle) * math.cos(rwClockAngle), -math.cos(rwElAngle)]
            rwPointer.Js = 100.0 / (6000.0 / 60.0 * math.pi * 2.0)
            vehicleConfigData.RWConfigArray_setitem(rwClass.reactionWheels, i, rwPointer)
            rwClockAngle += 90.0 * math.pi / 180.0
            i += 1

        msgSizeRW = 4 + vehicleConfigData.MAX_EFF_CNT*7*8
        self.TotalSim.CreateNewMessage("FSWProcess", "rwa_config_data",
                                       msgSizeRW, 2, "RWConstellation")
        self.TotalSim.WriteMessageData("rwa_config_data", msgSizeRW, 0, rwClass)

        
        rcsClass = vehicleConfigData.ThrusterCluster()
        rcsPointer = vehicleConfigData.ThrusterPointData()
        rcsLocationData = [ \
                   [-0.86360, -0.82550,  1.79070],
                   [-0.82550, -0.86360,  1.79070],
                   [ 0.82550,  0.86360,  1.79070],
                   [ 0.86360,  0.82550,  1.79070],
                   [-0.86360, -0.82550, -1.79070],
                   [-0.82550, -0.86360, -1.79070],
                   [ 0.82550,  0.86360, -1.79070],
                   [ 0.86360,  0.82550, -1.79070] \
                   ]
        rcsDirectionData = [ \
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0] \
                        ]

        rcsClass.numThrusters = 8
        for i in range(8):
            rcsPointer.rThrust_S = rcsLocationData[i]
            rcsPointer.tHatThrust_S = rcsDirectionData[i]
            vehicleConfigData.ThrustConfigArray_setitem(rcsClass.thrusters, i, rcsPointer)

        msgSizeThrust = 4 + vehicleConfigData.MAX_EFF_CNT*6*8
        self.TotalSim.CreateNewMessage("FSWProcess", "rcs_config_data",
                                       msgSizeThrust, 2, "ThrusterCluster")
        self.TotalSim.WriteMessageData("rcs_config_data", msgSizeThrust, 0, rcsClass)



    def setRwFSWDeviceAvailability(self):
        rwAvailabilityMessage = rwMotorTorque.RWAvailabilityData()
        avail = [1, 0, 1, 0]
        rwAvailabilityMessage.wheelAvailability = avail
        msgSize = vehicleConfigData.MAX_EFF_CNT*4
        self.TotalSim.CreateNewMessage("FSWProcess", "rw_availability", msgSize, 2, "RWAvailabilityData")
        self.TotalSim.WriteMessageData("rw_availability", msgSize, 0, rwAvailabilityMessage)

    def SetSpiceObject(self):
        self.SpiceObject.ModelTag = "SpiceInterfaceData"
        self.SpiceObject.SPICEDataPath = self.simBasePath + '/External/EphemerisData/'
        self.SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
        self.SpiceObject.OutputBufferCount = 2
        self.SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars", "sun"])
        self.SpiceObject.referenceBase = "MARSIAU"

    def SetIMUSensor(self):
        def turnOffCorruption():
            rotBiasValue = 0.0
            rotNoiseStdValue = 0.0
            transBiasValue = 0.0
            transNoiseStdValue = 0.0
            return (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue)

        rotBiasValue = 0.0
        rotNoiseStdValue = 0.000001
        transBiasValue = 0.0
        transNoiseStdValue = 1.0E-6

        self.IMUSensor = imu_sensor.ImuSensor()
        self.IMUSensor.SensorPosStr = imu_sensor.DoubleVector([1.5, 0.1, 0.1])
        self.IMUSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
        self.IMUSensor.accelLSB = 2.77E-4 * 9.80665
        self.IMUSensor.gyroLSB = 8.75E-3 * math.pi / 180.0

        # Turn off corruption of IMU data
        (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue) = turnOffCorruption()

        self.IMUSensor.senRotBias = [rotBiasValue, rotBiasValue, rotBiasValue]
        self.IMUSensor.senRotNoiseStd = [rotNoiseStdValue, rotNoiseStdValue, rotNoiseStdValue]
        self.IMUSensor.senTransBias = [transBiasValue, transBiasValue, transBiasValue]
        self.IMUSensor.senTransNoiseStd = [transNoiseStdValue, transNoiseStdValue, transNoiseStdValue]


    def SetReactionWheelDynObject(self):
        rwElAngle = 45.0 * math.pi / 180.0
        rwClockAngle = 45.0 * math.pi / 180.0
        rwType = 'Honeywell_HR16'
        modelTag = "ReactionWheels"
        self.rwDynObject.inputVehProps = "spacecraft_mass_props"

        simSetupRW.clearSetup()
        simSetupRW.options.useRWfriction = False
        simSetupRW.options.useMinTorque = False
        simSetupRW.options.maxMomentum = 100    # Nms
        simSetupRW.create(rwType,
            [-math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
             -math.cos(rwElAngle)],  # gsHat_S
            0.0,  # Omega [RPM]
            [0.8, 0.8, 1.79070] #r_S [m]
        )
        rwClockAngle += 90.0 * math.pi / 180.0
        simSetupRW.create(
            rwType,
            [-math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
             -math.cos(rwElAngle)],  # gsHat_S
            0.0,  # Omega [RPM]
            [0.8, -0.8, 1.79070]  # r_S [m]
        )

        rwClockAngle += 90.0 * math.pi / 180.0
        simSetupRW.create(
            rwType,
            [-math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
             -math.cos(rwElAngle)],  # gsHat_S
            0.0,  # Omega [RPM]
            [-0.8, -0.8, 1.79070]  # r_S [m]
        )

        rwClockAngle += 90.0 * math.pi / 180.0
        simSetupRW.create(
            rwType,
            [-math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
             -math.cos(rwElAngle)],  # gsHat_S
            0.0,  # Omega [RPM]
            [-0.8, 0.8, 1.79070]  # r_S [m]
        )
        simSetupRW.addToSpacecraft(modelTag, self.rwDynObject, self.VehDynObject)

    def SetACSThrusterDynObject(self):
        self.ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
        self.ACSThrusterDynObject.InputCmds = "acs_thruster_cmds"

        simSetupThruster.clearSetup()
        thrusterType = 'MOOG_Monarc_1'
        simSetupThruster.create(
            thrusterType,
            [-0.86360, -0.82550, 1.79070],  # location in S frame
            [1.0, 0.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [-0.82550, -0.86360, 1.79070],  # location in S frame
            [0.0, 1.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [0.82550, 0.86360, 1.79070],  # location in S frame
            [0.0, -1.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [0.86360, 0.82550, 1.79070],  # location in S frame
            [-1.0, 0.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [-0.86360, -0.82550, -1.79070],  # location in S frame
            [1.0, 0.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [-0.82550, -0.86360, -1.79070],  # location in S frame
            [0.0, 1.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [0.82550, 0.86360, -1.79070],  # location in S frame
            [0.0, -1.0, 0.0]  # direction in S frame
        )
        simSetupThruster.create(
            thrusterType,
            [0.86360, 0.82550, -1.79070],  # location in S frame
            [-1.0, 0.0, 0.0]  # direction in S frame
        )
        simSetupThruster.addToSpacecraft(self.ACSThrusterDynObject.ModelTag,
                                         self.ACSThrusterDynObject,
                                         self.VehDynObject)
        ACSpropCM = [0.0, 0.0, 1.2]
        ACSpropMass = 40  # Made up!!!!
        ACSpropRadius = 46.0 / 2.0 / 3.2808399 / 12.0
        sphereInerita = 2.0 / 5.0 * ACSpropMass * ACSpropRadius * ACSpropRadius
        ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
        self.ACSThrusterDynObject.objProps.Mass = ACSpropMass
        self.ACSThrusterDynObject.objProps.CoM = ACSpropCM
        self.ACSThrusterDynObject.objProps.InertiaTensor = ACSInertia
        self.ACSThrusterDynObject.inputProperties = "spacecraft_mass_props"

    def SetDVThrusterDynObject(self):
        self.DVThrusterDynObject.ModelTag = "DVThrusterDynamics"
        self.DVThrusterDynObject.InputCmds = "dv_thruster_cmds"
        self.DVThrusterDynObject.inputProperties = "spacecraft_mass_props"

        simSetupThruster.clearSetup()
        thrusterType = 'MOOG_Monarc_90HT'
        dvRadius = 0.4
        maxThrust = 111.0
        minOnTime = 0.020
        i = 0
        angleInc = math.radians(60.0)
        while i < 6:
            simSetupThruster.create(
                thrusterType,
                [dvRadius * math.cos(i * angleInc), dvRadius * math.sin(i * angleInc), 0.0],  # location in S frame
                [0.0, 0.0, 1.0]  # direction in S frame
            )
            simSetupThruster.thrusterList[i].MaxThrust = maxThrust
            simSetupThruster.thrusterList[i].MinOnTime = minOnTime
            i += 1

            simSetupThruster.addToSpacecraft(self.DVThrusterDynObject.ModelTag,
                                             self.DVThrusterDynObject,
                                             self.VehDynObject)

        DVpropCM = [0.0, 0.0, 1.0]
        DVpropMass = 812.3 - 40  # The 40 comes from the made up ACS number!
        DVpropRadius = 46.0 / 2.0 / 3.2808399 / 12.0
        sphereInerita = 2.0 / 5.0 * DVpropMass * DVpropRadius * DVpropRadius
        DVInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
        self.DVThrusterDynObject.objProps.Mass = DVpropMass
        self.DVThrusterDynObject.objProps.CoM = DVpropCM
        self.DVThrusterDynObject.objProps.InertiaTensor = DVInertia

    def InitCSSHeads(self):
        def turnOffSensorCorruption():
            CSSNoiseStd = 0.0
            CSSNoiseBias = 0.0
            CSSKellyFactor = 0.0
            return (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor)

        # Note the re-use between different instances of the modules.
        # Handy but not required.
        CSSNoiseStd = 0.001  # Standard deviation of white noise
        CSSNoiseBias = 0.0  # Constant bias
        CSSKellyFactor = 0.1  # Used to get the curve shape correct for output
        CSSscaleFactor = 500.0E-6  # Scale factor (500 mu-amps) for max measurement
        CSSFOV = 90.0 * math.pi / 180.0  # 90 degree field of view

        # Turn off corruption of CSS data
        (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor) = turnOffSensorCorruption()

        # Platform 1 is forward, platform 2 is back notionally
        CSSPlatform1YPR = [-math.pi / 2.0, -math.pi / 4.0, -math.pi / 2.0]
        CSSPlatform2YPR = [0.0, -math.pi / 2.0, 0.0]

        # Initialize one sensor by hand and then init the rest off of it
        self.cssConstellation.ModelTag = "CSSConstelation"
        self.cssConstellation.outputConstellationMessage = "css_sensors_data"
        CSSPyramid1HeadA = coarse_sun_sensor.CoarseSunSensor()
        CSSPyramid1HeadA.ModelTag = "CSSPyramid1HeadA"
        CSSPyramid1HeadA.SenBias = CSSNoiseBias
        CSSPyramid1HeadA.SenNoiseStd = CSSNoiseStd
        CSSPyramid1HeadA.setStructureToPlatformDCM(CSSPlatform1YPR[0],
                                                        CSSPlatform1YPR[1], CSSPlatform1YPR[2])
        CSSPyramid1HeadA.scaleFactor = CSSscaleFactor
        CSSPyramid1HeadA.fov = CSSFOV
        CSSPyramid1HeadA.KellyFactor = CSSKellyFactor
        CSSPyramid1HeadA.OutputDataMsg = "coarse_sun_data_pyramid1_headA"
        CSSPyramid1HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadB.ModelTag = "CSSPyramid1HeadB"
        CSSPyramid1HeadB.OutputDataMsg = "coarse_sun_data_pyramid1_headB"
        CSSPyramid1HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadC.ModelTag = "CSSPyramid1HeadC"
        CSSPyramid1HeadC.OutputDataMsg = "coarse_sun_data_pyramid1_headC"
        CSSPyramid1HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadD.ModelTag = "CSSPyramid1HeadD"
        CSSPyramid1HeadD.OutputDataMsg = "coarse_sun_data_pyramid1_headD"

        # Set up the sun sensor orientation information
        # Maybe we should add the method call to the SelfInit of the CSS module
        CSSPyramid1HeadA.theta = 0.0
        CSSPyramid1HeadA.phi = 45.0 * math.pi / 180.0
        CSSPyramid1HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid1HeadB.theta = 90.0 * math.pi / 180.0
        CSSPyramid1HeadB.phi = 45.0 * math.pi / 180.0
        CSSPyramid1HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid1HeadC.theta = 180.0 * math.pi / 180.0
        CSSPyramid1HeadC.phi = 45.0 * math.pi / 180.0
        CSSPyramid1HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid1HeadD.theta = 270.0 * math.pi / 180.0
        CSSPyramid1HeadD.phi = 45 * math.pi / 180.0
        CSSPyramid1HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

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
        
        self.cssConstellation.sensorList = coarse_sun_sensor.CSSVector([CSSPyramid1HeadA, CSSPyramid1HeadB, CSSPyramid1HeadC, CSSPyramid1HeadD,
            CSSPyramid2HeadA, CSSPyramid2HeadB, CSSPyramid2HeadC, CSSPyramid2HeadD])

    def SetVehDynObject(self):
        self.SunGravBody = six_dof_eom.GravityBodyData()
        self.SunGravBody.BodyMsgName = "sun_planet_data"
        self.SunGravBody.outputMsgName = "sun_display_frame_data"
        self.SunGravBody.mu = 1.32712440018E20  # meters!
        self.SunGravBody.IsCentralBody = True
        self.SunGravBody.IsDisplayBody = True
        self.SunGravBody.UseJParams = False

        JParamsSelect = [2, 3, 4, 5, 6]
        EarthGravFile = self.simBasePath + '/External/LocalGravData/GGM03S.txt'
        MarsGravFile = self.simBasePath + '/External/LocalGravData/GGM2BData.txt'

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
        self.VehDynObject.PositionInit = six_dof_eom.DoubleVector(
            [2.342211275644610E+07 * 1000.0, -1.503236698659483E+08 * 1000.0, -1.786319594218582E+04 * 1000.0])
        self.VehDynObject.VelocityInit = six_dof_eom.DoubleVector(
            [2.896852053342327E+01 * 1000.0, 4.386175246767674E+00 * 1000.0, -3.469168621992313E-04 * 1000.0])
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
        # Add the three gravity bodies in to the simulation
        self.VehDynObject.AddGravityBody(self.SunGravBody)
        self.VehDynObject.AddGravityBody(self.EarthGravBody)
        self.VehDynObject.AddGravityBody(self.MarsGravBody)
        # Here is where the thruster dynamics are attached/scheduled to the overall
        # vehicle dynamics.  Anything that is going to impact the dynamics of the
        # vehicle
        # should be one of these body effectors I think.
        self.VehDynObject.addThrusterSet(self.ACSThrusterDynObject)
        self.VehDynObject.addThrusterSet(self.DVThrusterDynObject)
        # self.VehDynObject.addBodyEffector(self.radiationPressure)
        self.VehDynObject.useTranslation = True
        self.VehDynObject.useRotation = True

    def setRadiationPressure(self):
        self.radiationPressure.ModelTag = "RadiationPressureDynamics"
        self.radiationPressure.m_srpDataPath = self.simBasePath + 'External/RadiationPressureData/lookup_EMM_boxAndWing.txt'
        self.radiationPressure.setUseCannonballModel(False)
        self.radiationPressure.m_area = 4.0  # m^2
        self.radiationPressure.m_coeffReflection = 1.2  # no units

        srpParser = parseSRPLookup()
        srpParser.parseXML("SRPLookupTable.xml")
        self.radiationPressure.setLookupForceVecs(srpParser.forceBLookup)

    def SetVehOrbElemObject(self):
        self.VehOrbElemObject.ModelTag = "VehicleOrbitalElements"
        self.VehOrbElemObject.mu = self.SunGravBody.mu

    def SetsolarArrayBore(self):
        self.solarArrayBore.ModelTag = "solarArrayBoresight"
        self.solarArrayBore.StateString = "inertial_state_output"
        self.solarArrayBore.celBodyString = "sun_display_frame_data"
        self.solarArrayBore.OutputDataString = "solar_array_sun_bore"
        self.solarArrayBore.strBoreVec = [0.0, 0.0, 1.0]

    def SethighGainBore(self):
        self.highGainBore.ModelTag = "highGainBoresight"
        self.highGainBore.StateString = "inertial_state_output"
        self.highGainBore.celBodyString = "earth_display_frame_data"
        self.highGainBore.OutputDataString = "high_gain_earth_bore"
        angSin = math.sin(23.0 * math.pi / 180.0)
        angCos = math.cos(23.0 * math.pi / 180.0)
        self.highGainBore.strBoreVec = [0.0, -angSin, angCos]

    def SetinstrumentBore(self):
        self.instrumentBore.ModelTag = "instrumentBoresight"
        self.instrumentBore.StateString = "inertial_state_output"
        self.instrumentBore.celBodyString = "mars_display_frame_data"
        self.instrumentBore.OutputDataString = "instrument_mars_bore"
        self.instrumentBore.strBoreVec = [0.0, 1.0, 0.0]

    def SetSimpleNavObject(self):
        def turnOffCorruption():
            PMatrix = [0.0] * 18 * 18
            PMatrix[0 * 18 + 0] = PMatrix[1 * 18 + 1] = PMatrix[2 * 18 + 2] = 0.0  # Position
            PMatrix[3 * 18 + 3] = PMatrix[4 * 18 + 4] = PMatrix[5 * 18 + 5] = 0.0  # Velocity
            PMatrix[6 * 18 + 6] = PMatrix[7 * 18 + 7] = PMatrix[8 * 18 + 8] = 0.0 * math.pi / 180.0  # Attitude (sigma!)
            PMatrix[9 * 18 + 9] = PMatrix[10 * 18 + 10] = PMatrix[11 * 18 + 11] = 0.0 * math.pi / 180.0  # Attitude rate
            PMatrix[12 * 18 + 12] = PMatrix[13 * 18 + 13] = PMatrix[14 * 18 + 14] = 0.0 * math.pi / 180.0  # Sun vector
            PMatrix[15 * 18 + 15] = PMatrix[16 * 18 + 16] = PMatrix[17 * 18 + 17] = 0.0  # Accumulated DV
            errorBounds = [0.0, 0.0, 0.0,  # Position
                        0.0, 0.0, 0.0,  # Velocity
                        0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0,  # Attitude
                        0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0,  # Attitude Rate
                        0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0, 0.0 * math.pi / 180.0,  # Sun vector
                        0.0, 0.0, 0.0]  # Accumulated DV
            return (PMatrix, errorBounds)

        self.SimpleNavObject.ModelTag = "SimpleNavigation"
        PMatrix = [0.0] * 18 * 18
        PMatrix[0 * 18 + 0] = PMatrix[1 * 18 + 1] = PMatrix[2 * 18 + 2] = 10.0  # Position
        PMatrix[3 * 18 + 3] = PMatrix[4 * 18 + 4] = PMatrix[5 * 18 + 5] = 0.05  # Velocity
        PMatrix[6 * 18 + 6] = PMatrix[7 * 18 + 7] = PMatrix[
            8 * 18 + 8] = 1.0 / 3600.0 * math.pi / 180.0  # Attitude (sigma!)
        PMatrix[9 * 18 + 9] = PMatrix[10 * 18 + 10] = PMatrix[11 * 18 + 11] = 0.0001 * math.pi / 180.0  # Attitude rate
        PMatrix[12 * 18 + 12] = PMatrix[13 * 18 + 13] = PMatrix[14 * 18 + 14] = 0.1 * math.pi / 180.0  # Sun vector
        PMatrix[15 * 18 + 15] = PMatrix[16 * 18 + 16] = PMatrix[17 * 18 + 17] = 0.003  # Accumulated DV
        errorBounds = [1000.0, 1000.0, 1000.0,  # Position
                       1.0, 1.0, 1.0,  # Velocity
                       1.6E-2 * math.pi / 180.0, 1.6E-2 * math.pi / 180.0, 1.6E-2 * math.pi / 180.0,  # Attitude
                       0.0004 * math.pi / 180.0, 0.0004 * math.pi / 180.0, 0.0004 * math.pi / 180.0,  # Attitude Rate
                       5.0 * math.pi / 180.0, 5.0 * math.pi / 180.0, 5.0 * math.pi / 180.0,  # Sun vector
                       0.053, 0.053, 0.053]  # Accumulated DV
        # Turn off FSW corruption of navigation data
        (PMatrix, errorBounds) = turnOffCorruption()
        self.SimpleNavObject.walkBounds = sim_model.DoubleVector(errorBounds)
        self.SimpleNavObject.PMatrix = sim_model.DoubleVector(PMatrix)
        self.SimpleNavObject.crossTrans = True
        self.SimpleNavObject.crossAtt = False

    def SetStarTrackerData(self):
        def turnOffCorruption():
            PMatrix = [0.0] * 3 * 3
            PMatrix[0 * 3 + 0] = PMatrix[1 * 3 + 1] = PMatrix[2 * 3 + 2] = 0.0
            errorBounds = [0.0 / 3600 * math.pi / 180.0] * 3
            return (PMatrix, errorBounds)

        self.trackerA.ModelTag = "StarTrackerA"
        PMatrix = [0.0] * 3 * 3
        PMatrix[0 * 3 + 0] = PMatrix[1 * 3 + 1] = PMatrix[2 * 3 + 2] = 0.5 / 3600.0 * math.pi / 180.0  # 20 arcsecs?+
        errorBounds = [5.0 / 3600 * math.pi / 180.0] * 3
        # Turn off FSW corruption of star tracker data
        (PMatrix, errorBounds) = turnOffCorruption()
        self.trackerA.walkBounds = sim_model.DoubleVector(errorBounds)
        self.trackerA.PMatrix = sim_model.DoubleVector(PMatrix)

    def setClockSynchData(self):
        self.clockSynchData.ModelTag = "ClockSynchModel"
        self.clockSynchData.accelFactor = 1.0
        self.clockSynchData.clockOutputName = "clock_synch_data"
        self.clockSynchData.outputBufferCount = 2
    
    def SetVehicleConfigData(self):
        BS = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.VehConfigData.BS = BS
        Inertia = [700.0, 0.0, 0.0, 0.0, 700.0, 0.0, 0.0, 0.0, 800]  # kg * m^2
        self.VehConfigData.ISCPntB_S = Inertia
        CoM = [0.0, 0.0, 1.0]
        self.VehConfigData.CoM_S = CoM
        self.VehConfigData.outputPropsName = "adcs_config_data"

    def SetCSSDecodeFSWConfig(self):
        self.CSSDecodeFSWConfig.NumSensors = 8
        self.CSSDecodeFSWConfig.MaxSensorValue = 500E-6
        self.CSSDecodeFSWConfig.OutputDataName = "css_data_aggregate"
        ChebyList = [-1.734963346951471e+06, 3.294117146099591e+06,
                     -2.816333294617512e+06, 2.163709942144332e+06,
                     -1.488025993860025e+06, 9.107359382775769e+05,
                     -4.919712500291216e+05, 2.318436583511218e+05,
                     -9.376105045529010e+04, 3.177536873430168e+04,
                     -8.704033370738143e+03, 1.816188108176300e+03,
                     -2.581556805090373e+02, 1.888418924282780e+01]
        self.CSSDecodeFSWConfig.ChebyCount = len(ChebyList)
        self.CSSDecodeFSWConfig.KellyCheby = ChebyList
        self.CSSDecodeFSWConfig.SensorListName = "css_sensors_data"

    def SetIMUCommData(self):
        self.IMUCommData.InputDataName = "imu_meas_data"
        self.IMUCommData.InputPropsName = "adcs_config_data"
        self.IMUCommData.OutputDataName = "parsed_imu_data"
        platform2str = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.IMUCommData.platform2StrDCM = platform2str

    def SetSTCommData(self):
        self.STCommData.InputDataName = "star_tracker_state"
        self.STCommData.InputPropsName = "adcs_config_data"
        self.STCommData.OutputDataName = "parsed_st_data"
        platform2str = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.STCommData.T_StrPlatform = platform2str

    def SetCSSWlsEstFSWConfig(self):
        self.CSSWlsEstFSWConfig.InputDataName = "css_data_aggregate"
        self.CSSWlsEstFSWConfig.OutputDataName = "css_wls_est"
        self.CSSWlsEstFSWConfig.InputPropsName = "adcs_config_data"
        self.CSSWlsEstFSWConfig.UseWeights = True
        self.CSSWlsEstFSWConfig.SensorUseThresh = 0.1

        CSSOrientationList = [[0.70710678118654746, -0.5, 0.5],
                              [0.70710678118654746, -0.5, -0.5],
                              [0.70710678118654746, 0.5, -0.5],
                              [0.70710678118654746, 0.5, 0.5],
                              [-0.70710678118654746, 0, 0.70710678118654757],
                              [-0.70710678118654746, 0.70710678118654757, 0.0],
                              [-0.70710678118654746, 0, -0.70710678118654757],
                              [-0.70710678118654746, -0.70710678118654757, 0.0], ]
        i = 0
        totalCSSList = []
        for CSSHat in CSSOrientationList:
            CSSConfigElement = cssWlsEst.SingleCSSConfig()
            CSSConfigElement.CBias = 1.0
            CSSConfigElement.cssNoiseStd = 0.2
            CSSConfigElement.nHatStr = CSSHat
            totalCSSList.append(CSSConfigElement)
            i += 1
        
        self.CSSWlsEstFSWConfig.CSSData = totalCSSList

    def SetsunSafePoint(self):
        self.sunSafePointData.outputDataName = "sun_safe_att_err"
        self.sunSafePointData.inputSunVecName = "css_wls_est"
        self.sunSafePointData.inputIMUDataName = "parsed_imu_data"
        self.sunSafePointData.minUnitMag = 0.95
        self.sunSafePointData.sHatBdyCmd = [0.0, 0.0, 1.0]
    
    def SetMRP_Steering(self):
        self.MRP_SteeringSafeData.omega_max = 0.4 * (math.pi / 180.)
        self.MRP_SteeringSafeData.K1 = 0.05
        self.MRP_SteeringSafeData.K3 = 1.0  # rad/sec
        self.MRP_SteeringSafeData.P = 150.0  # N*m*sec
        self.MRP_SteeringSafeData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.MRP_SteeringSafeData.integralLimit = 0.15  # rad
        #self.MRP_SteeringSafeData.inputGuidName = "sun_safe_att_err"
        self.MRP_SteeringSafeData.inputGuidName = "db_att_guid_out"
        self.MRP_SteeringSafeData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_SteeringSafeData.outputDataName = "controlTorqueRaw"

    def SetMRP_PD(self):
        self.MRP_PDSafeData.K = 4.5
        self.MRP_PDSafeData.P = 150.0  # N*m*sec
        #self.MRP_PD_SafeData.inputGuidName = "sun_safe_att_err"
        self.MRP_PDSafeData.inputGuidName = "db_att_guid_out"
        self.MRP_PDSafeData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_PDSafeData.outputDataName = "controlTorqueRaw"
        
    def setSimpleDeadband(self):
        self.simpleDeadbandData.inputGuidName = "sun_safe_att_err"
        #self.simpleDeadbandData.inputGuidName = "nom_att_guid_out"
        self.simpleDeadbandData.outputDataName = "db_att_guid_out"
        self.simpleDeadbandData.innerAttThresh = 4.0 * (math.pi / 180.)
        self.simpleDeadbandData.outerAttThresh = 17.5 * (math.pi / 180.)
        self.simpleDeadbandData.innerRateThresh = 0.1 * (math.pi / 180.)
        self.simpleDeadbandData.outerRateThresh = 0.1 * (math.pi / 180.)

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
        self.sunSafeACSData.thrData.thrOnMap = onTimeMap

    def SetattMnvrPoint(self):
        self.attMnvrPointData.inputNavStateName = "simple_att_nav_output"
        self.attMnvrPointData.inputAttCmdName = "att_cmd_output"
        self.attMnvrPointData.outputDataName = "nom_att_guid_out"
        self.attMnvrPointData.outputRefName = "att_ref_output_gen"
        self.attMnvrPointData.zeroAngleTol = 1.0 * math.pi / 180.0
        self.attMnvrPointData.mnvrActive = 0
        self.attMnvrPointData.totalMnvrTime = 1000.0
        self.attMnvrPointData.propagateReference = 1

    # Init of Guidance Modules
    def setInertial3D(self):
        self.inertial3DData.outputDataName = "att_ref_output_stage1"
        sigma_R0N = [0., 0., 0.]
        self.inertial3DData.sigma_R0N = sigma_R0N

    def setHillPoint(self):
        self.hillPointData.inputNavDataName = "simple_trans_nav_output"
        self.hillPointData.inputCelMessName = "mars_display_frame_data"
        self.hillPointData.outputDataName = "att_ref_output_stage1"

    def setVelocityPoint(self):
        self.velocityPointData.inputNavDataName = "simple_trans_nav_output"
        self.velocityPointData.inputCelMessName = "mars_display_frame_data"
        self.velocityPointData.outputDataName = "att_ref_output"
        self.velocityPointData.mu = self.SunGravBody.mu

    def setCelTwoBodyPoint(self):
        self.celTwoBodyPointData.inputNavDataName = "simple_trans_nav_output"
        self.celTwoBodyPointData.inputCelMessName = "mars_display_frame_data"
        #self.celTwoBodyPointData.inputSecMessName = "sun_display_frame_data"
        self.celTwoBodyPointData.outputDataName = "att_ref_output"
        self.celTwoBodyPointData.singularityThresh = 1.0 * mc.D2R

    def setRasterManager(self):
        self.rasterManagerData.outputEulerSetName = "euler_angle_set"
        self.rasterManagerData.outputEulerRatesName = "euler_angle_rates"

        def crossingRaster(alpha, offAlpha, totalMnvrTime):
            t_raster = totalMnvrTime / 6
            alphaDot = 2.0 * alpha / t_raster
            t_offset = offAlpha / alphaDot
            angleSetList = [
                alpha + offAlpha, 0.0, 0.0,
                -alpha - offAlpha, -alpha - offAlpha, 0.0,
                alpha + offAlpha, -alpha - offAlpha, 0.0,
                0.0, alpha + offAlpha, 0.0,
            ]
            angleRatesList = [
                -alphaDot, 0.0, 0.0
                , alphaDot, alphaDot, 0.0
                , -alphaDot, alphaDot, 0.0
                , 0.0, -alphaDot, 0.0
            ]
            rasterTimeList = [
                t_raster + t_offset, t_raster + t_offset, t_raster + t_offset, t_raster + t_offset
            ]
            return (angleSetList, angleRatesList, rasterTimeList)


        alpha = 8.0 * math.pi / 180.0
        offAlpha = 0.5 * alpha
        totalGuidSimTime = 2 * 60 * 20 * 4
        (angleSetList, angleRatesList, rasterTimeList) = crossingRaster(alpha, offAlpha, totalGuidSimTime)
        self.rasterManagerData.scanningAngles = angleSetList
        self.rasterManagerData.scanningRates = angleRatesList
        self.rasterManagerData.rasterTimes = rasterTimeList
        self.rasterManagerData.numRasters = len(rasterTimeList)

    def setInertial3DSpin(self):
        self.inertial3DSpinData.inputRefName = "att_ref_output_stage1"
        self.inertial3DSpinData.outputDataName = "att_ref_output"
        omega_RN_N = np.array([0.2, 0.2, 0.4]) * mc.D2R
        self.inertial3DSpinData.omega_spin = omega_RN_N

    def setEulerRotation(self):
        self.eulerRotationData.inputRefName = "att_ref_output_stage1"
        self.eulerRotationData.outputDataName = "att_ref_output"
        self.eulerRotationData.outputEulerSetName = "euler_set_output"
        self.eulerRotationData.outputEulerRatesName = "euler_rates_output"

    def setAttTrackingError(self):
        self.attTrackingErrorData.inputRefName = "att_ref_output"
        self.attTrackingErrorData.inputNavName = "simple_att_nav_output"
        self.attTrackingErrorData.outputDataName = "nom_att_guid_out"
        R0R = np.identity(3) # DCM from s/c body reference to body-fixed reference (offset)
        sigma_R0R = rbk.C2MRP(R0R)
        self.attTrackingErrorData.sigma_R0R = sigma_R0R


    def SetMRP_SteeringRWA(self):
        self.MRP_SteeringRWAData.K1 = 0.3  # rad/sec
        self.MRP_SteeringRWAData.K3 = 1.0  # rad/sec
        self.MRP_SteeringRWAData.omega_max = 1.5 * (math.pi / 180.)  # rad/sec
        self.MRP_SteeringRWAData.P = 150.0  # N*m*sec
        self.MRP_SteeringRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.MRP_SteeringRWAData.integralLimit = 0.0  # rad

        self.MRP_SteeringRWAData.inputGuidName = "nom_att_guid_out"
        self.MRP_SteeringRWAData.inputRWConfigData = "rwa_config_data"
        self.MRP_SteeringRWAData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_SteeringRWAData.outputDataName = "controlTorqueRaw"
        self.MRP_SteeringRWAData.inputRWSpeedsName = "reactionwheel_output_states"

    def SetMRP_FeedbackRWA(self):
        self.MRP_FeedbackRWAData.K = 1.  # rad/sec
        self.MRP_FeedbackRWAData.P = 3.  # N*m*sec
        self.MRP_FeedbackRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.MRP_FeedbackRWAData.integralLimit = 0.0  # rad

        self.MRP_FeedbackRWAData.inputGuidName = "nom_att_guid_out"
        self.MRP_FeedbackRWAData.inputRWConfigData = "rwa_config_data"
        self.MRP_FeedbackRWAData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_FeedbackRWAData.outputDataName = "controlTorqueRaw"
        self.MRP_FeedbackRWAData.inputRWSpeedsName = "reactionwheel_output_states"

    def SetPRV_SteeringRWA(self):
        self.PRV_SteeringRWAData.K1 = 0.3  # rad/sec
        self.PRV_SteeringRWAData.K3 = 1.0  # rad/sec
        self.PRV_SteeringRWAData.omega_max = 1.5 * (math.pi / 180.)  # rad/sec
        self.PRV_SteeringRWAData.P = 150.0  # N*m*sec
        self.PRV_SteeringRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.PRV_SteeringRWAData.integralLimit = 0.0  # rad

        self.PRV_SteeringRWAData.inputGuidName = "nom_att_guid_out"
        self.PRV_SteeringRWAData.inputRWConfigData = "rwa_config_data"
        self.PRV_SteeringRWAData.inputVehicleConfigDataName = "adcs_config_data"
        self.PRV_SteeringRWAData.outputDataName = "controlTorqueRaw"
        self.PRV_SteeringRWAData.inputRWSpeedsName = "reactionwheel_output_states"

    def SetMRP_SteeringMOI(self):
        self.MRP_SteeringMOIData.K1 = 0.5  # rad/sec
        self.MRP_SteeringMOIData.K3 = 3.0  # rad/sec
        self.MRP_SteeringMOIData.omega_max = 1.5 * (math.pi / 180.)  # rad/sec
        self.MRP_SteeringMOIData.P = 100.0  # N*m*sec
        self.MRP_SteeringMOIData.Ki = 11.7  # N*m - negative values turn off the integral feedback
        self.MRP_SteeringMOIData.integralLimit = 0.5  # rad
        self.MRP_SteeringMOIData.inputGuidName = "nom_att_guid_out"
        self.MRP_SteeringMOIData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_SteeringMOIData.outputDataName = "controlTorqueRaw"

    def SetdvAttEffect(self):
        self.dvAttEffectData.inputControlName = "controlTorqueRaw"
        self.dvAttEffectData.numThrGroups = 2
        totalThrGroup = []
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
        newThrGroup.thrOnMap = onTimeMap

        totalThrGroup.append(newThrGroup)
        
        newThrGroup = dvAttEffect.ThrustGroupData()
        newThrGroup.numEffectors = 6
        newThrGroup.maxNumCmds = 6
        newThrGroup.nomThrustOn = 0.52

        newThrGroup.outputDataName = "dv_thruster_cmds"
        matMult = 0.7
        onTimeMap = [0.0, -0.1 * matMult, 0.0,
                     0.0866 * matMult, -0.05 * matMult, 0.0,
                     0.0866 * matMult, 0.05 * matMult, 0.0,
                     0.0, 0.1 * matMult, 0.0,
                     -0.0866 * matMult, 0.05 * matMult, 0.0,
                     -0.0866 * matMult, -0.05 * matMult, 0.0]
        newThrGroup.thrOnMap = onTimeMap
        
        totalThrGroup.append(newThrGroup)

        self.dvAttEffectData.thrGroups = totalThrGroup

    def SetRWAMappingData(self):
        self.RWAMappingData.inputControlName = "controlTorqueRaw"
        self.RWAMappingData.numThrGroups = 1
        newThrGroup = dvAttEffect.ThrustGroupData()
        #newThrGroup.outputDataName = "reactionwheel_cmds_raw"
        newThrGroup.outputDataName = "reactionwheel_cmds"
        newThrGroup.minThrustRequest = -10.0
        newThrGroup.numEffectors = 4
        newThrGroup.maxNumCmds = 4
        onTimeMap = []
        for i in range(4):
            for j in range(3):
                onTimeMap.append(-self.RWAGsMatrix[i*3+j])
        newThrGroup.thrOnMap = onTimeMap
        self.RWAMappingData.thrGroups = [newThrGroup]

    def SetRWMotorTorque(self):
        controlAxes_B = [
            1.0, 0.0, 0.0
            , 0.0, 1.0, 0.0
            , 0.0, 0.0, 1.0
        ]
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.inputVehControlName = "controlTorqueRaw"
        self.rwMotorTorqueData.inputRWConfigDataName = "rwa_config_data"
        self.rwMotorTorqueData.inputVehicleConfigDataName = "adcs_config_data"
        #self.rwMotorTorqueData.outputDataName = "reactionwheel_cmds_raw"
        self.rwMotorTorqueData.outputDataName = "reactionwheel_cmds"
        self.rwMotorTorqueData.inputRWsAvailDataName = "rw_availability"

    def SetRWANullSpaceData(self):
        self.RWANullSpaceData.inputRWCommands = "reactionwheel_cmds_raw"
        self.RWANullSpaceData.inputRWSpeeds = "reactionwheel_output_states"
        self.RWANullSpaceData.outputControlName = "reactionwheel_cmds"
        self.RWANullSpaceData.inputRWConfigData = "rwa_config_data"
        self.RWANullSpaceData.OmegaGain = 0.002

    def SetdvGuidance(self):
        self.dvGuidanceData.outputDataName = "att_cmd_output"
        self.dvGuidanceData.inputNavDataName = "simple_trans_nav_output"
        self.dvGuidanceData.inputMassPropName = "adcs_config_data"
        self.dvGuidanceData.inputBurnDataName = "vehicle_dv_cmd"
        desiredBurnDir = [1.0, 0.0, 0.0]
        desiredOffAxis = [0.0, 1.0, 0.0]
        Tburn2Body = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]

    def SetsunPoint(self):
        self.sunPointData.inputNavDataName = "simple_trans_nav_output"
        self.sunPointData.inputCelMessName = "sun_display_frame_data"
        self.sunPointData.outputDataName = "att_cmd_output"
        self.sunPointData.inputSecMessName = "earth_display_frame_data"
        TsunVec2Body = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
        self.sunPointData.TPoint2Bdy = TsunVec2Body

    def SetearthPoint(self):
        self.earthPointData.inputNavDataName = "simple_trans_nav_output"
        self.earthPointData.inputCelMessName = "earth_display_frame_data"
        self.earthPointData.outputDataName = "att_cmd_output"
        self.earthPointData.inputSecMessName = "sun_display_frame_data"
        angSin = math.sin(23.0 * math.pi / 180.0)
        angCos = math.cos(23.0 * math.pi / 180.0)
        TearthVec2Body = [0.0, 0.0, -1.0, -angSin, angCos, 0.0, angCos, angSin, 0.0]
        self.earthPointData.TPoint2Bdy = TearthVec2Body

    def SetmarsPoint(self):
        self.marsPointData.inputNavDataName = "simple_trans_nav_output"
        self.marsPointData.inputCelMessName = "mars_display_frame_data"
        self.marsPointData.inputSecMessName = "sun_display_frame_data"
        self.marsPointData.outputDataName = "att_cmd_output"
        TmarsVec2Body = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        # TmarsVec2Body = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.baseMarsTrans = TmarsVec2Body
        self.marsPointData.TPoint2Bdy = TmarsVec2Body
    
    def SetthrustRWDesat(self):
        self.thrustRWADesatData.inputSpeedName = "reactionwheel_output_states"
        self.thrustRWADesatData.outputThrName = "acs_thruster_cmds"
        self.thrustRWADesatData.inputRWConfigData = "rwa_config_data"
        self.thrustRWADesatData.inputThrConfigName = "rcs_config_data"
        self.thrustRWADesatData.inputMassPropsName = "adcs_config_data"
        self.thrustRWADesatData.maxFiring = 0.5
        self.thrustRWADesatData.thrFiringPeriod = 1.9
        RWAlignScale = 1.0 / 25.0
        self.thrustRWADesatData.DMThresh = 50 * (math.pi * 2.0) / 60.0 * RWAlignScale

    def SetAttUKF(self):
        self.AttUKF.ModelTag = "AttitudeUKF"
        initialCovariance = [0.0] * 6 * 6
        initialCovariance[0 * 6 + 0] = initialCovariance[1 * 6 + 1] = initialCovariance[2 * 6 + 2] = 0.02
        initialCovariance[3 * 6 + 3] = initialCovariance[4 * 6 + 4] = initialCovariance[5 * 6 + 5] = 0.0006
        self.AttUKF.CovarInit = initialCovariance
        obsNoise = [0.0] * 3 * 3
        obsNoise[0 * 3 + 0] = obsNoise[1 * 3 + 1] = obsNoise[2 * 3 + 2] = 0.062
        #obsNoise[3 * 6 + 3] = obsNoise[4 * 6 + 4] = obsNoise[5 * 6 + 5] = 0.008
        self.AttUKF.QStObs = obsNoise
        Qnoise = [0.0] * 6 * 6
        Qnoise[0 * 6 + 0] = Qnoise[1 * 6 + 1] = Qnoise[2 * 6 + 2] = 0.00002
        Qnoise[3 * 6 + 3] = Qnoise[4 * 6 + 4] = Qnoise[5 * 6 + 5] = 0.002
        self.AttUKF.QNoiseInit = Qnoise
        self.AttUKF.stInputName = "parsed_st_data"
        self.AttUKF.InertialUKFStateName = "attitude_filter_state"
        self.AttUKF.inputRWSpeeds = "reactionwheel_output_states"
        self.AttUKF.inputVehicleConfigDataName = "adcs_config_data"
        self.AttUKF.alpha = 0.1
        self.AttUKF.beta = 2.1
        self.AttUKF.kappa = 2.0
        self.AttUKF.ReInitFilter = True
        self.AttUKF.initToMeas = True

    def InitAllDynObjects(self):
        self.SetSpiceObject()
        self.SetIMUSensor()
        self.InitCSSHeads()
        self.SetACSThrusterDynObject()
        self.SetDVThrusterDynObject()
        self.SetVehDynObject()
        # self.setRadiationPressure()
        self.SetVehOrbElemObject()
        self.SetSimpleNavObject()
        self.SetsolarArrayBore()
        self.SetinstrumentBore()
        self.SethighGainBore()
        self.setClockSynchData()
        self.SetReactionWheelDynObject()
        self.SetStarTrackerData()

    def InitAllFSWObjects(self):
        self.SetVehicleConfigData()
        self.SetLocalConfigData()
        self.setRwFSWDeviceAvailability()
        self.SetCSSDecodeFSWConfig()
        self.SetIMUCommData()
        self.SetSTCommData()
        self.SetCSSWlsEstFSWConfig()
        self.SetsunSafePoint()
        self.SetMRP_Steering()
        self.SetsunSafeACS()
        self.SetattMnvrPoint()
        self.SetMRP_SteeringRWA()
        self.SetMRP_PD()
        self.SetMRP_FeedbackRWA()
        self.SetPRV_SteeringRWA()
        self.SetMRP_SteeringMOI()
        self.SetdvAttEffect()
        self.SetdvGuidance()
        self.SetsunPoint()
        self.SetearthPoint()
        self.SetmarsPoint()
        self.SetRWAMappingData()
        self.SetRWMotorTorque()
        self.SetRWANullSpaceData()
        self.SetthrustRWDesat()
        self.SetAttUKF()
        # Guidance FSW Objects
        self.setInertial3D()
        self.setHillPoint()
        self.setVelocityPoint()
        self.setCelTwoBodyPoint()
        self.setRasterManager()
        self.setInertial3DSpin()
        self.setEulerRotation()
        self.setAttTrackingError()
        self.setSimpleDeadband()

def LoadGravFromFile(FileName, GravBody, JParamsSelect):
    csvfile = open(FileName, 'rb')
    csvreader = csv.reader(csvfile)
    FirstLine = True
    NextJindex = 0
    AllJParams = []
    for row in csvreader:
        if (FirstLine == True):
            GravBody.mu = float(row[1])
            GravBody.radEquator = float(row[0])
            FirstLine = False
        elif (int(row[0]) == JParamsSelect[NextJindex]):
            LocalJParam = -math.sqrt(2 * JParamsSelect[NextJindex] + 1) * float(row[2])
            AllJParams.append(LocalJParam)
            NextJindex += 1
            if (NextJindex >= len(JParamsSelect)):
                break
    return (AllJParams)
