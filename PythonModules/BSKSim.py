'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

# Vehicle dynamics and avionics models
import spice_interface
import sim_model
import setup_gravityBody as gb

import six_dof_eom
import orb_elem_convert
import thruster_dynamics
import coarse_sun_sensor
import imu_sensor
import simple_nav
import bore_ang_calc
import reactionwheel_dynamics
import star_tracker
# FSW algorithms that we want to call
import cssComm
import vehicleConfigData
import cssWlsEst
import sunSafePoint
import imuComm
import stComm
import MRP_Feedback
import MRP_PD
import clock_synch
import thrFiringSchmitt
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
import rwConfigData
import VisualizationServer

import simSetupRW                 # RW simulation setup utilties
import simSetupThruster           # Thruster simulation setup utilties
import fswSetupRW

class BSKSim(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        self.modeRequest = 'None'
        self.isUsingVisualization = False
        self.server = None

        # Processes
        self.fswProc = self.CreateNewProcess("FSWProcess")
        self.dynProc = self.CreateNewProcess("DynamicsProcess")

        # Process message interfaces.
        self.dyn2FSWInterface = sim_model.SysInterface()
        self.fsw2DynInterface = sim_model.SysInterface()

        self.dyn2FSWInterface.addNewInterface("DynamicsProcess", "FSWProcess")
        self.fsw2DynInterface.addNewInterface("FSWProcess", "DynamicsProcess")

        self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
        self.fswProc.addInterfaceRef(self.fsw2DynInterface)

        # Dyn simulation tasks.
        self.dynProc.addTask(self.CreateNewTask("SynchTask", int(5E8)), 2000)
        self.dynProc.addTask(self.CreateNewTask("DynamicsTask", int(1E8)), 1000)
        # Flight software tasks.
        self.fswProc.addTask(self.CreateNewTask("initOnlyTask", int(1E10)), 1)
        self.fswProc.addTask(self.CreateNewTask("sunSafeFSWTask", int(5E8)), 999)
        self.fswProc.addTask(self.CreateNewTask("sensorProcessing", int(5E8)), 210)
        # -- Guidance Tasks
        self.fswProc.addTask(self.CreateNewTask("inertial3DPointTask", int(5E8)), 128)
        self.fswProc.addTask(self.CreateNewTask("hillPointTask", int(5E8)), 126)
        self.fswProc.addTask(self.CreateNewTask("velocityPointTask", int(5E8)), 125)
        self.fswProc.addTask(self.CreateNewTask("celTwoBodyPointTask", int(5E8)), 124)
        self.fswProc.addTask(self.CreateNewTask("rasterMnvrTask", int(5E8)), 118)
        self.fswProc.addTask(self.CreateNewTask("eulerRotationTask", int(5E8)), 117)
        self.fswProc.addTask(self.CreateNewTask("inertial3DSpinTask", int(5E8)), 116)
        # -- Controls Tasks
        self.fswProc.addTask(self.CreateNewTask("feedbackControlMnvrTask", int(5E8)), 110)
        self.fswProc.addTask(self.CreateNewTask("simpleRWControlTask", int(5E8)), 111)
        self.fswProc.addTask(self.CreateNewTask("thrForceMappingTask", int(5E8)), 101)
        self.fswProc.addTask(self.CreateNewTask("thrFiringSchmittTask", int(5E8)), 100)

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
        self.AddModelToTask("DynamicsTask", self.ACSThrusterDynObject, None, 302)
        self.AddModelToTask("DynamicsTask", self.DVThrusterDynObject, None, 301)
        self.AddModelToTask("DynamicsTask", self.rwDynObject, None, 300)
        self.AddModelToTask("DynamicsTask", self.VehDynObject, None, 201)
        self.AddModelToTask("DynamicsTask", self.VehOrbElemObject, None, 200)
        self.AddModelToTask("DynamicsTask", self.SimpleNavObject, None, 109)
        self.AddModelToTask("DynamicsTask", self.solarArrayBore, None, 110)
        self.AddModelToTask("DynamicsTask", self.trackerA, None, 113)

        # Flight software modules.
        self.VehConfigData = vehicleConfigData.VehConfigInputData()
        self.VehConfigDataWrap = self.setModelDataWrap(self.VehConfigData)
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

        self.MRP_PDSafeData = MRP_PD.MRP_PDConfig()
        self.MRP_PDSafeWrap = self.setModelDataWrap(self.MRP_PDSafeData)
        self.MRP_PDSafeWrap.ModelTag = "MRP_PD"
        
        self.MRP_FeedbackRWAData = MRP_Feedback.MRP_FeedbackConfig()
        self.MRP_FeedbackRWAWrap = self.setModelDataWrap(self.MRP_FeedbackRWAData)
        self.MRP_FeedbackRWAWrap.ModelTag = "MRP_FeedbackRWA"

        self.thrFiringSchmittData = thrFiringSchmitt.thrFiringSchmittConfig()
        self.thrFiringSchmittDataWrap = self.setModelDataWrap(self.thrFiringSchmittData)
        self.thrFiringSchmittDataWrap.ModelTag = "thrFiringSchmitt"

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

        self.rwConfigData = rwConfigData.rwConfigData()
        self.rwConfigWrap = self.setModelDataWrap(self.rwConfigData)
        self.rwConfigWrap.ModelTag = "rwConfigData"
        
        self.thrForceMappingData = thrForceMapping.thrForceMappingConfig()
        self.thrForceMappingWrap = self.setModelDataWrap(self.thrForceMappingData)
        self.thrForceMappingWrap.ModelTag = "thrForceMapping"

        # Initialize flight software modules.
        self.InitAllFSWObjects()
        self.AddModelToTask("initOnlyTask", self.VehConfigDataWrap, self.VehConfigData, 2)
        self.AddModelToTask("initOnlyTask", self.rwConfigWrap, self.rwConfigData, 2)

        # Add flight software modules to task groups.
        self.AddModelToTask("sunSafeFSWTask", self.IMUCommWrap, self.IMUCommData, 10)
        self.AddModelToTask("sunSafeFSWTask", self.CSSAlgWrap, self.CSSDecodeFSWConfig, 9)
        self.AddModelToTask("sunSafeFSWTask", self.CSSWlsWrap, self.CSSWlsEstFSWConfig, 8)
        self.AddModelToTask("sunSafeFSWTask", self.sunSafePointWrap, self.sunSafePointData, 7)
        self.AddModelToTask("sunSafeFSWTask", self.simpleDeadbandWrap, self.simpleDeadbandData, 6)
        self.AddModelToTask("sunSafeFSWTask", self.MRP_PDSafeWrap, self.MRP_PDSafeData, 5)

        self.AddModelToTask("sensorProcessing", self.CSSAlgWrap, self.CSSDecodeFSWConfig, 9)
        self.AddModelToTask("sensorProcessing", self.IMUCommWrap, self.IMUCommData, 10)
        self.AddModelToTask("sensorProcessing", self.STCommWrap, self.STCommData, 11)

        self.AddModelToTask("thrForceMappingTask", self.thrForceMappingWrap, self.thrForceMappingData)
        self.AddModelToTask("thrFiringSchmittTask", self.thrFiringSchmittDataWrap, self.thrFiringSchmittData)

        # Mapping of Guidance Models to Guidance Tasks
        self.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 20)
        self.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 20)
        self.AddModelToTask("velocityPointTask", self.velocityPointWrap, self.velocityPointData, 20)
        self.AddModelToTask("celTwoBodyPointTask", self.celTwoBodyPointWrap, self.celTwoBodyPointData, 20)
        self.AddModelToTask("eulerRotationTask", self.eulerRotationWrap, self.eulerRotationData, 19)
        self.AddModelToTask("inertial3DSpinTask", self.inertial3DSpinWrap, self.inertial3DSpinData, 19)
        self.AddModelToTask("rasterMnvrTask", self.rasterManagerWrap, self.rasterManagerData, 19)
        self.AddModelToTask("rasterMnvrTask", self.eulerRotationWrap, self.eulerRotationData, 18)
        

        self.AddModelToTask("feedbackControlMnvrTask", self.attTrackingErrorWrap, self.attTrackingErrorData, 10)
        self.AddModelToTask("feedbackControlMnvrTask", self.MRP_FeedbackRWAWrap, self.MRP_FeedbackRWAData, 9)
        self.AddModelToTask("feedbackControlMnvrTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)


        # Disable all tasks in the FSW process
        self.fswProc.disableAllTasks()
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
                             "self.enableTask('sunSafeFSWTask')",
                             "self.enableTask('thrForceMappingTask')",
                             "self.enableTask('thrFiringSchmittTask')"])



    def InitializeSimulation(self):
        SimulationBaseClass.SimBaseClass.InitializeSimulation(self)
        self.dyn2FSWInterface.discoverAllMessages()
        self.fsw2DynInterface.discoverAllMessages()
        if self.isUsingVisualization:
            self.server = VisualizationServer.VisualizationServer(self)
            self.server.startServer()

    # Set the static spacecraft parameters
    def SetLocalConfigData(self):
        def ReactionWheelConst():
            self.RWAGsMatrix = []
            self.RWAJsList = []
            rwElAngle = 45.0 * math.pi / 180.0
            rwClockAngle = 45.0 * math.pi / 180.0
            rwClass = vehicleConfigData.RWConstellation()
            rwClass.numRW = 4
            rwPointer = vehicleConfigData.RWConfigurationElement()
            for i in range(0, rwClass.numRW):
                gsHat_S = [-math.sin(rwElAngle) * math.sin(rwClockAngle),
                           -math.sin(rwElAngle) * math.cos(rwClockAngle),
                           -math.cos(rwElAngle)]
                J_S = 100.0 / (6000.0 / 60.0 * math.pi * 2.0)
                self.RWAGsMatrix.extend(gsHat_S)
                self.RWAJsList.extend([J_S])
                rwPointer.gsHat_S = gsHat_S
                rwPointer.Js = J_S
                vehicleConfigData.RWConfigArray_setitem(rwClass.reactionWheels, i, rwPointer)
                rwClockAngle += 90.0 * math.pi / 180.0
            msgSizeRW = 4 + vehicleConfigData.MAX_EFF_CNT*7*8
            self.TotalSim.CreateNewMessage("FSWProcess", "rwa_config_data",msgSizeRW, 2, "RWConstellation")
            self.TotalSim.WriteMessageData("rwa_config_data", msgSizeRW, 0, rwClass)
            self.rwConfigData.rwConstellation = rwClass

        def ThrusterCluster():
            rcsClass = vehicleConfigData.ThrusterCluster()
            rcsPointer = vehicleConfigData.ThrusterPointData()
            rcsLocationData = [
                [-0.86360, -0.82550,  1.79070],
                [-0.82550, -0.86360,  1.79070],
                [ 0.82550,  0.86360,  1.79070],
                [ 0.86360,  0.82550,  1.79070],
                [-0.86360, -0.82550, -1.79070],
                [-0.82550, -0.86360, -1.79070],
                [ 0.82550,  0.86360, -1.79070],
                [ 0.86360,  0.82550, -1.79070]
            ]
            rcsDirectionData = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0]
            ]
            maxThr = 0.875
            maxThrust = [maxThr, maxThr, maxThr, maxThr,
                         maxThr, maxThr, maxThr, maxThr]
            rcsClass.numThrusters = 8
            for i in range(0, rcsClass.numThrusters):
                rcsPointer.rThrust_S = rcsLocationData[i]
                rcsPointer.tHatThrust_S = rcsDirectionData[i]
                rcsPointer.maxThrust = maxThrust[i]
                vehicleConfigData.ThrustConfigArray_setitem(rcsClass.thrusters, i, rcsPointer)

            msgSizeThrust = 4 + vehicleConfigData.MAX_EFF_CNT*7*8
            self.TotalSim.CreateNewMessage("FSWProcess", "rcs_config_data", msgSizeThrust, 2, "ThrusterCluster")
            self.TotalSim.WriteMessageData("rcs_config_data", msgSizeThrust, 0, rcsClass)

        ReactionWheelConst()
        ThrusterCluster()

    def SetRWConfigDataFSW(self):
        self.rwConfigData.rwConstellationInMsgName = "rwa_config_data"
        self.rwConfigData.vehConfigInMsgName = "adcs_config_data"
        self.rwConfigData.rwParamsOutMsgName = "rwa_config_data_parsed"

    def SetRwFSWDeviceAvailability(self):
        rwAvailabilityMessage = rwMotorTorque.RWAvailabilityData()
        avail = [rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE]
        rwAvailabilityMessage.wheelAvailability = avail
        msgSize = vehicleConfigData.MAX_EFF_CNT*4
        self.TotalSim.CreateNewMessage("FSWProcess", "rw_availability", msgSize, 2, "RWAvailabilityData")
        self.TotalSim.WriteMessageData("rw_availability", msgSize, 0, rwAvailabilityMessage)

    def SetSpiceObject(self):
        self.SpiceObject.ModelTag = "SpiceInterfaceData"
        self.SpiceObject.SPICEDataPath = self.simBasePath + '/External/EphemerisData/'
        self.SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
        self.SpiceObject.OutputBufferCount = 2
        self.SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
        self.SpiceObject.referenceBase = "MARSIAU"

    def SetIMUSensor(self):
        def turnOffCorruption_imu():
            rotBiasValue = 0.0
            rotNoiseStdValue = 0.0
            rotErrorBounds = [0] * 3
            transBiasValue = 0.0
            transNoiseStdValue = 0.0
            transErrorBounds = [0] * 3
            return (rotBiasValue, rotNoiseStdValue, rotErrorBounds, transBiasValue, transNoiseStdValue, transErrorBounds)

        def defaultCorruption_imu():
            rotBiasValue = 0.0
            rotNoiseStdValue = 0.000001
            rotErrorBounds = [0] * 3
            transBiasValue = 0.0
            transNoiseStdValue = 1.0E-6
            transErrorBounds = [0] * 3
            return (
            rotBiasValue, rotNoiseStdValue, rotErrorBounds, transBiasValue, transNoiseStdValue, transErrorBounds)

        # Turn off corruption of IMU data
        (rotBiasValue, rotNoiseStdValue, rotErrorBounds, transBiasValue, transNoiseStdValue,
         transErrorBounds) = turnOffCorruption_imu()

        PMatrixGyro = [0.0] * 3 * 3
        PMatrixGyro[0 * 3 + 0] = PMatrixGyro[1 * 3 + 1] = PMatrixGyro[2 * 3 + 2] = rotNoiseStdValue
        PMatrixAccel = [0.0] * 3 * 3
        PMatrixAccel[0 * 3 + 0] = PMatrixAccel[1 * 3 + 1] = PMatrixAccel[2 * 3 + 2] = transNoiseStdValue

        self.IMUSensor = imu_sensor.ImuSensor()
        self.IMUSensor.PMatrixAccel = sim_model.DoubleVector(PMatrixAccel)
        self.IMUSensor.walkBoundsAccel = sim_model.DoubleVector(transErrorBounds)
        self.IMUSensor.PMatrixGyro = sim_model.DoubleVector(PMatrixGyro)
        self.IMUSensor.walkBoundsGyro = sim_model.DoubleVector(rotErrorBounds)
        self.IMUSensor.SensorPosStr = imu_sensor.DoubleVector([1.5, 0.1, 0.1])
        self.IMUSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
        self.IMUSensor.accelLSB = 2.77E-4 * 9.80665
        self.IMUSensor.gyroLSB = 8.75E-3 * np.pi / 180.0
        self.IMUSensor.senRotMax = 1e6
        self.IMUSensor.senTransMax = 1e6
        self.IMUSensor.senTransBias = [transBiasValue, transBiasValue, transBiasValue]
        self.IMUSensor.senRotBias = [rotBiasValue, rotBiasValue, rotBiasValue]


    def SetReactionWheelDynObject(self):
        def define_gsHat(rwElAngle, rwClockAngle):
            gs_hat = [-math.sin(rwElAngle) * math.sin(rwClockAngle),
                      -math.sin(rwElAngle) * math.cos(rwClockAngle),
                      -math.cos(rwElAngle)
                      ]
            return gs_hat
        rwType = 'Honeywell_HR16'
        modelTag = "ReactionWheels"
        self.rwDynObject.inputVehProps = "spacecraft_mass_props"
        simSetupRW.clearSetup()
        simSetupRW.options.useRWfriction = False
        simSetupRW.options.useMinTorque = False
        simSetupRW.options.maxMomentum = 100    # Nms
        N = 4 # num RW
        Rs_list = [
            [0.8, 0.8, 1.79070],
            [0.8, -0.8, 1.79070],
            [-0.8, -0.8, 1.79070],
            [-0.8, 0.8, 1.79070]
        ]
        omega_spin0 = 0.0
        rwElAngle = 45.0 * math.pi / 180.0
        rwClockAngle = 45.0 * math.pi / 180.0
        for i in range(0, N):
            gs_hat = define_gsHat(rwElAngle, rwClockAngle)
            simSetupRW.create(rwType, gs_hat, omega_spin0, Rs_list[i])
            rwClockAngle += 90.0 * math.pi / 180.0
        simSetupRW.addToSpacecraft(modelTag, self.rwDynObject, self.VehDynObject)

    def SetACSThrusterDynObject(self):
        self.ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
        self.ACSThrusterDynObject.InputCmds = "acs_thruster_cmds"
        self.ACSThrusterDynObject.inputProperties = "spacecraft_mass_props"

        simSetupThruster.clearSetup()
        thrusterType = 'MOOG_Monarc_1'
        N = 8 # num AC thrusters
        Rs_list = [
            [-0.86360, -0.82550, 1.79070],
            [-0.82550, -0.86360, 1.79070],
            [0.82550, 0.86360, 1.79070],
            [0.86360, 0.82550, 1.79070],
            [-0.86360, -0.82550, -1.79070],
            [-0.82550, -0.86360, -1.79070],
            [0.82550, 0.86360, -1.79070],
            [0.86360, 0.82550, -1.79070]
        ]
        Gs_list = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0]
        ]
        if (N == len(Rs_list) and N == len(Gs_list)):
            for i in range(0, N):
                simSetupThruster.create(thrusterType, Rs_list[i], Gs_list[i])

            simSetupThruster.addToSpacecraft(self.ACSThrusterDynObject.ModelTag,
                                self.ACSThrusterDynObject,
                                self.VehDynObject)
        else:
            raise ValueError('ACS Thrusters dyn effectors are not initialized properly.')

        ACSpropCM = [0.0, 0.0, 1.2]
        ACSpropMass = 40  # Made up
        ACSpropRadius = 46.0 / 2.0 / 3.2808399 / 12.0
        sphereInerita = 2.0 / 5.0 * ACSpropMass * ACSpropRadius * ACSpropRadius
        ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
        self.ACSThrusterDynObject.objProps.Mass = ACSpropMass
        self.ACSThrusterDynObject.objProps.CoM = ACSpropCM
        self.ACSThrusterDynObject.objProps.InertiaTensor = ACSInertia

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
        # Turn off corruption of CSS data
        def turnOffCorruption_css():
            CSSNoiseStd = 0.0
            CSSNoiseBias = 0.0
            CSSKellyFactor = 0.0
            return (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor)
        # Use default values of CSS data corruption
        def defaultCorruption_css():
            CSSNoiseStd = 0.001  # Standard deviation of white noise
            CSSNoiseBias = 0.0  # Constant bias
            CSSKellyFactor = 0.1  # Used to get the curve shape correct for output
            return (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor)

        CSSscaleFactor = 500.0E-6  # Scale factor (500 mu-amps) for max measurement
        CSSFOV = 90.0 * math.pi / 180.0  # 90 degree field of view
        (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor) = turnOffCorruption_css()

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
        def initCSS_orientation(theta, phi, perturbDir, CSSHead):
            CSSHead.theta = theta * math.pi / 180.0
            CSSHead.phi = phi * math.pi / 180.0
            CSSHead.setUnitDirectionVectorWithPerturbation(perturbDir[0], perturbDir[1])
            return CSSHead

        perturbDir = [0.0, 0.0]
        CSSPyramid1HeadA = initCSS_orientation(0.0, 45.0, perturbDir, CSSPyramid1HeadA)
        CSSPyramid1HeadB = initCSS_orientation(90.0, 45.0, perturbDir, CSSPyramid1HeadB)
        CSSPyramid1HeadC = initCSS_orientation(180.0, 45.0, perturbDir, CSSPyramid1HeadC)
        CSSPyramid1HeadD = initCSS_orientation(270.0, 45.0, perturbDir, CSSPyramid1HeadD)

        CSSPyramid2HeadA = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid2HeadA.ModelTag = "CSSPyramid2HeadA"
        CSSPyramid2HeadA.OutputDataMsg = "coarse_sun_data_pyramid2_headA"
        CSSPyramid2HeadA.setStructureToPlatformDCM(CSSPlatform2YPR[0], CSSPlatform2YPR[1], CSSPlatform2YPR[2])
        CSSPyramid2HeadA.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid2HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadB)
        CSSPyramid2HeadB.ModelTag = "CSSPyramid2HeadB"
        CSSPyramid2HeadB.OutputDataMsg = "coarse_sun_data_pyramid2_headB"
        CSSPyramid2HeadB.setStructureToPlatformDCM(CSSPlatform2YPR[0], CSSPlatform2YPR[1], CSSPlatform2YPR[2])
        CSSPyramid2HeadB.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid2HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadC)
        CSSPyramid2HeadC.ModelTag = "CSSPyramid2HeadC"
        CSSPyramid2HeadC.OutputDataMsg = "coarse_sun_data_pyramid2_headC"
        CSSPyramid2HeadC.setStructureToPlatformDCM(CSSPlatform2YPR[0], CSSPlatform2YPR[1], CSSPlatform2YPR[2])
        CSSPyramid2HeadC.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        CSSPyramid2HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadD)
        CSSPyramid2HeadD.ModelTag = "CSSPyramid2HeadD"
        CSSPyramid2HeadD.OutputDataMsg = "coarse_sun_data_pyramid2_headD"
        CSSPyramid2HeadD.setStructureToPlatformDCM(CSSPlatform2YPR[0], CSSPlatform2YPR[1], CSSPlatform2YPR[2])
        CSSPyramid2HeadD.setUnitDirectionVectorWithPerturbation(0.0, 0.0)

        self.cssConstellation.sensorList = coarse_sun_sensor.CSSVector([CSSPyramid1HeadA, CSSPyramid1HeadB, CSSPyramid1HeadC, CSSPyramid1HeadD,
            CSSPyramid2HeadA, CSSPyramid2HeadB, CSSPyramid2HeadC, CSSPyramid2HeadD])

    def SetVehDynObject(self):
        self.VehDynObject.ModelTag = "VehicleDynamicsData"
        r0 = [2.342211275644610E+07 * 1000.0, -1.503236698659483E+08 * 1000.0, -1.786319594218582E+04 * 1000.0]
        v0 = [2.896852053342327E+01 * 1000.0, 4.386175246767674E+00 * 1000.0, -3.469168621992313E-04 * 1000.0]
        sigma0_BN = [0.4, 0.2, 0.1]
        omega0_BN = [0.0001, 0.0, 0.0]
        m = 1500.0 - 812.3
        I =[1000, 0.0, 0.0,
            0.0, 800.0, 0.0,
            0.0, 0.0, 800.0]
        DCM_BS = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        CoM = [0.0, 0.0, 1.0]
        gb.Init_spacecraftVehicle(self.VehDynObject, r0, v0, sigma0_BN, omega0_BN, m, I, DCM_BS, CoM)

        self.SunGravBody = six_dof_eom.GravityBodyData()
        self.SunGravBody.outputMsgName = "sun_display_frame_data"
        self.EarthGravBody = six_dof_eom.GravityBodyData()
        self.EarthGravBody.outputMsgName = "earth_display_frame_data"
        self.MarsGravBody = six_dof_eom.GravityBodyData()
        self.MarsGravBody.outputMsgName = "mars barycenter_display_frame_data"
        # Add the three gravity bodies in to the simulation
        gb.Add_sunGravityBody(self, self.SunGravBody)
        gb.Add_earthGravityBody(self, self.EarthGravBody)
        gb.Add_marsGravityBody(self, self.MarsGravBody)

        # Here is where the thruster dynamics are attached/scheduled to the overall
        # vehicle dynamics.  Anything that is going to impact the dynamics of the
        # vehicle should be one of these body effectors I think.
        self.VehDynObject.addThrusterSet(self.ACSThrusterDynObject)
        self.VehDynObject.addThrusterSet(self.DVThrusterDynObject)
        self.VehDynObject.useTranslation = True
        self.VehDynObject.useRotation = True


    def SetVehOrbElemObject(self):
        self.VehOrbElemObject.ModelTag = "VehicleOrbitalElements"
        self.VehOrbElemObject.mu = self.SunGravBody.mu

    def SetsolarArrayBore(self):
        self.solarArrayBore.ModelTag = "solarArrayBoresight"
        self.solarArrayBore.StateString = "inertial_state_output"
        self.solarArrayBore.celBodyString = "sun_display_frame_data"
        self.solarArrayBore.OutputDataString = "solar_array_sun_bore"
        self.solarArrayBore.strBoreVec = [0.0, 0.0, 1.0]

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
        def turnOffCorruption_st():
            PMatrix = [0.0] * 3 * 3
            PMatrix[0 * 3 + 0] = PMatrix[1 * 3 + 1] = PMatrix[2 * 3 + 2] = 0.0
            errorBounds = [0.0 / 3600 * np.pi / 180.0] * 3
            return (PMatrix, errorBounds)
        def defaultCorruption_st():
            PMatrix = [0.0] * 3 * 3
            PMatrix[0 * 3 + 0] = PMatrix[1 * 3 + 1] = PMatrix[2 * 3 + 2] = 0.5 / 3600.0 * np.pi / 180.0  # 20 arcsecs?+
            errorBounds = [5.0 / 3600 * np.pi / 180.0] * 3
            return (PMatrix, errorBounds)

        (PMatrix, errorBounds) = turnOffCorruption_st()
        self.trackerA.ModelTag = "StarTrackerA"
        self.trackerA.walkBounds = sim_model.DoubleVector(errorBounds)
        self.trackerA.PMatrix = sim_model.DoubleVector(PMatrix)

    def setClockSynchData(self):
        self.clockSynchData.ModelTag = "ClockSynchModel"
        self.clockSynchData.accelFactor = 1.0
        self.clockSynchData.clockOutputName = "clock_synch_data"
        self.clockSynchData.outputBufferCount = 2
    
    def SetVehicleConfigData(self):
        BS = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
        self.VehConfigData.BS = BS
        Inertia = [700.0, 0.0, 0.0,
                   0.0, 700.0, 0.0,
                   0.0, 0.0, 800]  # kg * m^2
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

    def SetMRP_PD(self):
        self.MRP_PDSafeData.K = 4.5
        self.MRP_PDSafeData.P = 150.0  # N*m*sec
        self.MRP_PDSafeData.inputGuidName = "db_att_guid_out"
        self.MRP_PDSafeData.inputVehicleConfigDataName = "adcs_config_data"
        self.MRP_PDSafeData.outputDataName = "controlTorqueRaw"
        
    def setSimpleDeadband(self):
        self.simpleDeadbandData.inputGuidName = "sun_safe_att_err"
        self.simpleDeadbandData.outputDataName = "db_att_guid_out"
        self.simpleDeadbandData.innerAttThresh = 4.0 * (math.pi / 180.)
        self.simpleDeadbandData.outerAttThresh = 4.0 * (math.pi / 180.)
        self.simpleDeadbandData.innerRateThresh = 0.1 * (math.pi / 180.)
        self.simpleDeadbandData.outerRateThresh = 0.1 * (math.pi / 180.)


    # Init of Guidance Modules
    def setInertial3D(self):
        self.inertial3DData.outputDataName = "att_ref_output_stage1"
        sigma_R0N = [0., 0., 0.]
        self.inertial3DData.sigma_R0N = sigma_R0N

    def setHillPoint(self):
        self.hillPointData.inputNavDataName = "simple_trans_nav_output"
        self.hillPointData.inputCelMessName = "mars barycenter_display_frame_data"
        self.hillPointData.outputDataName = "att_ref_output_stage1"

    def setVelocityPoint(self):
        self.velocityPointData.inputNavDataName = "simple_trans_nav_output"
        self.velocityPointData.inputCelMessName = "mars barycenter_display_frame_data"
        self.velocityPointData.outputDataName = "att_ref_output"
        self.velocityPointData.mu = self.SunGravBody.mu

    def setCelTwoBodyPoint(self):
        self.celTwoBodyPointData.inputNavDataName = "simple_trans_nav_output"
        self.celTwoBodyPointData.inputCelMessName = "mars barycenter_display_frame_data"
        #self.celTwoBodyPointData.inputSecMessName = "sun_display_frame_data"
        self.celTwoBodyPointData.outputDataName = "att_ref_output"
        self.celTwoBodyPointData.singularityThresh = 1.0 * mc.D2R

    def setRasterManager(self):
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
        self.rasterManagerData.outputEulerSetName = "euler_angle_set"
        self.rasterManagerData.outputEulerRatesName = "euler_angle_rates"
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

    def SetMRP_FeedbackRWA(self):
        self.MRP_FeedbackRWAData.K = 1.  # rad/sec
        self.MRP_FeedbackRWAData.P = 3.  # N*m*sec
        self.MRP_FeedbackRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.MRP_FeedbackRWAData.integralLimit = 0.0  # rad

        self.MRP_FeedbackRWAData.inputGuidName = "nom_att_guid_out"
        self.MRP_FeedbackRWAData.vehConfigInMsgName = "adcs_config_data"
        self.MRP_FeedbackRWAData.rwParamsInMsgName = "rwa_config_data_parsed"
        self.MRP_FeedbackRWAData.rwAvailInMsgName = "rw_availability"
        self.MRP_FeedbackRWAData.inputRWSpeedsName = "reactionwheel_output_states"
        self.MRP_FeedbackRWAData.outputDataName = "controlTorqueRaw"

    def SetRWMotorTorque(self):
        controlAxes_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.inputVehControlName = "controlTorqueRaw"
        self.rwMotorTorqueData.outputDataName = "reactionwheel_cmds"
        self.rwMotorTorqueData.rwAvailInMsgName = "rw_availability"
        self.rwMotorTorqueData.rwParamsInMsgName = "rwa_config_data_parsed"

    def SetthrForceMapping(self):
        self.thrForceMappingData.inputVehControlName = "controlTorqueRaw"
        self.thrForceMappingData.outputDataName = "acs_thruster_cmds_mapped"
        self.thrForceMappingData.inputThrusterConfName = "rcs_config_data"
        self.thrForceMappingData.inputVehicleConfigDataName = "adcs_config_data"
        self.thrForceMappingData.thrForceSign = +1
        self.thrForceMappingData.epsilon = 0.0005
        self.thrForceMappingData.controlAxes_B = [
             1,0,0
            ,0,1,0
            ,0,0,1
        ]

    def SetthrFiringSchmitt(self):
        self.thrFiringSchmittData.thrForceInMsgName = "acs_thruster_cmds_mapped"
        self.thrFiringSchmittData.onTimeOutMsgName = "acs_thruster_cmds"
        self.thrFiringSchmittData.thrConfInMsgName = "rcs_config_data"
        self.thrFiringSchmittData.level_on = 0.50
        self.thrFiringSchmittData.level_off = 0.25
        self.thrFiringSchmittData.thrMinFireTime = 0.030
        self.thrFiringSchmittData.baseThrustState = 0


    def InitAllDynObjects(self):
        self.SetSpiceObject()
        self.SetIMUSensor()
        self.InitCSSHeads()
        self.SetACSThrusterDynObject()
        self.SetDVThrusterDynObject()
        self.SetVehDynObject()

        self.SetVehOrbElemObject()
        self.SetSimpleNavObject()
        self.SetsolarArrayBore()
        self.setClockSynchData()
        self.SetReactionWheelDynObject()
        self.SetStarTrackerData()

    def InitAllFSWObjects(self):
        # Vehicle
        self.SetVehicleConfigData()
        self.SetLocalConfigData()
        self.SetRWConfigDataFSW()
        self.SetRwFSWDeviceAvailability()
        # Sensors
        self.SetCSSDecodeFSWConfig()
        self.SetIMUCommData()
        self.SetSTCommData()
        self.SetCSSWlsEstFSWConfig()
        # Guidance AttRef.
        self.SetsunSafePoint()
        self.setInertial3D()
        self.setHillPoint()
        self.setVelocityPoint()
        self.setCelTwoBodyPoint()
        self.setRasterManager()
        self.setInertial3DSpin()
        self.setEulerRotation()
        # Tracking Error
        self.setAttTrackingError()
        self.setSimpleDeadband()
        # Controls
        self.SetMRP_PD()
        self.SetMRP_FeedbackRWA()
        self.SetRWMotorTorque()
        self.SetthrForceMapping()
        self.SetthrFiringSchmitt()