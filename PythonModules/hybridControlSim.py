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

import numpy as np

# support modules


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# This part definitely needs work.  Need to detect Basilisk somehow.
sys.path.append(path + '/../../Basilisk/PythonModules')
sys.path.append(path + '/../../Basilisk/modules')
# Simulation base class is needed because we inherit from it
import SimulationBaseClass as SBC
import sim_model
import setup_gravityBody as gb

import simSetupRW as drw
import simSetupThruster as dth

import clock_synch

import vehicleConfigData
import six_dof_eom

import simple_nav
import spice_interface
import orb_elem_convert

import reactionwheel_dynamics
import thruster_dynamics

import star_tracker
import coarse_sun_sensor
import imu_sensor

import bore_ang_calc


class HybridControlSim(SBC.SimBaseClass):
    def __init__(self):
        # Create a sim module as an empty container
        SBC.SimBaseClass.__init__(self)
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


        # Process task groups.
        self.dynProc.addTask(self.CreateNewTask("SynchTask", int(5E8)), 2000)
        self.dynProc.addTask(self.CreateNewTask("DynamicsTask", int(1E8)), 1000)

        # Flight software tasks.
        self.fswProc.addTask(self.CreateNewTask("initOnlyTask", int(1E10)), 1)
        self.fswProc.addTask(self.CreateNewTask("sunSafeFSWTask", int(5E8)), 999)
        self.fswProc.addTask(self.CreateNewTask("sensorProcessing", int(5E8)), 210)
        self.fswProc.addTask(self.CreateNewTask("attitudeNav", int(5E8)), 209)
        self.fswProc.addTask(self.CreateNewTask("hillPointTask", int(5E8)), 126)
        self.fswProc.addTask(self.CreateNewTask("velocityPointTask", int(5E8)), 125)
        self.fswProc.addTask(self.CreateNewTask("celTwoBodyPointTask", int(5E8)), 124)
        self.fswProc.addTask(self.CreateNewTask("steeringControlMnvrTask", int(5E8)), 110)
        self.fswProc.addTask(self.CreateNewTask("feedbackControlMnvrTask", int(5E8)), 110)
        self.fswProc.addTask(self.CreateNewTask("simpleRWControlTask", int(5E8)), 111)


        # Spacecraft configuration data module.
        self.clockSynchData = clock_synch.ClockSynch()
        
        self.LocalConfigData = vehicleConfigData.vehicleConfigData()
        self.VehDynObject = six_dof_eom.SixDofEOM()
        
        self.SimpleNavObject = simple_nav.SimpleNav()
        self.SpiceObject = spice_interface.SpiceInterface()
        self.VehOrbElemObject = orb_elem_convert.OrbElemConvert()

        # Control effectors
        self.rwDynObject = reactionwheel_dynamics.ReactionWheelDynamics()
        self.ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()
        # Sensors
        self.starTracker = star_tracker.StarTracker()
        self.cssConstellation = coarse_sun_sensor.CSSConstellation()
        self.IMUSensor = imu_sensor.ImuSensor()
        # Antennas
        self.solarArrayBore = bore_ang_calc.BoreAngCalc()
        self.highGainBore = bore_ang_calc.BoreAngCalc()

        self.InitAllDynObjects()

        # Add simulation modules to task groups.
        self.disableTask("SynchTask")
        self.AddModelToTask("SynchTask", self.clockSynchData, None, 100)
        
        self.AddModelToTask("DynamicsTask", self.VehDynObject, None, 201)
        self.AddModelToTask("DynamicsTask", self.SimpleNavObject, None, 109)
        self.AddModelToTask("DynamicsTask", self.SpiceObject, None, 202)
        self.AddModelToTask("DynamicsTask", self.VehOrbElemObject, None, 200)

        self.AddModelToTask("DynamicsTask", self.rwDynObject, None, 300)
        self.AddModelToTask("DynamicsTask", self.ACSThrusterDynObject, None, 302)

        self.AddModelToTask("DynamicsTask", self.starTracker, None, 113)
        self.AddModelToTask("DynamicsTask", self.cssConstellation, None, 108)
        self.AddModelToTask("DynamicsTask", self.IMUSensor, None, 100)
        
        self.AddModelToTask("DynamicsTask", self.solarArrayBore, None, 110)
        self.AddModelToTask("DynamicsTask", self.highGainBore, None, 112)
        

    # -------- DYNAMIC OBJECTS -------- #
    def setClockSynchData(self):
        self.clockSynchData.ModelTag = "ClockSynchModel"
        self.clockSynchData.accelFactor = 1.0
        self.clockSynchData.clockOutputName = "clock_synch_data"
        self.clockSynchData.outputBufferCount = 2
        
    def SetSpiceObject(self):
        self.SpiceObject.ModelTag = "SpiceInterfaceData"
        self.SpiceObject.SPICEDataPath = self.simBasePath + '/External/EphemerisData/'
        self.SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
        self.SpiceObject.OutputBufferCount = 2
        self.SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
        self.SpiceObject.referenceBase = "MARSIAU"

    def SetVehDynObject(self):
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

        self.EarthGravBody = six_dof_eom.GravityBodyData()
        self.EarthGravBody.BodyMsgName = "earth_planet_data"
        self.SunGravBody = six_dof_eom.GravityBodyData()
        self.SunGravBody.outputMsgName = "sun_display_frame_data"
        gb.Add_sunGravityBody(self, self.SunGravBody)
        gb.Add_earthGravityBody(self, self.EarthGravBody)

        self.VehDynObject.ModelTag = "VehicleDynamicsData"
        self.VehDynObject.useTranslation = True
        self.VehDynObject.useRotation = True

    def SetVehOrbElemObject(self):
        self.VehOrbElemObject.ModelTag = "VehicleOrbitalElements"
        self.VehOrbElemObject.mu = self.SunGravBody.mu

    # NAV SYSTEM
    def SetSimpleNavObject(self):
        def turnOffCorruption_nav():
            PMatrix = [0.0] * 18 * 18
            PMatrix[0 * 18 + 0] = PMatrix[1 * 18 + 1] = PMatrix[2 * 18 + 2] = 0.0  # Position
            PMatrix[3 * 18 + 3] = PMatrix[4 * 18 + 4] = PMatrix[5 * 18 + 5] = 0.0  # Velocity
            PMatrix[6 * 18 + 6] = PMatrix[7 * 18 + 7] = PMatrix[8 * 18 + 8] = 0.0 * np.pi / 180.0  # Attitude (sigma!)
            PMatrix[9 * 18 + 9] = PMatrix[10 * 18 + 10] = PMatrix[11 * 18 + 11] = 0.0 * np.pi / 180.0  # Attitude rate
            PMatrix[12 * 18 + 12] = PMatrix[13 * 18 + 13] = PMatrix[14 * 18 + 14] = 0.0 * np.pi / 180.0  # Sun vector
            PMatrix[15 * 18 + 15] = PMatrix[16 * 18 + 16] = PMatrix[17 * 18 + 17] = 0.0  # Accumulated DV
            errorBounds = [0.0, 0.0, 0.0,  # Position
                           0.0, 0.0, 0.0,  # Velocity
                           0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0,  # Attitude
                           0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0,  # Attitude Rate
                           0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0, 0.0 * np.pi / 180.0,  # Sun vector
                           0.0, 0.0, 0.0]  # Accumulated DV
            return (PMatrix, errorBounds)

        def defaultCorruption_nav():
            PMatrix = [0.0] * 18 * 18
            PMatrix[0 * 18 + 0] = PMatrix[1 * 18 + 1] = PMatrix[2 * 18 + 2] = 10.0  # Position
            PMatrix[3 * 18 + 3] = PMatrix[4 * 18 + 4] = PMatrix[5 * 18 + 5] = 0.05  # Velocity
            PMatrix[6 * 18 + 6] = PMatrix[7 * 18 + 7] = PMatrix[
                8 * 18 + 8] = 1.0 / 3600.0 * np.pi / 180.0  # Attitude (sigma!)
            PMatrix[9 * 18 + 9] = PMatrix[10 * 18 + 10] = PMatrix[11 * 18 + 11] = 0.0001 * np.pi / 180.0  # Attitude rate
            PMatrix[12 * 18 + 12] = PMatrix[13 * 18 + 13] = PMatrix[14 * 18 + 14] = 0.1 * np.pi / 180.0  # Sun vector
            PMatrix[15 * 18 + 15] = PMatrix[16 * 18 + 16] = PMatrix[17 * 18 + 17] = 0.003  # Accumulated DV
            errorBounds = [1000.0, 1000.0, 1000.0,  # Position
                           1.0, 1.0, 1.0,  # Velocity
                           1.6E-2 * np.pi / 180.0, 1.6E-2 * np.pi / 180.0, 1.6E-2 * np.pi / 180.0,  # Attitude
                           0.0004 * np.pi / 180.0, 0.0004 * np.pi / 180.0, 0.0004 * np.pi / 180.0,  # Attitude Rate
                           5.0 * np.pi / 180.0, 5.0 * np.pi / 180.0, 5.0 * np.pi / 180.0,  # Sun vector
                           0.053, 0.053, 0.053]  # Accumulated DV
            return (PMatrix, errorBounds)

        (PMatrix, errorBounds) = turnOffCorruption_nav()
        self.SimpleNavObject.ModelTag = "SimpleNavigation"
        self.SimpleNavObject.walkBounds = sim_model.DoubleVector(errorBounds)
        self.SimpleNavObject.PMatrix = sim_model.DoubleVector(PMatrix)
        self.SimpleNavObject.crossTrans = True
        self.SimpleNavObject.crossAtt = False

    # DYN EFFECTORS: RW and ACS Thrusters
    def SetReactionWheelDynObject(self):
        rwElAngle = 45.0 * np.pi / 180.0
        rwClockAngle = 45.0 * np.pi / 180.0
        rwType = 'Honeywell_HR16'
        modelTag = "ReactionWheels"
        self.rwDynObject.inputVehProps = "spacecraft_mass_props"

        def createPyramid(rwElAngle, rwClockAngle, N):
            Gs = []
            alpha = 2.0*np.pi / N
            for i in range(0, N):
                clockAngle = rwClockAngle + i * alpha
                gs_hat = [
                    -np.sin(rwElAngle) * np.sin(clockAngle),
                    -np.sin(rwElAngle) * np.cos(clockAngle),
                    -np.cos(rwElAngle)
                ]
                Gs.extend(gs_hat)
            return Gs

        x = 0.8 # [m]
        y = 0.8 # [m]
        z = 1.79070 # [m]

        N = 4
        r1_S = [x, y, z]
        r2_S = [x, -1.0*y, z]
        r3_s = [-1.0*x, -1.0*y, z]
        r4_s = [-1.0*x, y, z]
        Rs = (r1_S, r2_S, r3_s, r4_s)

        Gs = createPyramid(rwElAngle, rwClockAngle, N)
        omega_spin0 = 0.0 # [rpm]

        drw.clearSetup()
        drw.options.useRWfriction = False
        drw.options.useMinTorque = False
        drw.options.maxMomentum = 100    # Nms

        for i in range(0, N):
            drw.create(rwType, Gs[i:i+3], omega_spin0, Rs[i])
        drw.addToSpacecraft(modelTag, self.rwDynObject, self.VehDynObject)

    def SetACSThrusterDynObject(self):
        self.ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
        self.ACSThrusterDynObject.InputCmds = "acs_thruster_cmds"

        dth.clearSetup()
        thrusterType = 'MOOG_Monarc_1'

        N = 8
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
                dth.create(thrusterType, Rs_list[i], Gs_list[i])

            dth.addToSpacecraft(self.ACSThrusterDynObject.ModelTag,
                                self.ACSThrusterDynObject,
                                self.VehDynObject)
        else:
            raise ValueError('ACS Thrusters dyn effectors are not initialized properly.')

        ACSpropCM = [0.0, 0.0, 1.2]
        ACSpropMass = 40
        ACSpropRadius = 46.0 / 2.0 / 3.2808399 / 12.0
        sphereInerita = 2.0 / 5.0 * ACSpropMass * ACSpropRadius * ACSpropRadius
        ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
        self.ACSThrusterDynObject.objProps.Mass = ACSpropMass
        self.ACSThrusterDynObject.objProps.CoM = ACSpropCM
        self.ACSThrusterDynObject.objProps.InertiaTensor = ACSInertia
        self.ACSThrusterDynObject.inputProperties = "spacecraft_mass_props"

    # SENSORS: ST, IMU, CSS
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
        self.starTracker.ModelTag = "StarTrackerA"
        self.starTracker.walkBounds = sim_model.DoubleVector(errorBounds)
        self.starTracker.PMatrix = sim_model.DoubleVector(PMatrix)

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

        # Set up the sun sensor orientation information
        # Maybe we should add the method call to the SelfInit of the CSS module
        def initCSS(css_list, css_platform):
            N = len(css_list)
            i = 0
            for css in css_list:
                css.setStructureToPlatformDCM(css_platform[0], css_platform[1], css_platform[2])
                css.setUnitDirectionVectorWithPerturbation(0.0, 0.0)
                css.phi = 45.0 * np.pi / 180.0
                css.theta = (2*np.pi) * i / N
                i += 1

        # Note the re-use between different instances of the modules.
        # Handy but not required.
        CSSscaleFactor = 500.0E-6  # Scale factor (500 mu-amps) for max measurement
        CSSFOV = 90.0 * np.pi / 180.0  # 90 degree field of view
        (CSSNoiseStd, CSSNoiseBias, CSSKellyFactor) = turnOffCorruption_css()

        # Platform 1 is forward, platform 2 is back notionally
        CSSPlatform1YPR = [-np.pi / 2.0, -np.pi / 4.0, -np.pi / 2.0]
        CSSPlatform2YPR = [0.0, -np.pi / 2.0, 0.0]

        # Initialize one sensor by hand and then init the rest off of it
        CSSPyramid1HeadA = coarse_sun_sensor.CoarseSunSensor()
        CSSPyramid1HeadA.ModelTag = "CSSPyramid1HeadA"
        CSSPyramid1HeadA.OutputDataMsg = "coarse_sun_data_pyramid1_headA"
        CSSPyramid1HeadA.SenBias = CSSNoiseBias
        CSSPyramid1HeadA.SenNoiseStd = CSSNoiseStd
        CSSPyramid1HeadA.scaleFactor = CSSscaleFactor
        CSSPyramid1HeadA.fov = CSSFOV
        CSSPyramid1HeadA.KellyFactor = CSSKellyFactor

        CSSPyramid1HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadB.ModelTag = "CSSPyramid1HeadB"
        CSSPyramid1HeadB.OutputDataMsg = "coarse_sun_data_pyramid1_headB"
        CSSPyramid1HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadC.ModelTag = "CSSPyramid1HeadC"
        CSSPyramid1HeadC.OutputDataMsg = "coarse_sun_data_pyramid1_headC"
        CSSPyramid1HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid1HeadD.ModelTag = "CSSPyramid1HeadD"
        CSSPyramid1HeadD.OutputDataMsg = "coarse_sun_data_pyramid1_headD"

        css_list = (CSSPyramid1HeadA, CSSPyramid1HeadB, CSSPyramid1HeadC, CSSPyramid1HeadD)
        initCSS(css_list, CSSPlatform1YPR)


        CSSPyramid2HeadA = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadA)
        CSSPyramid2HeadA.ModelTag = "CSSPyramid2HeadA"
        CSSPyramid2HeadA.OutputDataMsg = "coarse_sun_data_pyramid2_headA"
        CSSPyramid2HeadB = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadB)
        CSSPyramid2HeadB.ModelTag = "CSSPyramid2HeadB"
        CSSPyramid2HeadB.OutputDataMsg = "coarse_sun_data_pyramid2_headB"
        CSSPyramid2HeadC = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadC)
        CSSPyramid2HeadC.ModelTag = "CSSPyramid2HeadC"
        CSSPyramid2HeadC.OutputDataMsg = "coarse_sun_data_pyramid2_headC"
        CSSPyramid2HeadD = coarse_sun_sensor.CoarseSunSensor(CSSPyramid1HeadD)
        CSSPyramid2HeadD.ModelTag = "CSSPyramid2HeadD"
        CSSPyramid2HeadD.OutputDataMsg = "coarse_sun_data_pyramid2_headD"

        css_list2 = (CSSPyramid2HeadA, CSSPyramid2HeadB, CSSPyramid2HeadC, CSSPyramid2HeadD)
        initCSS(css_list2, CSSPlatform2YPR)

        # Create Constellation
        self.cssConstellation.ModelTag = "CSSConstelation"
        self.cssConstellation.outputConstellationMessage = "css_sensors_data"
        self.cssConstellation.sensorList = coarse_sun_sensor.CSSVector([CSSPyramid1HeadA, CSSPyramid1HeadB, CSSPyramid1HeadC, CSSPyramid1HeadD,
            CSSPyramid2HeadA, CSSPyramid2HeadB, CSSPyramid2HeadC, CSSPyramid2HeadD])

    # ANTENNAS: Solar array antenna, Earth high gain antenna
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
        angSin = np.sin(23.0 * np.pi / 180.0)
        angCos = np.cos(23.0 * np.pi / 180.0)
        self.highGainBore.strBoreVec = [0.0, -angSin, angCos]


    def InitAllDynObjects(self):
        self.setClockSynchData()
        self.SetSpiceObject()
        self.SetVehDynObject()
        self.SetSimpleNavObject()
        self.SetVehOrbElemObject()

        self.SetReactionWheelDynObject()
        self.SetACSThrusterDynObject()

        self.SetStarTrackerData()
        self.SetIMUSensor()
        self.InitCSSHeads()

        #self.SetsolarArrayBore()
        #self.SethighGainBore()


if __name__ == "__main__":
    TheSim = HybridControlSim()