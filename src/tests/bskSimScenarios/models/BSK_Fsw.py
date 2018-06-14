''' '''
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
import math
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (vehicleConfigData, hillPoint, inertial3D, attTrackingError, MRP_Feedback,
                                    rwMotorTorque, fswMessages,
                                    velocityPoint, MRP_Steering, rateServoFullNonlinear)
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW


class BSKFswModels():
    def __init__(self, SimBase):
        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(0.1)  # 0.5

        # Create module data and module wraps
        self.vehicleData = vehicleConfigData.VehConfigInputData()
        self.vehicleWrap = SimBase.setModelDataWrap(self.vehicleData)
        self.vehicleWrap.ModelTag = "vehicleConfiguration"

        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        self.velocityPointData = velocityPoint.velocityPointConfig()
        self.velocityPointWrap = SimBase.setModelDataWrap(self.velocityPointData)
        self.velocityPointWrap.ModelTag  = "velocityPoint"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackControlData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackControlWrap = SimBase.setModelDataWrap(self.mrpFeedbackControlData)
        self.mrpFeedbackControlWrap.ModelTag = "mrpFeedbackControl"

        self.mrpFeedbackRWsData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.mrpSteeringData = MRP_Steering.MRP_SteeringConfig()
        self.mrpSteeringWrap = SimBase.setModelDataWrap(self.mrpSteeringData)
        self.mrpSteeringWrap.ModelTag = "MRP_Steering"

        self.rateServoData = rateServoFullNonlinear.rateServoFullNonlinearConfig()
        self.rateServoWrap = SimBase.setModelDataWrap(self.rateServoData)
        self.rateServoWrap.ModelTag = "rate_servo"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        # setup FSW Message names
        self.rwConfigMsgName = "rwa_config_data"
        self.rwCmdMsgName = "reactionwheel_cmds"
        self.attControlTorqueRaw = "controlTorqueRaw"
        self.attRateCmdMsgName = "rate_steering"
        self.inputRWSpeedsName = "reactionwheel_output_states"
        self.vehConfigMsgName = "adcs_config_data"
        self.attGuidanceMsgName = "guidanceOut"

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("initOnlyTask", int(1E10)), 1)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpSteeringRWsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("initOnlyTask", self.vehicleWrap, self.vehicleData, 2)

        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
        SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("velocityPointTask", self.velocityPointWrap, self.velocityPointData, 10)
        SimBase.AddModelToTask("velocityPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControlWrap, self.mrpFeedbackControlData, 10)

        SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteeringWrap, self.mrpSteeringData, 10)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServoWrap, self.rateServoData, 9)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)
        #masterSim.AddModelToTask("mrpFeedbackRWsTask", self.RWANullSpaceDataWrap,self.RWANullSpaceData, 7)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()
        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackTask')"])

        SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'hillPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpFeedbackTask')"])

        SimBase.createNewEvent("initiateVelocityPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'velocityPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('velocityPointTask')",
                                "self.enableTask('mrpFeedbackTask')"])

        SimBase.createNewEvent("initiateFeedbackRW", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'feedbackRW'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("initiateSteeringRW", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'steeringRW'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpSteeringRWsTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
        self.inertial3DData.outputDataName = "referenceOut"

    def SetHillPointGuidance(self, SimBase):
        self.hillPointData.outputDataName = "referenceOut"
        self.hillPointData.inputNavDataName = SimBase.DynModels.simpleNavObject.outputTransName
        self.hillPointData.inputCelMessName = SimBase.DynModels.earthGravBody.bodyInMsgName[:-12]


    def SetVelocityPointGuidance(self, SimBase):
        self.velocityPointData.outputDataName = "referenceOut"
        self.velocityPointData.inputNavDataName = SimBase.DynModels.simpleNavObject.outputTransName
        self.velocityPointData.inputCelMessName = SimBase.DynModels.earthGravBody.bodyInMsgName[:-12]
        self.velocityPointData.mu = SimBase.DynModels.earthGravBody.mu

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingErrorData.inputNavName = SimBase.DynModels.simpleNavObject.outputAttName
        # Note: SimBase.DynModels.simpleNavObject.outputAttName = "simple_att_nav_output"
        self.trackingErrorData.inputRefName = "referenceOut"
        self.trackingErrorData.outputDataName = self.attGuidanceMsgName

    def SetMRPFeedbackControl(self, SimBase):
        self.mrpFeedbackControlData.inputGuidName = self.attGuidanceMsgName
        self.mrpFeedbackControlData.vehConfigInMsgName = self.vehConfigMsgName
        self.mrpFeedbackControlData.outputDataName =  SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsgName
        # Note: SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsgName = "extTorquePntB_B_cmds"

        self.mrpFeedbackControlData.K = 3.5
        self.mrpFeedbackControlData.Ki = -1.0 # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControlData.P = 30.0
        self.mrpFeedbackControlData.integralLimit = 2. / self.mrpFeedbackControlData.Ki * 0.1
        self.mrpFeedbackControlData.domega0 = [0.0, 0.0, 0.0]


    def SetMRPFeedbackRWA(self):
        self.mrpFeedbackRWsData.K = 3.5
        self.mrpFeedbackRWsData.Ki = -1  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 30.0
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1
        self.mrpFeedbackRWsData.domega0 = [0.0, 0.0, 0.0]

        self.mrpFeedbackRWsData.vehConfigInMsgName = self.vehConfigMsgName
        self.mrpFeedbackRWsData.inputRWSpeedsName = self.inputRWSpeedsName
        self.mrpFeedbackRWsData.rwParamsInMsgName = self.rwConfigMsgName
        self.mrpFeedbackRWsData.inputGuidName = self.attGuidanceMsgName
        self.mrpFeedbackRWsData.outputDataName = self.attControlTorqueRaw

    def SetMRPSteering(self):
        self.mrpSteeringData.K1 = 0.05
        self.mrpSteeringData.ignoreOuterLoopFeedforward = False
        self.mrpSteeringData.K3 = 0.75
        self.mrpSteeringData.omega_max = 1.0 * mc.D2R
        self.mrpSteeringData.inputGuidName = self.attGuidanceMsgName
        self.mrpSteeringData.outputDataName = self.attRateCmdMsgName

    def SetRateServo(self):
        self.rateServoData.inputGuidName = self.attGuidanceMsgName
        self.rateServoData.vehConfigInMsgName = self.vehConfigMsgName
        self.rateServoData.rwParamsInMsgName = self.rwConfigMsgName
        self.rateServoData.inputRWSpeedsName = self.inputRWSpeedsName
        self.rateServoData.inputRateSteeringName = self.attRateCmdMsgName
        self.rateServoData.outputDataName = self.attControlTorqueRaw
        self.rateServoData.Ki = 5.0
        self.rateServoData.P = 150.0
        self.rateServoData.integralLimit = 2. / self.rateServoData.Ki * 0.1
        self.rateServoData.knownTorquePntB_B = [0., 0., 0.]


    def SetVehicleConfiguration(self, SimBase):
        self.vehicleData.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vehicleData.CoM_B = [0.0, 0.0, 1.0]
        self.vehicleData.outputPropsName = self.vehConfigMsgName

    def SetRWConfigMsg(self, SimBase):
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)

        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              wheelJs,  # kg*m^2
                              0.2)  # Nm        uMax

        fswSetupRW.writeConfigMessage(self.rwConfigMsgName, SimBase.TotalSim, SimBase.FSWProcessName)


    def SetRWMotorTorque(self):
        controlAxes_B = [
            1.0, 0.0, 0.0
            , 0.0, 1.0, 0.0
            , 0.0, 0.0, 1.0
        ]
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.inputVehControlName = self.attControlTorqueRaw
        self.rwMotorTorqueData.outputDataName = self.rwCmdMsgName
        self.rwMotorTorqueData.rwParamsInMsgName = self.rwConfigMsgName

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetInertial3DPointGuidance()
        self.SetHillPointGuidance(SimBase)
        self.SetVelocityPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackControl(SimBase)
        self.SetVehicleConfiguration(SimBase)
        self.SetRWConfigMsg(SimBase)
        self.SetMRPFeedbackRWA()
        self.SetRWMotorTorque()
        self.SetMRPSteering()
        self.SetRateServo()


#BSKFswModels()