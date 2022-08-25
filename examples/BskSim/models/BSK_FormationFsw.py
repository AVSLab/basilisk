#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import math
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque,
                                    spacecraftPointing)
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging

class BSKFswModels():
    def __init__(self, SimBase, fswRate):
        # define empty class variables
        self.vcMsg = None
        self.fswRwConfigMsg = None

        self.cmdTorqueMsg = None
        self.cmdTorque2Msg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attRef2Msg = None
        self.attGuidMsg = None
        self.attGuid2Msg = None
        self.cmdRwMotorMsg = None
        self.cmdRwMotor2Msg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)

        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask2", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask2", self.processTasksTimeStep), 10)

        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("spacecraftPointingTask", self.processTasksTimeStep))

        # Create module data and module wraps
        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackRWsData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        self.inertial3DData2 = inertial3D.inertial3DConfig()
        self.inertial3DWrap2 = SimBase.setModelDataWrap(self.inertial3DData2)
        self.inertial3DWrap2.ModelTag = "inertial3D2"

        self.trackingErrorData2 = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap2 = SimBase.setModelDataWrap(self.trackingErrorData2)
        self.trackingErrorWrap2.ModelTag = "trackingError2"

        self.mrpFeedbackRWsData2 = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap2 = SimBase.setModelDataWrap(self.mrpFeedbackRWsData2)
        self.mrpFeedbackRWsWrap2.ModelTag = "mrpFeedbackRWs2"

        self.rwMotorTorqueData2 = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap2 = SimBase.setModelDataWrap(self.rwMotorTorqueData2)
        self.rwMotorTorqueWrap2.ModelTag = "rwMotorTorque2"

        self.spacecraftPointing = spacecraftPointing.spacecraftPointingConfig()
        self.spacecraftPointingWrap = SimBase.setModelDataWrap(self.spacecraftPointing)
        self.spacecraftPointingWrap.ModelTag = "spacecraftPointing"

        self.mrpFeedbackControlData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackControlWrap = SimBase.setModelDataWrap(self.mrpFeedbackControlData)
        self.mrpFeedbackControlWrap.ModelTag = "mrpFeedbackControl"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)


        SimBase.AddModelToTask("inertial3DPointTask2", self.inertial3DWrap2, self.inertial3DData2, 10)
        SimBase.AddModelToTask("inertial3DPointTask2", self.trackingErrorWrap2, self.trackingErrorData2, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.mrpFeedbackRWsWrap2, self.mrpFeedbackRWsData2, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.rwMotorTorqueWrap2, self.rwMotorTorqueData2, 8)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControlWrap, self.mrpFeedbackControlData, 10)

        SimBase.AddModelToTask("spacecraftPointingTask", self.spacecraftPointingWrap, self.spacecraftPointing, 8)
        SimBase.AddModelToTask("spacecraftPointingTask", self.trackingErrorWrap2, self.trackingErrorData2, 7)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('inertial3DPointTask2')",
                                "self.enableTask('mrpFeedbackRWsTask2')"])

        SimBase.createNewEvent("initiateSpacecraftPointing", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'spacecraftPointing'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('spacecraftPointingTask')",
                                "self.enableTask('mrpFeedbackTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DData.attRefOutMsg, self.attRefMsg)

        self.inertial3DData2.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DData2.attRefOutMsg, self.attRef2Msg)

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingErrorData.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingErrorData.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingErrorData.attGuidOutMsg, self.attGuidMsg)

        self.trackingErrorData2.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject2.attOutMsg)
        self.trackingErrorData2.attRefInMsg.subscribeTo(self.attRef2Msg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingErrorData2.attGuidOutMsg, self.attGuid2Msg)

    def SetMRPFeedbackRWA(self, SimBase):
        self.mrpFeedbackRWsData.K = 3.5
        self.mrpFeedbackRWsData.Ki = -1
        self.mrpFeedbackRWsData.P = 30.0
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1
        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWsData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWsData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWsData.cmdTorqueOutMsg, self.cmdTorqueMsg)

        self.mrpFeedbackRWsData2.K = 3.5
        self.mrpFeedbackRWsData2.Ki = -1  # TURN OFF IN CASE OF RUNNING Inertial3D!!!
        self.mrpFeedbackRWsData2.P = 30.0
        self.mrpFeedbackRWsData2.integralLimit = 2. / self.mrpFeedbackRWsData2.Ki * 0.1
        self.mrpFeedbackRWsData2.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWsData2.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector2.rwSpeedOutMsg)
        self.mrpFeedbackRWsData2.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData2.guidInMsg.subscribeTo(self.attGuid2Msg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWsData2.cmdTorqueOutMsg, self.cmdTorque2Msg)

    def SetRWConfigMsg(self):
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        # the same msg is used here for both spacecraft
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              wheelJs,  # kg*m^2
                              0.2)  # Nm        uMax
        self.fswRwConfigMsg = fswSetupRW.writeConfigMessage()

    def SetRWMotorTorque(self):
        controlAxes_B = [
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]

        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.cmdTorqueMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorqueData.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

        self.rwMotorTorqueData2.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData2.vehControlInMsg.subscribeTo(self.cmdTorque2Msg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorqueData2.rwMotorTorqueOutMsg, self.cmdRwMotor2Msg)
        self.rwMotorTorqueData2.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    def SetSpacecraftPointing(self, SimBase):
        self.spacecraftPointing.chiefPositionInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.spacecraftPointing.deputyPositionInMsg.subscribeTo(SimBase.DynModels.simpleNavObject2.transOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.spacecraftPointing.attReferenceOutMsg, self.attRef2Msg)
        self.spacecraftPointing.alignmentVector_B = [1.0, 2.0, 3.0]

    def SetVehicleConfiguration(self):
        # use the same inertia in the FSW algorithm as in the simulation
        vcData = messaging.VehicleConfigMsgPayload()
        vcData.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vcMsg = messaging.VehicleConfigMsg().write(vcData)

    def SetMRPFeedbackControl(self):
        self.mrpFeedbackControlData.guidInMsg.subscribeTo(self.attGuid2Msg)
        self.mrpFeedbackControlData.vehConfigInMsg.subscribeTo(self.vcMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackControlData.cmdTorqueOutMsg, self.cmdTorqueDirectMsg)

        self.mrpFeedbackControlData.K = 10.0
        self.mrpFeedbackControlData.Ki = 0.0001  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControlData.P = 30.0
        self.mrpFeedbackControlData.integralLimit = 2. / self.mrpFeedbackControlData.Ki * 0.1

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetInertial3DPointGuidance()
        self.SetAttitudeTrackingError(SimBase)
        self.SetVehicleConfiguration()
        self.SetRWConfigMsg()
        self.SetMRPFeedbackRWA(SimBase)
        self.SetRWMotorTorque()
        self.SetSpacecraftPointing(SimBase)
        self.SetMRPFeedbackControl()

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorque2Msg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorqueDirectMsg = messaging.CmdTorqueBodyMsg_C()
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attRef2Msg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.attGuid2Msg = messaging.AttGuidMsg_C()
        self.cmdRwMotorMsg = messaging.ArrayMotorTorqueMsg_C()
        self.cmdRwMotor2Msg = messaging.ArrayMotorTorqueMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels.extForceTorqueObject2.cmdTorqueInMsg.subscribeTo(self.cmdTorqueDirectMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotorMsg)
        SimBase.DynModels.rwStateEffector2.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotor2Msg)

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.cmdTorqueMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorque2Msg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorqueDirectMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attRef2Msg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.attGuid2Msg.write(messaging.AttGuidMsgPayload())
        self.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.cmdRwMotor2Msg.write(messaging.ArrayMotorTorqueMsgPayload())


