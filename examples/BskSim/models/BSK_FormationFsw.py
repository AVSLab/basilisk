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

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque,
                                    spacecraftPointing)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import deprecated
from Basilisk.utilities import macros as mc


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

        # Create modules
        self.inertial3D = inertial3D.inertial3D()
        self.inertial3D.ModelTag = "inertial3D"

        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        self.inertial3D2 = inertial3D.inertial3D()
        self.inertial3D2.ModelTag = "inertial3D2"

        self.trackingError2 = attTrackingError.attTrackingError()
        self.trackingError2.ModelTag = "trackingError2"

        self.mrpFeedbackRWs2 = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs2.ModelTag = "mrpFeedbackRWs2"

        self.rwMotorTorque2 = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque2.ModelTag = "rwMotorTorque2"

        self.spacecraftPointing = spacecraftPointing.spacecraftPointing()
        self.spacecraftPointing.ModelTag = "spacecraftPointing"

        self.mrpFeedbackControl = mrpFeedback.mrpFeedback()
        self.mrpFeedbackControl.ModelTag = "mrpFeedbackControl"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3D, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWs, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorque, 8)


        SimBase.AddModelToTask("inertial3DPointTask2", self.inertial3D2, 10)
        SimBase.AddModelToTask("inertial3DPointTask2", self.trackingError2, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.mrpFeedbackRWs2, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.rwMotorTorque2,8)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControl, 10)

        SimBase.AddModelToTask("spacecraftPointingTask", self.spacecraftPointing, 8)
        SimBase.AddModelToTask("spacecraftPointingTask", self.trackingError2, 7)

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
        self.inertial3D.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3D.attRefOutMsg, self.attRefMsg)

        self.inertial3D2.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3D2.attRefOutMsg, self.attRef2Msg)

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingError.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)

        self.trackingError2.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject2.attOutMsg)
        self.trackingError2.attRefInMsg.subscribeTo(self.attRef2Msg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError2.attGuidOutMsg, self.attGuid2Msg)

    def SetMRPFeedbackRWA(self, SimBase):
        self.mrpFeedbackRWs.K = 3.5
        self.mrpFeedbackRWs.Ki = -1
        self.mrpFeedbackRWs.P = 30.0
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1
        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWs.cmdTorqueOutMsg, self.cmdTorqueMsg)

        self.mrpFeedbackRWs2.K = 3.5
        self.mrpFeedbackRWs2.Ki = -1  # TURN OFF IN CASE OF RUNNING Inertial3D!!!
        self.mrpFeedbackRWs2.P = 30.0
        self.mrpFeedbackRWs2.integralLimit = 2. / self.mrpFeedbackRWs2.Ki * 0.1
        self.mrpFeedbackRWs2.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWs2.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector2.rwSpeedOutMsg)
        self.mrpFeedbackRWs2.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs2.guidInMsg.subscribeTo(self.attGuid2Msg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWs2.cmdTorqueOutMsg, self.cmdTorque2Msg)

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

        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.cmdTorqueMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorque.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

        self.rwMotorTorque2.controlAxes_B = controlAxes_B
        self.rwMotorTorque2.vehControlInMsg.subscribeTo(self.cmdTorque2Msg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorque2.rwMotorTorqueOutMsg, self.cmdRwMotor2Msg)
        self.rwMotorTorque2.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

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
        self.mrpFeedbackControl.guidInMsg.subscribeTo(self.attGuid2Msg)
        self.mrpFeedbackControl.vehConfigInMsg.subscribeTo(self.vcMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackControl.cmdTorqueOutMsg, self.cmdTorqueDirectMsg)

        self.mrpFeedbackControl.K = 10.0
        self.mrpFeedbackControl.Ki = 0.0001  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControl.P = 30.0
        self.mrpFeedbackControl.integralLimit = 2. / self.mrpFeedbackControl.Ki * 0.1

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


    @property
    def inertial3DData(self):
        return self.inertial3D

    inertial3DData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DData as inertial3D",
        inertial3DData)

    @property
    def inertial3DWrap(self):
        return self.inertial3D

    inertial3DWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DWrap as inertial3D",
        inertial3DWrap)


    @property
    def trackingErrorData(self):
        return self.trackingError

    trackingErrorData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorData as trackingError",
        trackingErrorData)

    @property
    def trackingErrorWrap(self):
        return self.trackingError

    trackingErrorWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorWrap as trackingError",
        trackingErrorWrap)


    @property
    def mrpFeedbackRWsData(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsData as mrpFeedbackRWs",
        mrpFeedbackRWsData)

    @property
    def mrpFeedbackRWsWrap(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsWrap as mrpFeedbackRWs",
        mrpFeedbackRWsWrap)


    @property
    def rwMotorTorqueData(self):
        return self.rwMotorTorque

    rwMotorTorqueData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueData as rwMotorTorque",
        rwMotorTorqueData)

    @property
    def rwMotorTorqueWrap(self):
        return self.rwMotorTorque

    rwMotorTorqueWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueWrap as rwMotorTorque",
        rwMotorTorqueWrap)

    @property
    def inertial3DData2(self):
        return self.inertial3D2

    inertial3DData2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DData2 as inertial3D2",
        inertial3DData2)

    @property
    def inertial3DWrap2(self):
        return self.inertial3D2

    inertial3DWrap2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DWrap2 as inertial3D2",
        inertial3DWrap2)


    @property
    def trackingErrorData2(self):
        return self.trackingError2

    trackingErrorData2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorData2 as trackingError2",
        trackingErrorData2)

    @property
    def trackingErrorWrap2(self):
        return self.trackingError2

    trackingErrorWrap2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorWrap2 as trackingError2",
        trackingErrorWrap2)


    @property
    def mrpFeedbackRWsData2(self):
        return self.mrpFeedbackRWs2

    mrpFeedbackRWsData2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsData2 as mrpFeedbackRWs2",
        mrpFeedbackRWsData2)

    @property
    def mrpFeedbackRWsWrap2(self):
        return self.mrpFeedbackRWs2

    mrpFeedbackRWsWrap2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsWrap2 as mrpFeedbackRWs2",
        mrpFeedbackRWsWrap2)


    @property
    def rwMotorTorqueData2(self):
        return self.rwMotorTorque2

    rwMotorTorqueData2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueData2 as rwMotorTorque2",
        rwMotorTorqueData2)

    @property
    def rwMotorTorqueWrap2(self):
        return self.rwMotorTorque2

    rwMotorTorqueWrap2 = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueWrap2 as rwMotorTorque2",
        rwMotorTorqueWrap2)
    
    @property
    def mrpFeedbackControlData(self):
        return self.mrpFeedbackControl

    mrpFeedbackControlData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackControlData as mrpFeedbackControl",
        mrpFeedbackControlData)

    @property
    def mrpFeedbackControlWrap(self):
        return self.mrpFeedbackControl

    mrpFeedbackControlWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackControlWrap as mrpFeedbackControl",
        mrpFeedbackControlWrap)

    @property
    def spacecraftPointingWrap(self):
        return self.spacecraftPointing

    spacecraftPointingWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to spacecraftPointingWrap as spacecraftPointing",
        spacecraftPointingWrap)