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

from Basilisk.fswAlgorithms import (hillPoint, inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque,
                                    velocityPoint, mrpSteering, rateServoFullNonlinear,
                                    sunSafePoint, cssWlsEst)

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW

from Basilisk.architecture import messaging


class BSKFswModels:
    """Defines the bskSim FSW class"""
    def __init__(self, SimBase, fswRate):
        # define empty class variables
        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create module data and module wraps
        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        self.sunSafePointData = sunSafePoint.sunSafePointConfig()
        self.sunSafePointWrap = SimBase.setModelDataWrap(self.sunSafePointData)
        self.sunSafePointWrap.ModelTag = "sunSafePoint"

        self.velocityPointData = velocityPoint.velocityPointConfig()
        self.velocityPointWrap = SimBase.setModelDataWrap(self.velocityPointData)
        self.velocityPointWrap.ModelTag  = "velocityPoint"

        self.cssWlsEstData = cssWlsEst.CSSWLSConfig()
        self.cssWlsEstWrap = SimBase.setModelDataWrap(self.cssWlsEstData)
        self.cssWlsEstWrap.ModelTag = "cssWlsEst"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackControlData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackControlWrap = SimBase.setModelDataWrap(self.mrpFeedbackControlData)
        self.mrpFeedbackControlWrap.ModelTag = "mrpFeedbackControl"

        self.mrpFeedbackRWsData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.mrpSteeringData = mrpSteering.mrpSteeringConfig()
        self.mrpSteeringWrap = SimBase.setModelDataWrap(self.mrpSteeringData)
        self.mrpSteeringWrap.ModelTag = "MRP_Steering"

        self.rateServoData = rateServoFullNonlinear.rateServoFullNonlinearConfig()
        self.rateServoWrap = SimBase.setModelDataWrap(self.rateServoData)
        self.rateServoWrap.ModelTag = "rate_servo"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpSteeringRWsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
        SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("sunSafePointTask", self.cssWlsEstWrap, self.cssWlsEstData, 10)
        SimBase.AddModelToTask("sunSafePointTask", self.sunSafePointWrap, self.sunSafePointData, 9)

        SimBase.AddModelToTask("velocityPointTask", self.velocityPointWrap, self.velocityPointData, 10)
        SimBase.AddModelToTask("velocityPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControlWrap, self.mrpFeedbackControlData, 10)

        SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteeringWrap, self.mrpSteeringData, 10)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServoWrap, self.rateServoData, 9)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.setAllButCurrentEventActivity('initiateStandby', True)"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateAttitudeGuidance', True)"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidanceDirect", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'directInertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackTask')",
                                "self.setAllButCurrentEventActivity('initiateAttitudeGuidanceDirect', True)"
                                ])

        SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'hillPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateHillPoint', True)"
                                ])

        SimBase.createNewEvent("initiateSunSafePoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'sunSafePoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('sunSafePointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateSunSafePoint', True)"
                                ])

        SimBase.createNewEvent("initiateVelocityPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'velocityPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('velocityPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateVelocityPoint', True)"])

        SimBase.createNewEvent("initiateSteeringRW", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'steeringRW'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateSteeringRW', True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        """Define the inertial 3D guidance module"""
        self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DData.attRefOutMsg, self.attRefMsg)

    def SetHillPointGuidance(self, SimBase):
        """Define the Hill pointing guidance module"""
        self.hillPointData.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.hillPointData.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])  # earth
        messaging.AttRefMsg_C_addAuthor(self.hillPointData.attRefOutMsg, self.attRefMsg)

    def SetSunSafePointGuidance(self, SimBase):
        """Define the sun safe pointing guidance module"""
        self.sunSafePointData.imuInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.sunSafePointData.sunDirectionInMsg.subscribeTo(self.cssWlsEstData.navStateOutMsg)
        self.sunSafePointData.sHatBdyCmd = [0.0, 0.0, 1.0]
        messaging.AttGuidMsg_C_addAuthor(self.sunSafePointData.attGuidanceOutMsg, self.attGuidMsg)

    def SetVelocityPointGuidance(self, SimBase):
        """Define the velocity pointing guidance module"""
        self.velocityPointData.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.velocityPointData.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])
        self.velocityPointData.mu = SimBase.DynModels.gravFactory.gravBodies['earth'].mu
        messaging.AttRefMsg_C_addAuthor(self.velocityPointData.attRefOutMsg, self.attRefMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """Define the attitude tracking error module"""
        self.trackingErrorData.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingErrorData.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingErrorData.attGuidOutMsg, self.attGuidMsg)

    def SetCSSWlsEst(self, SimBase):
        """Set the FSW CSS configuration information """
        cssConfig = messaging.CSSConfigMsgPayload()
        totalCSSList = []
        nHat_B_vec = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        for CSSHat in nHat_B_vec:
            CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
            CSSConfigElement.CBias = 1.0
            CSSConfigElement.nHat_B = CSSHat
            totalCSSList.append(CSSConfigElement)
        cssConfig.cssVals = totalCSSList

        cssConfig.nCSS = len(nHat_B_vec)
        self.cssConfigMsg = messaging.CSSConfigMsg().write(cssConfig)

        self.cssWlsEstData.cssDataInMsg.subscribeTo(SimBase.DynModels.CSSConstellationObject.constellationOutMsg)
        self.cssWlsEstData.cssConfigInMsg.subscribeTo(self.cssConfigMsg)

    def SetMRPFeedbackControl(self, SimBase):
        """Set the MRP feedback module configuration"""
        self.mrpFeedbackControlData.guidInMsg.subscribeTo(self.attGuidMsg)
        self.mrpFeedbackControlData.vehConfigInMsg.subscribeTo(self.vcMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackControlData.cmdTorqueOutMsg, self.cmdTorqueDirectMsg)

        self.mrpFeedbackControlData.K = 3.5
        self.mrpFeedbackControlData.Ki = -1.0  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControlData.P = 30.0
        self.mrpFeedbackControlData.integralLimit = 2. / self.mrpFeedbackControlData.Ki * 0.1

    def SetMRPFeedbackRWA(self, SimBase):
        """Set the MRP feedback information if RWs are considered"""
        self.mrpFeedbackRWsData.K = 3.5
        self.mrpFeedbackRWsData.Ki = -1  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 30.0
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1

        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWsData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWsData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWsData.cmdTorqueOutMsg, self.cmdTorqueMsg)

    def SetMRPSteering(self):
        """Set the MRP Steering module"""
        self.mrpSteeringData.K1 = 0.05
        self.mrpSteeringData.ignoreOuterLoopFeedforward = False
        self.mrpSteeringData.K3 = 0.75
        self.mrpSteeringData.omega_max = 1.0 * mc.D2R
        self.mrpSteeringData.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRateServo(self, SimBase):
        """Set the rate servo module"""
        self.rateServoData.guidInMsg.subscribeTo(self.attGuidMsg)
        self.rateServoData.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.rateServoData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.rateServoData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.rateServoData.rateSteeringInMsg.subscribeTo(self.mrpSteeringData.rateCmdOutMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.rateServoData.cmdTorqueOutMsg, self.cmdTorqueMsg)

        self.rateServoData.Ki = 5.0
        self.rateServoData.P = 150.0
        self.rateServoData.integralLimit = 2. / self.rateServoData.Ki * 0.1
        self.rateServoData.knownTorquePntB_B = [0., 0., 0.]

    def SetVehicleConfiguration(self):
        """Set the spacecraft configuration information"""
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        # use the same inertia in the FSW algorithm as in the simulation
        vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetRWConfigMsg(self):
        """Set the RW device information"""
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

        self.fswRwConfigMsg = fswSetupRW.writeConfigMessage()

    def SetRWMotorTorque(self):
        """Set the RW motor torque information"""
        controlAxes_B = [
            1.0, 0.0, 0.0
            , 0.0, 1.0, 0.0
            , 0.0, 0.0, 1.0
        ]
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.cmdTorqueMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorqueData.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """Initialize all the FSW objects"""

        # note that the order in which these routines are called is important.
        # To subscribe to a message that message must already exit.
        self.SetVehicleConfiguration()
        self.SetRWConfigMsg()
        self.SetInertial3DPointGuidance()
        self.SetHillPointGuidance(SimBase)
        self.SetCSSWlsEst(SimBase)
        self.SetSunSafePointGuidance(SimBase)
        self.SetVelocityPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackControl(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetMRPSteering()
        self.SetRateServo(SimBase)
        self.SetRWMotorTorque()

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorqueDirectMsg = messaging.CmdTorqueBodyMsg_C()
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.cmdRwMotorMsg = messaging.ArrayMotorTorqueMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsg.subscribeTo(self.cmdTorqueDirectMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotorMsg)

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.cmdTorqueMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorqueDirectMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())


