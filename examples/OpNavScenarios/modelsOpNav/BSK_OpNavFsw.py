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
r"""
Overview
--------

``OpNavScenarios/models/BSK_OpNavFsw.py`` contains the FSW algorithms used in the scenarios. Examples are the
Orbit Determination
filters, the pointing guidance module, the CNN module, and more. This file also contains
the ``modeRequest`` definitions which enable all the tasks necessary to perform a specific action.



"""


import math

import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (
    attTrackingError,
    faultDetection,
    headingSuKF,
    hillPoint,
    horizonOpNav,
    mrpFeedback,
    opNavPoint,
    pixelLineBiasUKF,
    pixelLineConverter,
    relativeODuKF,
    rwMotorTorque,
)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import deprecated, fswSetupRW, macros, orbitalMotion

bskPath = __path__[0]

try:
    from Basilisk.fswAlgorithms import houghCircles, limbFinding  # FSW for OpNav
except ImportError:
    print("OpNav Modules Missing, check build options")

try:
    from Basilisk.fswAlgorithms import centerRadiusCNN  # FSW for OpNav
    centerRadiusCNNIncluded = True
except ImportError:
    centerRadiusCNNIncluded = False


class BSKFswModels():
    """
    OpNav BSK FSW Models
    """
    def __init__(self, SimBase, fswRate):
        # define empty class variables
        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.attGuidMsg = None
        self.opnavMsg = None
        self.opnavPrimaryMsg = None
        self.opnavSecondaryMsg = None
        self.opnavCirclesMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = macros.sec2nano(fswRate)

        # Create module data and module wraps
        self.hillPoint = hillPoint.hillPoint()
        self.hillPoint.ModelTag = "hillPoint"

        self.opNavPoint = opNavPoint.opNavPoint()
        self.opNavPoint.ModelTag = "opNavPoint"

        self.trackingErrorCam = attTrackingError.attTrackingError()
        self.trackingErrorCam.ModelTag = "trackingErrorCam"

        self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        self.imageProcessing = houghCircles.HoughCircles()
        self.imageProcessing.ModelTag = "houghCircles"

        if centerRadiusCNNIncluded:
            self.opNavCNN = centerRadiusCNN.CenterRadiusCNN()
            self.opNavCNN.ModelTag = "opNavCNN"

        self.pixelLine = pixelLineConverter.pixelLineConverter()
        self.pixelLine.ModelTag = "pixelLine"

        self.opNavFault = faultDetection.faultDetection()
        self.opNavFault.ModelTag = "OpNav_Fault"

        self.limbFinding = limbFinding.LimbFinding()
        self.limbFinding.ModelTag = "limbFind"

        self.horizonNav = horizonOpNav.horizonOpNav()
        self.horizonNav.ModelTag = "limbNav"

        self.relativeOD = relativeODuKF.relativeODuKF()
        self.relativeOD.ModelTag = "relativeOD"

        self.pixelLineFilter = pixelLineBiasUKF.pixelLineBiasUKF()
        self.pixelLineFilter.ModelTag = "pixelLineFilter"

        self.headingUKF = headingSuKF.headingSuKF()
        self.headingUKF.ModelTag = "headingUKF"

        # create the FSW module gateway messages
        self.setupGatewayMsgs()

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("headingPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavPointLimbTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavAttODLimbTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavPointTaskCheat", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 15)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavODTask", self.processTasksTimeStep), 5)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("imageProcTask", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavODTaskLimb", self.processTasksTimeStep), 15)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavODTaskB", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavAttODTask", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("cnnAttODTask", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("opNavFaultDet", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("attODFaultDet", self.processTasksTimeStep), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("cnnFaultDet", self.processTasksTimeStep), 9)

        SimBase.AddModelToTask("opNavPointTask", self.imageProcessing, 15)
        SimBase.AddModelToTask("opNavPointTask", self.pixelLine, 12)
        SimBase.AddModelToTask("opNavPointTask", self.opNavPoint, 9)

        SimBase.AddModelToTask("headingPointTask", self.imageProcessing, 15)
        SimBase.AddModelToTask("headingPointTask", self.pixelLine, 12)
        SimBase.AddModelToTask("headingPointTask", self.headingUKF, 10)
        SimBase.AddModelToTask("headingPointTask", self.opNavPoint, 9)

        SimBase.AddModelToTask("opNavPointLimbTask", self.limbFinding, 25)
        SimBase.AddModelToTask("opNavPointLimbTask", self.horizonNav, 12)
        SimBase.AddModelToTask("opNavPointLimbTask", self.opNavPoint, 10)

        SimBase.AddModelToTask("opNavAttODLimbTask", self.limbFinding, 25)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.horizonNav, 12)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.opNavPoint, 10)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.relativeOD, 9)

        SimBase.AddModelToTask("opNavODTaskLimb", self.limbFinding, 25)
        SimBase.AddModelToTask("opNavODTaskLimb", self.horizonNav, 22)
        SimBase.AddModelToTask("opNavODTaskLimb", self.relativeOD, 20)

        SimBase.AddModelToTask("opNavPointTaskCheat", self.hillPoint, 10)
        SimBase.AddModelToTask("opNavPointTaskCheat", self.trackingErrorCam, 9)

        SimBase.AddModelToTask("opNavODTask", self.imageProcessing, 15)
        SimBase.AddModelToTask("opNavODTask", self.pixelLine, 14)
        SimBase.AddModelToTask("opNavODTask", self.relativeOD, 13)

        SimBase.AddModelToTask("opNavODTaskB", self.imageProcessing, 15)
        SimBase.AddModelToTask("opNavODTaskB", self.pixelLineFilter, 13)

        SimBase.AddModelToTask("imageProcTask", self.imageProcessing, 15)

        SimBase.AddModelToTask("opNavAttODTask", self.imageProcessing, 15)
        SimBase.AddModelToTask("opNavAttODTask", self.pixelLine, 14)
        SimBase.AddModelToTask("opNavAttODTask", self.opNavPoint, 10)
        SimBase.AddModelToTask("opNavAttODTask", self.relativeOD, 9)

        if centerRadiusCNNIncluded:
            SimBase.AddModelToTask("cnnAttODTask", self.opNavCNN, 15)
        SimBase.AddModelToTask("cnnAttODTask", self.pixelLine, 14)
        SimBase.AddModelToTask("cnnAttODTask", self.opNavPoint, 10)
        SimBase.AddModelToTask("cnnAttODTask", self.relativeOD, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWs, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorque, 8)

        SimBase.AddModelToTask("attODFaultDet", self.limbFinding, 25)
        SimBase.AddModelToTask("attODFaultDet", self.horizonNav, 20)
        SimBase.AddModelToTask("attODFaultDet", self.imageProcessing, 18)
        SimBase.AddModelToTask("attODFaultDet", self.pixelLine, 16)
        SimBase.AddModelToTask("attODFaultDet", self.opNavFault, 14)
        SimBase.AddModelToTask("attODFaultDet", self.opNavPoint, 10)
        SimBase.AddModelToTask("attODFaultDet", self.relativeOD, 9)

        SimBase.AddModelToTask("opNavFaultDet", self.limbFinding, 25)
        SimBase.AddModelToTask("opNavFaultDet", self.horizonNav, 20)
        SimBase.AddModelToTask("opNavFaultDet", self.imageProcessing, 18)
        SimBase.AddModelToTask("opNavFaultDet", self.pixelLine, 16)
        SimBase.AddModelToTask("opNavFaultDet", self.opNavFault, 14)
        SimBase.AddModelToTask("opNavFaultDet", self.relativeOD, 9)

        if centerRadiusCNNIncluded:
            SimBase.AddModelToTask("cnnFaultDet", self.opNavCNN, 25)
            SimBase.AddModelToTask("cnnFaultDet", self.pixelLine, 20)
            SimBase.AddModelToTask("cnnFaultDet", self.imageProcessing, 18)
            SimBase.AddModelToTask("cnnFaultDet", self.pixelLine, 16)
            SimBase.AddModelToTask("cnnFaultDet", self.opNavFault, 14)
            SimBase.AddModelToTask("cnnFaultDet", self.opNavPoint, 10)
            SimBase.AddModelToTask("cnnFaultDet", self.relativeOD, 9)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent(
            "initiateStandby",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "standby",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
            ),
        )

        SimBase.createNewEvent(
            "prepOpNav",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "prepOpNav",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "imageGen",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "imageGen",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("imageProcTask"),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "pointOpNav",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "pointOpNav",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "pointHead",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "pointHead",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("headingPointTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "pointLimb",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "pointLimb",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointLimbTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "OpNavOD",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "OpNavOD",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
                self.enableTask("opNavODTask"),
            ),
        )

        SimBase.createNewEvent(
            "OpNavODLimb",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "OpNavODLimb",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
                self.enableTask("opNavODTaskLimb"),
            ),
        )

        SimBase.createNewEvent(
            "OpNavODB",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "OpNavODB",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
                self.enableTask("opNavODTaskB"),
            ),
        )

        SimBase.createNewEvent(
            "OpNavAttOD",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "OpNavAttOD",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavAttODTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "OpNavAttODLimb",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "OpNavAttODLimb",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavAttODLimbTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "CNNAttOD",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "CNNAttOD",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("cnnAttODTask"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "FaultDet",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "FaultDet",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("attODFaultDet"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

        SimBase.createNewEvent(
            "ODFaultDet",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "ODFaultDet",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("opNavPointTaskCheat"),
                self.enableTask("mrpFeedbackRWsTask"),
                self.enableTask("opNavFaultDet"),
            ),
        )

        SimBase.createNewEvent(
            "FaultDetCNN",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: self.modeRequest == "FaultDetCNN",
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                self.enableTask("cnnFaultDet"),
                self.enableTask("mrpFeedbackRWsTask"),
            ),
        )

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetHillPointGuidance(self, SimBase):
        self.hillPoint.transNavInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.transOutMsg)
        self.hillPoint.celBodyInMsg.subscribeTo(SimBase.DynModels.ephemObject.ephemOutMsgs[0])

    def SetOpNavPointGuidance(self, SimBase):
        messaging.AttGuidMsg_C_addAuthor(self.opNavPoint.attGuidanceOutMsg, self.attGuidMsg)
        self.opNavPoint.imuInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.opNavPoint.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.opNavPoint.opnavDataInMsg.subscribeTo(self.opnavMsg)
        self.opNavPoint.smallAngle = 0.001*np.pi/180.
        self.opNavPoint.timeOut = 1000  # Max time in sec between images before engaging search
        # self.opNavPointData.opNavAxisSpinRate = 0.1*np.pi/180.
        self.opNavPoint.omega_RN_B = [0.001, 0.0, -0.001]
        self.opNavPoint.alignAxis_C = [0., 0., 1]

    def SetHeadingUKF(self, SimBase):
        self.headingUKF.opnavDataInMsg.subscribeTo(self.opnavMsg)
        self.headingUKF.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)

        self.headingUKF.alpha = 0.02
        self.headingUKF.beta = 2.0
        self.headingUKF.kappa = 0.0

        self.headingUKF.state = [0.0, 0., 0., 0., 0.]
        self.headingUKF.stateInit = [0.0, 0.0, 1.0, 0.0, 0.0]
        self.headingUKF.covarInit = [0.2, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.2, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.2, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.005, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.005]

        qNoiseIn = np.identity(5)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-6 * 1E-6
        qNoiseIn[3:5, 3:5] = qNoiseIn[3:5, 3:5] * 1E-6 * 1E-6
        self.headingUKF.qNoise = qNoiseIn.reshape(25).tolist()
        self.headingUKF.qObsVal = 0.001

    def SetAttTrackingErrorCam(self, SimBase):
        self.trackingErrorCam.attRefInMsg.subscribeTo(self.hillPoint.attRefOutMsg)
        self.trackingErrorCam.attNavInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingErrorCam.attGuidOutMsg, self.attGuidMsg)

        M2 =  rbk.euler2(90 * macros.D2R) #rbk.euler2(-90 * macros.D2R) #
        M3 =  rbk.euler1(90 * macros.D2R) #rbk.euler3(90 * macros.D2R) #
        M_cam = rbk.MRP2C(SimBase.DynModels.cameraMRP_CB)

        MRP = rbk.C2MRP(np.dot(np.dot(M3, M2), M_cam)) # This assures that the s/c does not control to the hill frame, but to a rotated frame such that the camera is pointing to the planet
        self.trackingErrorCam.sigma_R0R = MRP
        # self.trackingErrorCamData.sigma_R0R = [1./3+0.1, 1./3-0.1, 0.1-1/3]

    def SetMRPFeedbackRWA(self, SimBase):
        self.mrpFeedbackRWs.K = 3.5
        self.mrpFeedbackRWs.Ki = -1  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackRWs.P = 30.0
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetVehicleConfiguration(self):
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        # use the same inertia in the FSW algorithm as in the simulation
        vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetRWConfigMsg(self):
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * macros.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * macros.D2R
        wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              wheelJs,  # kg*m^2
                              0.2)  # Nm        uMax

        self.fswRwConfigMsg = fswSetupRW.writeConfigMessage()

    def SetRWMotorTorque(self, SimBase):
        controlAxes_B = [
                        1.0, 0.0, 0.0
                        , 0.0, 1.0, 0.0
                        , 0.0, 0.0, 1.0
                        ]
        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.mrpFeedbackRWs.cmdTorqueOutMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.rwMotorTorque.rwMotorTorqueOutMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    def SetCNNOpNav(self, SimBase):
        self.opNavCNN.imageInMsg.subscribeTo(SimBase.DynModels.cameraMod.imageOutMsg)
        self.opNavCNN.opnavCirclesOutMsg = self.opnavCirclesMsg
        self.opNavCNN.pixelNoise = [5,5,5]
        self.opNavCNN.pathToNetwork = bskPath + "/../../src/fswAlgorithms/imageProcessing/centerRadiusCNN/CAD.onnx"

    def SetImageProcessing(self, SimBase):
        self.imageProcessing.imageInMsg.subscribeTo(SimBase.DynModels.cameraMod.imageOutMsg)
        self.imageProcessing.opnavCirclesOutMsg = self.opnavCirclesMsg

        self.imageProcessing.saveImages = 0
        self.imageProcessing.expectedCircles = 1
        self.imageProcessing.cannyThresh = 200
        self.imageProcessing.voteThresh = 25
        self.imageProcessing.houghMinDist = 50
        self.imageProcessing.houghMinRadius = 20
        self.imageProcessing.blurrSize = 9
        self.imageProcessing.noiseSF = 1
        self.imageProcessing.dpValue = 1
        self.imageProcessing.saveDir = 'Test'
        self.imageProcessing.houghMaxRadius = 0  # int(512 / 1.25)

    def SetPixelLineConversion(self, SimBase):
        self.pixelLine.circlesInMsg.subscribeTo(self.opnavCirclesMsg)
        self.pixelLine.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.pixelLine.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.pixelLine.planetTarget = 2
        messaging.OpNavMsg_C_addAuthor(self.pixelLine.opNavOutMsg, self.opnavMsg)

    def SetLimbFinding(self, SimBase):
        self.limbFinding.imageInMsg.subscribeTo(SimBase.DynModels.cameraMod.imageOutMsg)

        self.limbFinding.saveImages = 0
        self.limbFinding.cannyThreshLow = 50
        self.limbFinding.cannyThreshHigh = 100
        self.limbFinding.blurrSize = 5
        self.limbFinding.limbNumThresh = 0

    def SetHorizonNav(self, SimBase):
        self.horizonNav.limbInMsg.subscribeTo(self.limbFinding.opnavLimbOutMsg)
        self.horizonNav.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.horizonNav.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.horizonNav.planetTarget = 2
        self.horizonNav.noiseSF = 1  # 2 should work though
        messaging.OpNavMsg_C_addAuthor(self.horizonNav.opNavOutMsg, self.opnavMsg)

    def SetRelativeODFilter(self):
        self.relativeOD.opNavInMsg.subscribeTo(self.opnavMsg)

        self.relativeOD.planetIdInit = 2
        self.relativeOD.alpha = 0.02
        self.relativeOD.beta = 2.0
        self.relativeOD.kappa = 0.0
        self.relativeOD.noiseSF = 7.5

        mu = 42828.314 * 1E9  # m^3/s^2
        elementsInit = orbitalMotion.ClassicElements()
        elementsInit.a = 10000 * 1E3  # m
        elementsInit.e = 0.2
        elementsInit.i = 10 * macros.D2R
        elementsInit.Omega = 25. * macros.D2R
        elementsInit.omega = 10. * macros.D2R
        elementsInit.f = 40 * macros.D2R
        r, v = orbitalMotion.elem2rv(mu, elementsInit)

        self.relativeOD.stateInit = r.tolist() + v.tolist()
        self.relativeOD.covarInit = [1. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 1. * 1E6, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 1. * 1E6, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.02 * 1E6, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.02 * 1E6, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02 * 1E6]

        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.relativeOD.qNoise = qNoiseIn.reshape(36).tolist()

    def SetFaultDetection(self, SimBase):
        self.opNavFault.navMeasPrimaryInMsg.subscribeTo(self.opnavPrimaryMsg)
        self.opNavFault.navMeasSecondaryInMsg.subscribeTo(self.opnavSecondaryMsg)
        self.opNavFault.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.opNavFault.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        messaging.OpNavMsg_C_addAuthor(self.opNavFault.opNavOutMsg, self.opnavMsg)
        self.opNavFault.sigmaFault = 0.3
        self.opNavFault.faultMode = 0

    def SetPixelLineFilter(self, SimBase):
        self.pixelLineFilter.circlesInMsg.subscribeTo(self.opnavCirclesMsg)
        self.pixelLineFilter.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.pixelLineFilter.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)

        self.pixelLineFilter.planetIdInit = 2
        self.pixelLineFilter.alpha = 0.02
        self.pixelLineFilter.beta = 2.0
        self.pixelLineFilter.kappa = 0.0
        self.pixelLineFilter.gamma = 0.9

        mu = 42828.314 * 1E9  # m^3/s^2
        elementsInit = orbitalMotion.ClassicElements()
        elementsInit.a = 10000 * 1E3  # m
        elementsInit.e = 0.2
        elementsInit.i = 10 * macros.D2R
        elementsInit.Omega = 25. * macros.D2R
        elementsInit.omega = 10. * macros.D2R
        elementsInit.f = 40 * macros.D2R
        r, v = orbitalMotion.elem2rv(mu, elementsInit)
        bias = [1, 1, 2]

        self.pixelLineFilter.stateInit = r.tolist() + v.tolist() + bias
        self.pixelLineFilter.covarInit = [10. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 10. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 10. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.01 * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.01 * 1E6, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01 * 1E6, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]

        qNoiseIn = np.identity(9)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        qNoiseIn[6:9, 6:9] = qNoiseIn[6:9, 6:9] * 1E-8 * 1E-8
        self.pixelLineFilter.qNoise = qNoiseIn.reshape(9 * 9).tolist()

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetHillPointGuidance(SimBase)
        self.SetVehicleConfiguration()
        self.SetRWConfigMsg()
        self.SetMRPFeedbackRWA(SimBase)
        self.SetRWMotorTorque(SimBase)
        self.SetAttTrackingErrorCam(SimBase)
        self.SetImageProcessing(SimBase)
        self.SetPixelLineConversion(SimBase)

        if centerRadiusCNNIncluded:
            self.SetCNNOpNav(SimBase)
        self.SetRelativeODFilter()
        self.SetFaultDetection(SimBase)

        # J. Christian methods
        self.SetLimbFinding(SimBase)
        self.SetHorizonNav(SimBase)

        self.SetOpNavPointGuidance(SimBase)
        self.SetHeadingUKF(SimBase)
        self.SetPixelLineFilter(SimBase)

    def setupGatewayMsgs(self):
        """create gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        # C wrapped gateway messages
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.opnavMsg = messaging.OpNavMsg_C()
        self.opnavPrimaryMsg = messaging.OpNavMsg_C()
        self.opnavSecondaryMsg = messaging.OpNavMsg_C()

        # C++ wrapped gateway messages
        self.opnavCirclesMsg = messaging.OpNavCirclesMsg()

        self.zeroGateWayMsgs()

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.opnavMsg.write(messaging.OpNavMsgPayload())
        self.opnavPrimaryMsg.write(messaging.OpNavMsgPayload())
        self.opnavSecondaryMsg.write(messaging.OpNavMsgPayload())

        self.opnavCirclesMsg.write(messaging.OpNavCirclesMsgPayload())
