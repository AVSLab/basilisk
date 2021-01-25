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

from Basilisk.fswAlgorithms import (hillPoint, inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque, opNavPoint, velocityPoint,
                                    cssWlsEst, headingSuKF, relativeODuKF, horizonOpNav,
                                    pixelLineConverter, faultDetection, pixelLineBiasUKF)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW, unitTestSupport, orbitalMotion, macros
from Basilisk.architecture import messaging2
import Basilisk.architecture.cMsgCInterfacePy as cMsgPy

from Basilisk import __path__
bskPath = __path__[0]

try:
    from Basilisk.fswAlgorithms import limbFinding, houghCircles  # FSW for OpNav
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
        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        self.opNavPointData = opNavPoint.OpNavPointConfig()
        self.opNavPointWrap = SimBase.setModelDataWrap(self.opNavPointData)
        self.opNavPointWrap.ModelTag = "opNavPoint"

        self.trackingErrorCamData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorCamWrap = SimBase.setModelDataWrap(self.trackingErrorCamData)
        self.trackingErrorCamWrap.ModelTag = "trackingErrorCam"

        self.mrpFeedbackRWsData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"
        
        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        self.imageProcessing = houghCircles.HoughCircles()
        self.imageProcessing.ModelTag = "houghCircles"

        if centerRadiusCNNIncluded:
            self.opNavCNN = centerRadiusCNN.CenterRadiusCNN()
            self.opNavCNN.ModelTag = "opNavCNN"

        self.pixelLineData = pixelLineConverter.PixelLineConvertData()
        self.pixelLineWrap = SimBase.setModelDataWrap(self.pixelLineData)
        self.pixelLineWrap.ModelTag = "pixelLine"

        self.opNavFaultData = faultDetection.FaultDetectionData()
        self.opNavFaultWrap = SimBase.setModelDataWrap(self.opNavFaultData)
        self.opNavFaultWrap.ModelTag = "OpNav_Fault"

        self.limbFinding = limbFinding.LimbFinding()
        self.limbFinding.ModelTag = "limbFind"

        self.horizonNavData = horizonOpNav.HorizonOpNavData()
        self.horizonNavWrap = SimBase.setModelDataWrap(self.horizonNavData)
        self.horizonNavWrap.ModelTag = "limbNav"

        self.relativeODData = relativeODuKF.RelODuKFConfig()
        self.relativeODWrap = SimBase.setModelDataWrap(self.relativeODData)
        self.relativeODWrap.ModelTag = "relativeOD"

        self.pixelLineFilterData = pixelLineBiasUKF.PixelLineBiasUKFConfig()
        self.pixelLineFilterWrap = SimBase.setModelDataWrap(self.pixelLineFilterData)
        self.pixelLineFilterWrap.ModelTag = "pixelLineFilter"

        self.headingUKFData = headingSuKF.HeadingSuKFConfig()
        self.headingUKFWrap = SimBase.setModelDataWrap(self.headingUKFData)
        self.headingUKFWrap.ModelTag = "headingUKF"

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

        SimBase.AddModelToTask("opNavPointTask", self.imageProcessing, None, 15)
        SimBase.AddModelToTask("opNavPointTask", self.pixelLineWrap, self.pixelLineData, 12)
        SimBase.AddModelToTask("opNavPointTask", self.opNavPointWrap, self.opNavPointData, 9)

        SimBase.AddModelToTask("headingPointTask", self.imageProcessing, None, 15)
        SimBase.AddModelToTask("headingPointTask", self.pixelLineWrap, self.pixelLineData, 12)
        SimBase.AddModelToTask("headingPointTask", self.headingUKFWrap, self.headingUKFData, 10)
        SimBase.AddModelToTask("headingPointTask", self.opNavPointWrap, self.opNavPointData, 9)

        SimBase.AddModelToTask("opNavPointLimbTask", self.limbFinding, None, 25)
        SimBase.AddModelToTask("opNavPointLimbTask", self.horizonNavWrap, self.horizonNavData, 12)
        SimBase.AddModelToTask("opNavPointLimbTask", self.opNavPointWrap, self.opNavPointData, 10)

        SimBase.AddModelToTask("opNavAttODLimbTask", self.limbFinding, None, 25)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.horizonNavWrap, self.horizonNavData, 12)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.opNavPointWrap, self.opNavPointData, 10)
        SimBase.AddModelToTask("opNavAttODLimbTask", self.relativeODWrap, self.relativeODData, 9)

        SimBase.AddModelToTask("opNavODTaskLimb", self.limbFinding, None, 25)
        SimBase.AddModelToTask("opNavODTaskLimb", self.horizonNavWrap, self.horizonNavData, 22)
        SimBase.AddModelToTask("opNavODTaskLimb", self.relativeODWrap, self.relativeODData, 20)

        SimBase.AddModelToTask("opNavPointTaskCheat", self.hillPointWrap, self.hillPointData, 10)
        SimBase.AddModelToTask("opNavPointTaskCheat", self.trackingErrorCamWrap, self.trackingErrorCamData, 9)

        SimBase.AddModelToTask("opNavODTask", self.imageProcessing, None, 15)
        SimBase.AddModelToTask("opNavODTask", self.pixelLineWrap, self.pixelLineData, 14)
        SimBase.AddModelToTask("opNavODTask", self.relativeODWrap, self.relativeODData, 13)

        SimBase.AddModelToTask("opNavODTaskB", self.imageProcessing, None, 15)
        SimBase.AddModelToTask("opNavODTaskB", self.pixelLineFilterWrap, self.pixelLineFilterData, 13)

        # SimBase.AddModelToTask("imageProcTask", self.imageProcessing, None, 15)

        SimBase.AddModelToTask("opNavAttODTask", self.imageProcessing, None, 15)
        SimBase.AddModelToTask("opNavAttODTask", self.pixelLineWrap, self.pixelLineData, 14)
        SimBase.AddModelToTask("opNavAttODTask", self.opNavPointWrap, self.opNavPointData, 10)
        SimBase.AddModelToTask("opNavAttODTask", self.relativeODWrap, self.relativeODData, 9)

        if centerRadiusCNNIncluded:
            SimBase.AddModelToTask("cnnAttODTask", self.opNavCNN, None, 15)
        SimBase.AddModelToTask("cnnAttODTask", self.pixelLineWrap, self.pixelLineData, 14)
        SimBase.AddModelToTask("cnnAttODTask", self.opNavPointWrap, self.opNavPointData, 10)
        SimBase.AddModelToTask("cnnAttODTask", self.relativeODWrap, self.relativeODData, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)

        SimBase.AddModelToTask("attODFaultDet", self.limbFinding, None, 25)
        SimBase.AddModelToTask("attODFaultDet", self.horizonNavWrap, self.horizonNavData, 20)
        SimBase.AddModelToTask("attODFaultDet", self.imageProcessing, None, 18)
        SimBase.AddModelToTask("attODFaultDet", self.pixelLineWrap, self.pixelLineData, 16)
        SimBase.AddModelToTask("attODFaultDet", self.opNavFaultWrap, self.opNavFaultData, 14)
        SimBase.AddModelToTask("attODFaultDet", self.opNavPointWrap, self.opNavPointData, 10)
        SimBase.AddModelToTask("attODFaultDet", self.relativeODWrap, self.relativeODData, 9)

        SimBase.AddModelToTask("opNavFaultDet", self.limbFinding, None, 25)
        SimBase.AddModelToTask("opNavFaultDet", self.horizonNavWrap, self.horizonNavData, 20)
        SimBase.AddModelToTask("opNavFaultDet", self.imageProcessing, None, 18)
        SimBase.AddModelToTask("opNavFaultDet", self.pixelLineWrap, self.pixelLineData, 16)
        SimBase.AddModelToTask("opNavFaultDet", self.opNavFaultWrap, self.opNavFaultData, 14)
        SimBase.AddModelToTask("opNavFaultDet", self.relativeODWrap, self.relativeODData, 9)

        if centerRadiusCNNIncluded:
            SimBase.AddModelToTask("cnnFaultDet", self.opNavCNN, None, 25)
            SimBase.AddModelToTask("cnnFaultDet", self.pixelLineWrap, self.pixelLineData, 20)
            SimBase.AddModelToTask("cnnFaultDet", self.imageProcessing, None, 18)
            SimBase.AddModelToTask("cnnFaultDet", self.pixelLineWrap, self.pixelLineData, 16)
            SimBase.AddModelToTask("cnnFaultDet", self.opNavFaultWrap, self.opNavFaultData, 14)
            SimBase.AddModelToTask("cnnFaultDet", self.opNavPointWrap, self.opNavPointData, 10)
            SimBase.AddModelToTask("cnnFaultDet", self.relativeODWrap, self.relativeODData, 9)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()"
                                ])

        SimBase.createNewEvent("prepOpNav", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'prepOpNav'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("imageGen", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'imageGen'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('imageProcTask')",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("pointOpNav", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'pointOpNav'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("pointHead", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'pointHead'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('headingPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("pointLimb", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'pointLimb'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointLimbTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("OpNavOD", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'OpNavOD'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('opNavODTask')"])

        SimBase.createNewEvent("OpNavODLimb", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'OpNavODLimb'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('opNavODTaskLimb')"])

        SimBase.createNewEvent("OpNavODB", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'OpNavODB'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('opNavODTaskB')"])

        SimBase.createNewEvent("OpNavAttOD", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'OpNavAttOD'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavAttODTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("OpNavAttODLimb", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'OpNavAttODLimb'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavAttODLimbTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("CNNAttOD", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'CNNAttOD'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('cnnAttODTask')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("FaultDet", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'FaultDet'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('attODFaultDet')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

        SimBase.createNewEvent("ODFaultDet", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'ODFaultDet'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('opNavPointTaskCheat')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('opNavFaultDet')"])

        SimBase.createNewEvent("FaultDetCNN", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'FaultDetCNN'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('cnnFaultDet')",
                                "self.enableTask('mrpFeedbackRWsTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetHillPointGuidance(self, SimBase):
        self.hillPointData.transNavInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.transOutMsg)
        self.hillPointData.celBodyInMsg.subscribeTo(SimBase.DynModels.ephemObject.ephemOutMsgs[0])

    def SetOpNavPointGuidance(self, SimBase):
        cMsgPy.AttGuidMsg_C_addAuthor(self.opNavPointData.attGuidanceOutMsg, self.attGuidMsg)
        self.opNavPointData.imuInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.opNavPointData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.opNavPointData.opnavDataInMsg.subscribeTo(self.opnavMsg)
        self.opNavPointData.smallAngle = 0.001*np.pi/180.
        self.opNavPointData.timeOut = 1000  # Max time in sec between images before engaging search
        # self.opNavPointData.opNavAxisSpinRate = 0.1*np.pi/180.
        self.opNavPointData.omega_RN_B = [0.001, 0.0, -0.001]
        self.opNavPointData.alignAxis_C = [0., 0., 1]

    def SetHeadingUKF(self, SimBase):
        self.headingUKFData.opnavDataInMsg.subscribeTo(self.opnavMsg)
        self.headingUKFData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)

        self.headingUKFData.alpha = 0.02
        self.headingUKFData.beta = 2.0
        self.headingUKFData.kappa = 0.0

        self.headingUKFData.state = [0.0, 0., 0., 0., 0.]
        self.headingUKFData.stateInit = [0.0, 0.0, 1.0, 0.0, 0.0]
        self.headingUKFData.covarInit = [0.2, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.2, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.2, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.005, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.005]

        qNoiseIn = np.identity(5)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-6 * 1E-6
        qNoiseIn[3:5, 3:5] = qNoiseIn[3:5, 3:5] * 1E-6 * 1E-6
        self.headingUKFData.qNoise = qNoiseIn.reshape(25).tolist()
        self.headingUKFData.qObsVal = 0.001

    ## Celestial point to Mars
    def SetCelTwoBodyMarsPoint(self):
        self.celTwoBodyMarsData.inputNavDataName = "simple_trans_nav_output"
        self.celTwoBodyMarsData.inputCelMessName = "mars barycenter_ephemeris_data"
        self.celTwoBodyMarsData.outputDataName = "att_ref_output"
        self.celTwoBodyMarsData.singularityThresh = 1.0 * math.pi / 180.0

    def SetAttTrackingErrorCam(self, SimBase):
        self.trackingErrorCamData.attRefInMsg.subscribeTo(self.hillPointData.attRefOutMsg)
        self.trackingErrorCamData.attNavInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        cMsgPy.AttGuidMsg_C_addAuthor(self.trackingErrorCamData.attGuidOutMsg, self.attGuidMsg)

        M2 =  rbk.euler2(90 * macros.D2R) #rbk.euler2(-90 * macros.D2R) #
        M3 =  rbk.euler1(90 * macros.D2R) #rbk.euler3(90 * macros.D2R) #
        M_cam = rbk.MRP2C(SimBase.DynModels.cameraMRP_CB)

        MRP = rbk.C2MRP(np.dot(np.dot(M3, M2), M_cam)) # This assures that the s/c does not control to the hill frame, but to a rotated frame such that the camera is pointing to the planet
        self.trackingErrorCamData.sigma_R0R = MRP
        # self.trackingErrorCamData.sigma_R0R = [1./3+0.1, 1./3-0.1, 0.1-1/3]

    def SetMRPFeedbackRWA(self, SimBase):
        self.mrpFeedbackRWsData.K = 3.5
        self.mrpFeedbackRWsData.Ki = -1  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 30.0
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1

        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWsData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWsData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetVehicleConfiguration(self):
        vehicleConfigOut = messaging2.VehicleConfigMsgPayload()
        # use the same inertia in the FSW algorithm as in the simulation
        vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vcMsg = messaging2.VehicleConfigMsg().write(vehicleConfigOut)

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
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.mrpFeedbackRWsData.cmdTorqueOutMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.rwMotorTorqueData.rwMotorTorqueOutMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

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
        self.pixelLineData.circlesInMsg.subscribeTo(self.opnavCirclesMsg)
        self.pixelLineData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.pixelLineData.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.pixelLineData.planetTarget = 2
        cMsgPy.OpNavMsg_C_addAuthor(self.pixelLineData.opNavOutMsg, self.opnavMsg)

    def SetLimbFinding(self, SimBase):
        self.limbFinding.imageInMsg.subscribeTo(SimBase.DynModels.cameraMod.imageOutMsg)

        self.limbFinding.saveImages = 0
        self.limbFinding.cannyThreshLow = 50
        self.limbFinding.cannyThreshHigh = 100
        self.limbFinding.blurrSize = 5
        self.limbFinding.limbNumThresh = 0

    def SetHorizonNav(self, SimBase):
        self.horizonNavData.limbInMsg.subscribeTo(self.limbFinding.opnavLimbOutMsg)
        self.horizonNavData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.horizonNavData.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        self.horizonNavData.planetTarget = 2
        self.horizonNavData.noiseSF = 1  # 2 should work though
        cMsgPy.OpNavMsg_C_addAuthor(self.horizonNavData.opNavOutMsg, self.opnavMsg)

    def SetRelativeODFilter(self):
        self.relativeODData.opNavInMsg.subscribeTo(self.opnavMsg)

        self.relativeODData.planetIdInit = 2
        self.relativeODData.alpha = 0.02
        self.relativeODData.beta = 2.0
        self.relativeODData.kappa = 0.0
        self.relativeODData.noiseSF = 7.5

        mu = 42828.314 * 1E9  # m^3/s^2
        elementsInit = orbitalMotion.ClassicElements()
        elementsInit.a = 10000 * 1E3  # m
        elementsInit.e = 0.2
        elementsInit.i = 10 * macros.D2R
        elementsInit.Omega = 25. * macros.D2R
        elementsInit.omega = 10. * macros.D2R
        elementsInit.f = 40 * macros.D2R
        r, v = orbitalMotion.elem2rv(mu, elementsInit)

        self.relativeODData.stateInit = r.tolist() + v.tolist()
        self.relativeODData.covarInit = [1. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 1. * 1E6, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 1. * 1E6, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.02 * 1E6, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.02 * 1E6, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02 * 1E6]

        qNoiseIn = np.identity(6)
        qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 1E-3 * 1E-3
        qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 1E-4 * 1E-4
        self.relativeODData.qNoise = qNoiseIn.reshape(36).tolist()

    def SetFaultDetection(self, SimBase):
        self.opNavFaultData.navMeasPrimaryInMsg.subscribeTo(self.opnavPrimaryMsg)
        self.opNavFaultData.navMeasSecondaryInMsg.subscribeTo(self.opnavSecondaryMsg)
        self.opNavFaultData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.opNavFaultData.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)
        cMsgPy.OpNavMsg_C_addAuthor(self.opNavFaultData.opNavOutMsg, self.opnavMsg)
        self.opNavFaultData.sigmaFault = 0.3
        self.opNavFaultData.faultMode = 0

    def SetPixelLineFilter(self, SimBase):
        self.pixelLineFilterData.circlesInMsg.subscribeTo(self.opnavCirclesMsg)
        self.pixelLineFilterData.cameraConfigInMsg.subscribeTo(SimBase.DynModels.cameraMod.cameraConfigOutMsg)
        self.pixelLineFilterData.attInMsg.subscribeTo(SimBase.DynModels.SimpleNavObject.attOutMsg)

        self.pixelLineFilterData.planetIdInit = 2
        self.pixelLineFilterData.alpha = 0.02
        self.pixelLineFilterData.beta = 2.0
        self.pixelLineFilterData.kappa = 0.0
        self.pixelLineFilterData.gamma = 0.9

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

        self.pixelLineFilterData.stateInit = r.tolist() + v.tolist() + bias
        self.pixelLineFilterData.covarInit = [10. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
        self.pixelLineFilterData.qNoise = qNoiseIn.reshape(9 * 9).tolist()

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
        self.attGuidMsg = cMsgPy.AttGuidMsg_C()
        self.opnavMsg = cMsgPy.OpNavMsg_C()
        self.opnavPrimaryMsg = cMsgPy.OpNavMsg_C()
        self.opnavSecondaryMsg = cMsgPy.OpNavMsg_C()

        # C++ wrapped gateway messages
        self.opnavCirclesMsg = messaging2.CirclesOpNavMsg()

        self.zeroGateWayMsgs()

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.attGuidMsg.write(messaging2.AttGuidMsgPayload())
        self.opnavMsg.write(messaging2.OpNavMsgPayload())
        self.opnavPrimaryMsg.write(messaging2.OpNavMsgPayload())
        self.opnavSecondaryMsg.write(messaging2.OpNavMsgPayload())

        self.opnavCirclesMsg.write(messaging2.CirclesOpNavMsgPayload())

# BSKFswModels()
