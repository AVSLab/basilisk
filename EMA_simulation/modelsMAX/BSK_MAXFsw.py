#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (attTrackingError, mrpFeedback, rwMotorTorque, threeAxesPoint,
                                    platformRotation, solarArrayRotation, PIDController1D, torqueScheduler)
from Basilisk.architecture import messaging

import numpy as np


class BSKFswModels:
    """Defines the FSW class"""

    def __init__(self, SimBase, fswRate, spacecraftIndex):
        # define empty class variables
        self.spacecraftIndex = spacecraftIndex
        self.decayTime = None
        self.xi = None
        self.modeRequest = "earthPointing"
        self.stationKeeping = "OFF"

        self.fswRwConfigMsg = None
        self.fswVehConfigMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create tasks
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("platformRotationTask" + str(spacecraftIndex), self.processTasksTimeStep), 25)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("earthPointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("sepPointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("cruisePointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("solarArrayRotationTask" + str(spacecraftIndex), self.processTasksTimeStep), 15)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex), self.processTasksTimeStep), 10)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.processTasksTimeStep), 5)

        # Create module data and module wraps
        self.earthPointData = threeAxesPoint.threeAxesPointConfig()
        self.earthPointWrap = SimBase.setModelDataWrap(self.earthPointData)
        self.earthPointWrap.ModelTag = "earthPoint"

        self.sepPointData = threeAxesPoint.threeAxesPointConfig()
        self.sepPointWrap = SimBase.setModelDataWrap(self.sepPointData)
        self.sepPointWrap.ModelTag = "sepPoint"

        self.cruisePointData = threeAxesPoint.threeAxesPointConfig()
        self.cruisePointWrap = SimBase.setModelDataWrap(self.cruisePointData)
        self.cruisePointWrap.ModelTag = "cruisePoint"

        self.platformRotationData = platformRotation.platformRotationConfig()
        self.platformRotationWrap = SimBase.setModelDataWrap(self.platformRotationData)
        self.platformRotationWrap.ModelTag = "platformRotation"

        self.solarArrayRotationDataList = []
        self.solarArrayRotationWrapList = []
        self.solarArrayControllerDataList = []
        self.solarArrayControllerWrapList = []
        self.platformControllerDataList = []
        self.platformControllerWrapList = []
        for item in range(SimBase.DynModels[spacecraftIndex].numRSA):
            self.solarArrayRotationDataList.append(solarArrayRotation.solarArrayRotationConfig())
            self.solarArrayRotationWrapList.append(SimBase.setModelDataWrap(self.solarArrayRotationDataList[item]))
            self.solarArrayRotationWrapList[item].ModelTag = "solarArrayRotation"

            self.solarArrayControllerDataList.append(PIDController1D.PIDController1DConfig())
            self.solarArrayControllerWrapList.append(SimBase.setModelDataWrap(self.solarArrayControllerDataList[item]))
            self.solarArrayControllerWrapList[item].ModelTag = "solarArrayController"

        for item in range(2):
            self.platformControllerDataList.append(PIDController1D.PIDController1DConfig())
            self.platformControllerWrapList.append(SimBase.setModelDataWrap(self.platformControllerDataList[item]))
            self.platformControllerWrapList[item].ModelTag = "platformController"

        self.torqueSchedulerEarthData = torqueScheduler.torqueSchedulerConfig()
        self.torqueSchedulerEarthWrap = SimBase.setModelDataWrap(self.torqueSchedulerEarthData)
        self.torqueSchedulerEarthWrap.ModelTag = "torqueSchedulerEarth"

        self.torqueSchedulerSepData = torqueScheduler.torqueSchedulerConfig()
        self.torqueSchedulerSepWrap = SimBase.setModelDataWrap(self.torqueSchedulerSepData)
        self.torqueSchedulerSepWrap.ModelTag = "torqueSchedulerSEP"

        self.torqueSchedulerCruiseData = torqueScheduler.torqueSchedulerConfig()
        self.torqueSchedulerCruiseWrap = SimBase.setModelDataWrap(self.torqueSchedulerCruiseData)
        self.torqueSchedulerCruiseWrap.ModelTag = "torqueSchedulerCruise"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackRWsData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("platformRotationTask" + str(spacecraftIndex), self.platformRotationWrap, self.platformRotationData, 10)
        for item in range(len(self.platformControllerDataList)):
            SimBase.AddModelToTask("platformRotationTask" + str(spacecraftIndex), self.platformControllerWrapList[item], self.platformControllerDataList[item], 10)

        SimBase.AddModelToTask("earthPointTask" + str(spacecraftIndex), self.earthPointWrap, self.earthPointData, 10)
        SimBase.AddModelToTask("earthPointTask" + str(spacecraftIndex), self.torqueSchedulerEarthWrap, self.torqueSchedulerEarthData, 10)

        SimBase.AddModelToTask("sepPointTask" + str(spacecraftIndex), self.sepPointWrap, self.sepPointData, 10)
        SimBase.AddModelToTask("sepPointTask" + str(spacecraftIndex), self.torqueSchedulerSepWrap, self.torqueSchedulerSepData, 10)

        SimBase.AddModelToTask("cruisePointTask" + str(spacecraftIndex), self.cruisePointWrap, self.cruisePointData, 10)
        SimBase.AddModelToTask("cruisePointTask" + str(spacecraftIndex), self.torqueSchedulerCruiseWrap, self.torqueSchedulerCruiseData, 10)

        for item in range(SimBase.DynModels[spacecraftIndex].numRSA):
            SimBase.AddModelToTask("solarArrayRotationTask" + str(spacecraftIndex), self.solarArrayRotationWrapList[item], self.solarArrayRotationDataList[item], 7)
            SimBase.AddModelToTask("solarArrayRotationTask" + str(spacecraftIndex), self.solarArrayControllerWrapList[item], self.solarArrayControllerDataList[item], 6)

        SimBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingErrorWrap, self.trackingErrorData, 4)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 3)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorqueWrap, self.rwMotorTorqueData, 2)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc[spacecraftIndex].disableAllTasks()

        SimBase.createNewEvent("initiateEarthPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'earthPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('platformRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('earthPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateEarthPointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateSEPPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'sepPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('platformRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('sepPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateSEPPointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateCruisePointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'cruisePointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('platformRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('cruisePointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayRotationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateCruisePointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetEarthPointGuidance(self, SimBase):
        """
        Define MAX Earth point guidance module
        """
        self.earthPointData.a1_B = [1, 0, 0]        # solar array axis drive
        self.earthPointData.a2_B = [0, 1, 0]        # antiparallel direction to the sensitive surface
        self.earthPointData.h1_B = [0, 1, 0]        # high gain antenna 
        self.earthPointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.earthPointData.transNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.earthPointData.ephemerisInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.earth])
        messaging.AttRefMsg_C_addAuthor(self.earthPointData.attRefOutMsg, self.attRefMsg)

    def SetSEPPointGuidance(self, SimBase):
        """
        Defines MAX SEP point guidance module
        """
        self.sepPointData.a1_B = [1, 0, 0]          # solar array axis drive
        self.sepPointData.a2_B = [0, 1, 0]          # antiparallel direction to the sensitive surface
        self.sepPointData.h_N = [1, 0, 0]           # random inertial thrust direction
        self.sepPointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.sepPointData.bodyHeadingInMsg.subscribeTo(self.platformRotationData.bodyHeadingOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.sepPointData.attRefOutMsg, self.attRefMsg)

    def SetCruisePointGuidance(self, SimBase):
        """
        Defines MAX Cruise point guidance module
        """
        self.cruisePointData.a1_B = [1, 0, 0]       # solar array axis drive
        self.cruisePointData.a2_B = [0, 1, 0]       # antiparallel direction to the sensitive surface
        self.cruisePointData.h1_B = [0, 1, 0]       # low gain antenna #1
        self.cruisePointData.h2_B = [0, -1, 0]      # low gain antenna #2
        self.cruisePointData.priorityFlag = 1
        self.cruisePointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.cruisePointData.transNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.cruisePointData.ephemerisInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.earth])
        messaging.AttRefMsg_C_addAuthor(self.cruisePointData.attRefOutMsg, self.attRefMsg)

    def SetSolarArrayRotation(self, SimBase):
        """
        Defines the solar array rotation request
        """
        # Define the rotation for the first solar array
        self.solarArrayRotationDataList[0].a1_B = [1, 0, 0]
        self.solarArrayRotationDataList[0].a2_B = [0, 1, 0]
        self.solarArrayRotationDataList[0].attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solarArrayRotationDataList[0].attRefInMsg.subscribeTo(self.attRefMsg)
        self.solarArrayRotationDataList[0].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[0].spinningBodyOutMsg)

        # Define the rotation for the second solar array
        self.solarArrayRotationDataList[1].a1_B = [-1, 0, 0]
        self.solarArrayRotationDataList[1].a2_B = [0, 1, 0]
        self.solarArrayRotationDataList[1].attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solarArrayRotationDataList[1].attRefInMsg.subscribeTo(self.attRefMsg)
        self.solarArrayRotationDataList[1].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[1].spinningBodyOutMsg)

    def SetSolarArrayController(self, SimBase):
        """
        Defines the control torques to satisfy the requested solar array rotation
        """
        # Define the controller for the first solar array
        self.solarArrayControllerDataList[0].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[0].spinningBodyOutMsg)
        self.solarArrayControllerDataList[0].spinningBodyRefInMsg.subscribeTo(self.solarArrayRotationDataList[0].spinningBodyRefOutMsg)
        self.solarArrayControllerDataList[0].K = 0.08
        self.solarArrayControllerDataList[0].P = 3

        # Define the controller for the second solar array
        self.solarArrayControllerDataList[1].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[1].spinningBodyOutMsg)
        self.solarArrayControllerDataList[1].spinningBodyRefInMsg.subscribeTo(self.solarArrayRotationDataList[1].spinningBodyRefOutMsg)
        self.solarArrayControllerDataList[1].K = 0.08
        self.solarArrayControllerDataList[1].P = 3

    def SetPlatformRotation(self, SimBase):
        """
        Defines the thruster platform rotation logic
        """
        self.platformRotationData.sigma_MB = [0, 0, 0]
        self.platformRotationData.r_BM_M = [0, 0, 1.6]
        self.platformRotationData.r_FM_F = [0, 0, 0]
        self.platformRotationData.r_TF_F = [0.00, 0.00, 0.00]
        self.platformRotationData.T_F = [0.0, 0.0, 10.0]
        self.platformRotationData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)

    def SetPlatformController(self, SimBase):
        """
        Defines the control torques to satisfy the requested platform rotation
        """
        # Define the controller for the first solar array
        self.platformControllerDataList[0].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform.spinningBodyOutMsgs[0])
        self.platformControllerDataList[0].spinningBodyRefInMsg.subscribeTo(self.platformRotationData.SpinningBodyRef1OutMsg)
        self.platformControllerDataList[0].K = 1
        self.platformControllerDataList[0].P = 5

        # Define the controller for the second solar array
        self.platformControllerDataList[1].spinningBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform.spinningBodyOutMsgs[1])
        self.platformControllerDataList[1].spinningBodyRefInMsg.subscribeTo(self.platformRotationData.SpinningBodyRef2OutMsg)
        self.platformControllerDataList[1].K = 1
        self.platformControllerDataList[1].P = 5

    def SetTorqueSchedulerEarth(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueSchedulerEarthData.lockFlag = 3
        self.torqueSchedulerEarthData.tSwitch = 60
        self.torqueSchedulerEarthData.motorTorque1InMsg.subscribeTo(self.platformControllerDataList[0].motorTorqueOutMsg)
        self.torqueSchedulerEarthData.motorTorque2InMsg.subscribeTo(self.platformControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueSchedulerEarthData.motorTorqueOutMsg, self.platformTorqueMsg)

    def SetTorqueSchedulerSEP(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueSchedulerSepData.lockFlag = 1
        self.torqueSchedulerSepData.tSwitch = 60
        self.torqueSchedulerSepData.motorTorque1InMsg.subscribeTo(self.platformControllerDataList[0].motorTorqueOutMsg)
        self.torqueSchedulerSepData.motorTorque2InMsg.subscribeTo(self.platformControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueSchedulerSepData.motorTorqueOutMsg, self.platformTorqueMsg)

    def SetTorqueSchedulerCruise(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueSchedulerCruiseData.lockFlag = 3
        self.torqueSchedulerCruiseData.tSwitch = 60
        self.torqueSchedulerCruiseData.motorTorque1InMsg.subscribeTo(self.platformControllerDataList[0].motorTorqueOutMsg)
        self.torqueSchedulerCruiseData.motorTorque2InMsg.subscribeTo(self.platformControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueSchedulerCruiseData.motorTorqueOutMsg, self.platformTorqueMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """
        Defines the module that converts a reference message into a guidance message
        """
        self.trackingErrorData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.trackingErrorData.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingErrorData.attGuidOutMsg, self.attGuidMsg)

    def SetMRPFeedbackRWA(self, SimBase):
        """
        Defines the control properties
        """
        self.decayTime = 50
        self.xi = 0.9
        self.mrpFeedbackRWsData.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWsData.K = (self.mrpFeedbackRWsData.P / self.xi) * (self.mrpFeedbackRWsData.P / self.xi) / np.max(SimBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1

        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.mrpFeedbackRWsData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWsData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRWConfigMsg(self, SimBase):
        """
        Imports the RWs configuration information
        """
        # Configure RW pyramid
        self.fswRwConfigMsg = SimBase.DynModels[self.spacecraftIndex].rwFactory.getConfigMessage()

    def SetRWMotorTorque(self):
        """
        Defines the motor torque from the control law
        """
        controlAxes_B = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.mrpFeedbackRWsData.cmdTorqueOutMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """
        Initializes all FSW objects
        """
        self.SetRWConfigMsg(SimBase)
        self.SetEarthPointGuidance(SimBase)
        self.SetSEPPointGuidance(SimBase)
        self.SetCruisePointGuidance(SimBase)
        self.SetSolarArrayRotation(SimBase)
        self.SetSolarArrayController(SimBase)
        self.SetPlatformRotation(SimBase)
        self.SetPlatformController(SimBase)
        self.SetTorqueSchedulerEarth()
        self.SetTorqueSchedulerSEP()
        self.SetTorqueSchedulerCruise()
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetRWMotorTorque()

    def setupGatewayMsgs(self, SimBase):
        """
        Creates C-wrapped gateway messages such that different modules can write to and provide a common input msg for
        downstream modules
        """
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.platformTorqueMsg = messaging.ArrayMotorTorqueMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.rwMotorTorqueData.rwMotorTorqueOutMsg)
        SimBase.DynModels[self.spacecraftIndex].platform.motorTorqueInMsg.subscribeTo(self.platformTorqueMsg)
        for item in range(SimBase.DynModels[self.spacecraftIndex].numRSA):
            SimBase.DynModels[self.spacecraftIndex].RSAList[item].motorTorqueInMsg.subscribeTo(self.solarArrayControllerDataList[item].motorTorqueOutMsg)

    def zeroGateWayMsgs(self):
        """
        Zeroes all the FSW gateway message payloads
        """
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.platformTorqueMsg.write(messaging.ArrayMotorTorqueMsgPayload())
