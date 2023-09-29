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

from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (attTrackingError, mrpFeedback, rwMotorTorque, oneAxisSolarArrayPoint,
                                    thrusterPlatformReference, thrusterPlatformState, solarArrayReference, 
                                    hingedRigidBodyPIDMotor, torqueScheduler, thrustCMEstimation)
from Basilisk.architecture import messaging

import numpy as np


class BSKFswModels:
    """Defines the FSW class"""

    def __init__(self, SimBase, fswRate, platRefRate, spacecraftIndex):
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
        self.platformTorqueMsg = None
        self.platformLockMsg = None
        self.thrusterFlag = 1
        self.cmEstimation = True

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)
        self.processPlatformTimeStep = mc.sec2nano(platRefRate)

        # Create tasks
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("cmEstimationTask" + str(spacecraftIndex), self.processTasksTimeStep), 30)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("thrusterPlatformReferenceTask" + str(spacecraftIndex), self.processPlatformTimeStep), 25)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("thrusterPlatformTask" + str(spacecraftIndex), self.processTasksTimeStep), 25)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("earthPointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("sepPointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("cruisePointTask" + str(spacecraftIndex), self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("solarArrayReferenceTask" + str(spacecraftIndex), self.processTasksTimeStep), 15)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex), self.processTasksTimeStep), 10)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.processTasksTimeStep), 5)

        # Create module data and module wraps
        self.earthPointData = oneAxisSolarArrayPoint.OneAxisSolarArrayPointConfig()
        self.earthPointWrap = SimBase.setModelDataWrap(self.earthPointData)
        self.earthPointWrap.ModelTag = "earthPoint"

        self.sepPointData = oneAxisSolarArrayPoint.OneAxisSolarArrayPointConfig()
        self.sepPointWrap = SimBase.setModelDataWrap(self.sepPointData)
        self.sepPointWrap.ModelTag = "sepPoint"

        self.cruisePointData = oneAxisSolarArrayPoint.OneAxisSolarArrayPointConfig()
        self.cruisePointWrap = SimBase.setModelDataWrap(self.cruisePointData)
        self.cruisePointWrap.ModelTag = "cruisePoint"

        self.platform1ReferenceData = thrusterPlatformReference.ThrusterPlatformReferenceConfig()
        self.platform1ReferenceWrap = SimBase.setModelDataWrap(self.platform1ReferenceData)
        self.platform1ReferenceWrap.ModelTag = "thrusterPlatform1Reference"

        self.platform2ReferenceData = thrusterPlatformReference.ThrusterPlatformReferenceConfig()
        self.platform2ReferenceWrap = SimBase.setModelDataWrap(self.platform2ReferenceData)
        self.platform2ReferenceWrap.ModelTag = "thrusterPlatform2Reference"

        self.platform1StateData = thrusterPlatformState.ThrusterPlatformStateConfig()
        self.platform1StateWrap = SimBase.setModelDataWrap(self.platform1StateData)
        self.platform1StateWrap.ModelTag = "thrusterPlatform1State"

        self.platform2StateData = thrusterPlatformState.ThrusterPlatformStateConfig()
        self.platform2StateWrap = SimBase.setModelDataWrap(self.platform2StateData)
        self.platform2StateWrap.ModelTag = "thrusterPlatform2State"

        self.solarArrayReferenceDataList = []
        self.solarArrayReferenceWrapList = []
        self.solarArrayControllerDataList = []
        self.solarArrayControllerWrapList = []
        self.platform1ControllerDataList = []
        self.platform1ControllerWrapList = []
        self.platform2ControllerDataList = []
        self.platform2ControllerWrapList = []
        for item in range(SimBase.DynModels[spacecraftIndex].numRSA):
            self.solarArrayReferenceDataList.append(solarArrayReference.solarArrayReferenceConfig())
            self.solarArrayReferenceWrapList.append(SimBase.setModelDataWrap(self.solarArrayReferenceDataList[item]))
            self.solarArrayReferenceWrapList[item].ModelTag = "solarArrayReference"

            self.solarArrayControllerDataList.append(hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotorConfig())
            self.solarArrayControllerWrapList.append(SimBase.setModelDataWrap(self.solarArrayControllerDataList[item]))
            self.solarArrayControllerWrapList[item].ModelTag = "solarArrayReference"

        for item in range(2):
            self.platform1ControllerDataList.append(hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotorConfig())
            self.platform1ControllerWrapList.append(SimBase.setModelDataWrap(self.platform1ControllerDataList[item]))
            self.platform1ControllerWrapList[item].ModelTag = "platform1Controller"

            self.platform2ControllerDataList.append(hingedRigidBodyPIDMotor.hingedRigidBodyPIDMotorConfig())
            self.platform2ControllerWrapList.append(SimBase.setModelDataWrap(self.platform2ControllerDataList[item]))
            self.platform2ControllerWrapList[item].ModelTag = "platform2Controller"

        self.torqueScheduler1EarthData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler1EarthWrap = SimBase.setModelDataWrap(self.torqueScheduler1EarthData)
        self.torqueScheduler1EarthWrap.ModelTag = "torqueScheduler1Earth"

        self.torqueScheduler2EarthData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler2EarthWrap = SimBase.setModelDataWrap(self.torqueScheduler2EarthData)
        self.torqueScheduler2EarthWrap.ModelTag = "torqueScheduler2Earth"

        self.torqueScheduler1SepData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler1SepWrap = SimBase.setModelDataWrap(self.torqueScheduler1SepData)
        self.torqueScheduler1SepWrap.ModelTag = "torqueScheduler1SEP"

        self.torqueScheduler2SepData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler2SepWrap = SimBase.setModelDataWrap(self.torqueScheduler2SepData)
        self.torqueScheduler2SepWrap.ModelTag = "torqueScheduler2SEP"

        self.torqueScheduler1CruiseData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler1CruiseWrap = SimBase.setModelDataWrap(self.torqueScheduler1CruiseData)
        self.torqueScheduler1CruiseWrap.ModelTag = "torqueScheduler1Cruise"

        self.torqueScheduler2CruiseData = torqueScheduler.torqueSchedulerConfig()
        self.torqueScheduler2CruiseWrap = SimBase.setModelDataWrap(self.torqueScheduler2CruiseData)
        self.torqueScheduler2CruiseWrap.ModelTag = "torqueScheduler2Cruise"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackRWsData = mrpFeedback.mrpFeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"

        self.cmEstimationData = thrustCMEstimation.ThrustCMEstimation()
        self.cmEstimationData.ModelTag = "cmEstimation"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("cmEstimationTask" + str(spacecraftIndex), self.cmEstimationData, None, 10)

        SimBase.AddModelToTask("thrusterPlatformReferenceTask" + str(spacecraftIndex), self.platform1ReferenceWrap, self.platform1ReferenceData, 10)
        SimBase.AddModelToTask("thrusterPlatformReferenceTask" + str(spacecraftIndex), self.platform2ReferenceWrap, self.platform2ReferenceData, 10)

        for item in range(len(self.platform1ControllerDataList)):
            SimBase.AddModelToTask("thrusterPlatformTask" + str(spacecraftIndex), self.platform1ControllerWrapList[item], self.platform1ControllerDataList[item], 10)
            SimBase.AddModelToTask("thrusterPlatformTask" + str(spacecraftIndex), self.platform2ControllerWrapList[item], self.platform2ControllerDataList[item], 10)
        SimBase.AddModelToTask("thrusterPlatformTask" + str(spacecraftIndex), self.platform1StateWrap, self.platform1StateData, 9)
        SimBase.AddModelToTask("thrusterPlatformTask" + str(spacecraftIndex), self.platform2StateWrap, self.platform2StateData, 9)

        SimBase.AddModelToTask("earthPointTask" + str(spacecraftIndex), self.earthPointWrap, self.earthPointData, 10)
        SimBase.AddModelToTask("earthPointTask" + str(spacecraftIndex), self.torqueScheduler1EarthWrap, self.torqueScheduler1EarthData, 10)
        SimBase.AddModelToTask("earthPointTask" + str(spacecraftIndex), self.torqueScheduler2EarthWrap, self.torqueScheduler2EarthData, 10)

        SimBase.AddModelToTask("sepPointTask" + str(spacecraftIndex), self.sepPointWrap, self.sepPointData, 10)
        SimBase.AddModelToTask("sepPointTask" + str(spacecraftIndex), self.torqueScheduler1SepWrap, self.torqueScheduler1SepData, 10)
        SimBase.AddModelToTask("sepPointTask" + str(spacecraftIndex), self.torqueScheduler2SepWrap, self.torqueScheduler2SepData, 10)

        SimBase.AddModelToTask("cruisePointTask" + str(spacecraftIndex), self.cruisePointWrap, self.cruisePointData, 10)
        SimBase.AddModelToTask("cruisePointTask" + str(spacecraftIndex), self.torqueScheduler1CruiseWrap, self.torqueScheduler1CruiseData, 10)
        SimBase.AddModelToTask("cruisePointTask" + str(spacecraftIndex), self.torqueScheduler2CruiseWrap, self.torqueScheduler2CruiseData, 10)

        for item in range(SimBase.DynModels[spacecraftIndex].numRSA):
            SimBase.AddModelToTask("solarArrayReferenceTask" + str(spacecraftIndex), self.solarArrayReferenceWrapList[item], self.solarArrayReferenceDataList[item], 7)
            SimBase.AddModelToTask("solarArrayReferenceTask" + str(spacecraftIndex), self.solarArrayControllerWrapList[item], self.solarArrayControllerDataList[item], 6)

        SimBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingErrorWrap, self.trackingErrorData, 4)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 3)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorqueWrap, self.rwMotorTorqueData, 2)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc[spacecraftIndex].disableAllTasks()

        SimBase.createNewEvent("initiateEarthPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'earthPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('thrusterPlatformReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('thrusterPlatformTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('earthPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateEarthPointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateSEPPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'sepPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('cmEstimationTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('thrusterPlatformReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('thrusterPlatformTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('sepPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateSEPPointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateCruisePointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'cruisePointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('thrusterPlatformReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('thrusterPlatformTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('cruisePointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('solarArrayReferenceTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateCruisePointing_" + str(spacecraftIndex) + "', True, useIndex=True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetEarthPointGuidance(self, SimBase):
        """
        Define Earth point guidance module
        """
        self.earthPointData.a1Hat_B = [1, 0, 0]        # solar array axis drive
        self.earthPointData.a2Hat_B = [0, 1, 0]        # antiparallel direction to the sensitive surface
        self.earthPointData.h1Hat_B = [0, 1, 0]        # high gain antenna 
        self.earthPointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.earthPointData.transNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.earthPointData.ephemerisInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.earth])
        messaging.AttRefMsg_C_addAuthor(self.earthPointData.attRefOutMsg, self.attRefMsg)

    def SetSEPPointGuidance(self, SimBase):
        """
        Defines SEP point guidance module
        """
        self.sepPointData.a1Hat_B = [1, 0, 0]          # solar array axis drive
        self.sepPointData.a2Hat_B = [0, 1, 0]          # antiparallel direction to the sensitive surface
        self.sepPointData.hHat_N = [1, 0, 0]           # random inertial thrust direction
        self.sepPointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        if self.thrusterFlag == 1:
            self.sepPointData.bodyHeadingInMsg.subscribeTo(self.platform1ReferenceData.bodyHeadingOutMsg)
        else:
            self.sepPointData.bodyHeadingInMsg.subscribeTo(self.platform2ReferenceData.bodyHeadingOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.sepPointData.attRefOutMsg, self.attRefMsg)

    def SetCruisePointGuidance(self, SimBase):
        """
        Defines Cruise point guidance module
        """
        self.cruisePointData.a1Hat_B = [1, 0, 0]       # solar array axis drive
        self.cruisePointData.a2Hat_B = [0, 1, 0]       # antiparallel direction to the sensitive surface
        self.cruisePointData.h1Hat_B = [0, 1, 0]       # low gain antenna #1
        self.cruisePointData.h2Hat_B = [0, -1, 0]      # low gain antenna #2
        self.cruisePointData.alignmentPriority = 1
        self.cruisePointData.attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.cruisePointData.transNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.cruisePointData.ephemerisInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.earth])
        messaging.AttRefMsg_C_addAuthor(self.cruisePointData.attRefOutMsg, self.attRefMsg)

    def SetSolarArrayReference(self, SimBase):
        """
        Defines the solar array reference request
        """
        # Define the reference for the first solar array
        self.solarArrayReferenceDataList[0].a1Hat_B = [1, 0, 0]
        self.solarArrayReferenceDataList[0].a2Hat_B = [0, 1, 0]
        self.solarArrayReferenceDataList[0].attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solarArrayReferenceDataList[0].attRefInMsg.subscribeTo(self.attRefMsg)
        self.solarArrayReferenceDataList[0].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[0].spinningBodyOutMsg)

        # Define the reference for the second solar array
        self.solarArrayReferenceDataList[1].a1Hat_B = [-1, 0, 0]
        self.solarArrayReferenceDataList[1].a2Hat_B = [0, 1, 0]
        self.solarArrayReferenceDataList[1].attNavInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solarArrayReferenceDataList[1].attRefInMsg.subscribeTo(self.attRefMsg)
        self.solarArrayReferenceDataList[1].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[1].spinningBodyOutMsg)

    def SetSolarArrayController(self, SimBase):
        """
        Defines the control torques to satisfy the requested solar array reference
        """
        # Define the controller for the first solar array
        self.solarArrayControllerDataList[0].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[0].spinningBodyOutMsg)
        self.solarArrayControllerDataList[0].hingedRigidBodyRefInMsg.subscribeTo(self.solarArrayReferenceDataList[0].hingedRigidBodyRefOutMsg)
        self.solarArrayControllerDataList[0].K = 1.25
        self.solarArrayControllerDataList[0].P = 50
        self.solarArrayControllerDataList[0].I = 3e-3

        # Define the controller for the second solar array
        self.solarArrayControllerDataList[1].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].RSAList[1].spinningBodyOutMsg)
        self.solarArrayControllerDataList[1].hingedRigidBodyRefInMsg.subscribeTo(self.solarArrayReferenceDataList[1].hingedRigidBodyRefOutMsg)
        self.solarArrayControllerDataList[1].K = 1.25
        self.solarArrayControllerDataList[1].P = 50
        self.solarArrayControllerDataList[1].I = 3e-3

    def SetPlatform1Reference(self, SimBase):
        """
        Defines the thruster platform reference logic
        """
        c = np.cos(15*mc.D2R)
        self.platform1ReferenceData.sigma_MB = np.array([1,0,0])*np.tan(15/4*mc.D2R)
        self.platform1ReferenceData.r_BM_M = [0, 0, 1.43/c]
        self.platform1ReferenceData.r_FM_F = [0, 0, 0]
        self.platform1ReferenceData.K = 1e-4
        if self.cmEstimation:
            self.platform1ReferenceData.vehConfigInMsg.subscribeTo(self.cmEstimationData.vehConfigOutMsg)
        else:
            self.platform1ReferenceData.vehConfigInMsg.subscribeTo(self.fswVehConfigMsg)
            # self.platform1ReferenceData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.platform1ReferenceData.thrusterConfigFInMsg.subscribeTo(self.thrConfigFMsg)
        self.platform1ReferenceData.rwConfigDataInMsg.subscribeTo(self.fswRwConfigMsg)
        self.platform1ReferenceData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)

    def SetPlatform2Reference(self, SimBase):
        """
        Defines the thruster platform reference logic
        """
        c = np.cos(15*mc.D2R)
        self.platform2ReferenceData.sigma_MB = np.array([1,0,0])*np.tan(-15/4*mc.D2R)
        self.platform2ReferenceData.r_BM_M = [0, 0, 1.43/c]
        self.platform2ReferenceData.r_FM_F = [0, 0, 0]
        self.platform2ReferenceData.K = 1e-4
        if self.cmEstimation:
            self.platform2ReferenceData.vehConfigInMsg.subscribeTo(self.cmEstimationData.vehConfigOutMsg)
        else:
            self.platform2ReferenceData.vehConfigInMsg.subscribeTo(self.fswVehConfigMsg)
            # self.platform2ReferenceData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.platform2ReferenceData.thrusterConfigFInMsg.subscribeTo(self.thrConfigFMsg)
        self.platform2ReferenceData.rwConfigDataInMsg.subscribeTo(self.fswRwConfigMsg)
        self.platform2ReferenceData.rwSpeedsInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)

    def SetPlatform1State(self, SimBase):
        """
        Defines the thruster platform state module
        """
        self.platform1StateData.sigma_MB = self.platform1ReferenceData.sigma_MB
        self.platform1StateData.r_BM_M = self.platform1ReferenceData.r_BM_M
        self.platform1StateData.r_FM_F = self.platform1ReferenceData.r_FM_F
        self.platform1StateData.thrusterConfigFInMsg.subscribeTo(self.thrConfigFMsg)
        self.platform1StateData.hingedRigidBody1InMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform1.spinningBodyOutMsgs[0])
        self.platform1StateData.hingedRigidBody2InMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform1.spinningBodyOutMsgs[1])

    def SetPlatform2State(self, SimBase):
        """
        Defines the thruster platform state module
        """
        self.platform2StateData.sigma_MB = self.platform2ReferenceData.sigma_MB
        self.platform2StateData.r_BM_M = self.platform2ReferenceData.r_BM_M
        self.platform2StateData.r_FM_F = self.platform2ReferenceData.r_FM_F
        self.platform2StateData.thrusterConfigFInMsg.subscribeTo(self.thrConfigFMsg)
        self.platform2StateData.hingedRigidBody1InMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform2.spinningBodyOutMsgs[0])
        self.platform2StateData.hingedRigidBody2InMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform2.spinningBodyOutMsgs[1])

    def SetPlatform1Controller(self, SimBase):
        """
        Defines the control torques to satisfy the requested platform reference
        """
        # Define the controller for the first platform angle
        self.platform1ControllerDataList[0].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform1.spinningBodyOutMsgs[0])
        self.platform1ControllerDataList[0].hingedRigidBodyRefInMsg.subscribeTo(self.platform1ReferenceData.hingedRigidBodyRef1OutMsg)
        self.platform1ControllerDataList[0].K = 0.5
        self.platform1ControllerDataList[0].P = 3

        # Define the controller for the second platform angle
        self.platform1ControllerDataList[1].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform1.spinningBodyOutMsgs[1])
        self.platform1ControllerDataList[1].hingedRigidBodyRefInMsg.subscribeTo(self.platform1ReferenceData.hingedRigidBodyRef2OutMsg)
        self.platform1ControllerDataList[1].K = 0.5
        self.platform1ControllerDataList[1].P = 3

    def SetPlatform2Controller(self, SimBase):
        """
        Defines the control torques to satisfy the requested platform reference
        """
        # Define the controller for the first platform angle
        self.platform2ControllerDataList[0].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform2.spinningBodyOutMsgs[0])
        self.platform2ControllerDataList[0].hingedRigidBodyRefInMsg.subscribeTo(self.platform2ReferenceData.hingedRigidBodyRef1OutMsg)
        self.platform2ControllerDataList[0].K = 0.5
        self.platform2ControllerDataList[0].P = 3

        # Define the controller for the second platform angle
        self.platform2ControllerDataList[1].hingedRigidBodyInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].platform2.spinningBodyOutMsgs[1])
        self.platform2ControllerDataList[1].hingedRigidBodyRefInMsg.subscribeTo(self.platform2ReferenceData.hingedRigidBodyRef2OutMsg)
        self.platform2ControllerDataList[1].K = 0.5
        self.platform2ControllerDataList[1].P = 3

    def SetTorqueScheduler1Earth(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler1EarthData.lockFlag = 3
        self.torqueScheduler1EarthData.tSwitch = 60
        self.torqueScheduler1EarthData.motorTorque1InMsg.subscribeTo(self.platform1ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler1EarthData.motorTorque2InMsg.subscribeTo(self.platform1ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler1EarthData.motorTorqueOutMsg, self.platform1TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler1EarthData.effectorLockOutMsg, self.platform1LockMsg)

    def SetTorqueScheduler2Earth(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler2EarthData.lockFlag = 3
        self.torqueScheduler2EarthData.tSwitch = 60
        self.torqueScheduler2EarthData.motorTorque1InMsg.subscribeTo(self.platform2ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler2EarthData.motorTorque2InMsg.subscribeTo(self.platform2ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler2EarthData.motorTorqueOutMsg, self.platform2TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler2EarthData.effectorLockOutMsg, self.platform2LockMsg)

    def SetTorqueScheduler1SEP(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler1SepData.lockFlag = 0
        self.torqueScheduler1SepData.tSwitch = 60
        self.torqueScheduler1SepData.motorTorque1InMsg.subscribeTo(self.platform1ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler1SepData.motorTorque2InMsg.subscribeTo(self.platform1ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler1SepData.motorTorqueOutMsg, self.platform1TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler1SepData.effectorLockOutMsg, self.platform1LockMsg)

    def SetTorqueScheduler2SEP(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler2SepData.lockFlag = 0
        self.torqueScheduler2SepData.tSwitch = 60
        self.torqueScheduler2SepData.motorTorque1InMsg.subscribeTo(self.platform2ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler2SepData.motorTorque2InMsg.subscribeTo(self.platform2ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler2SepData.motorTorqueOutMsg, self.platform2TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler2SepData.effectorLockOutMsg, self.platform2LockMsg)

    def SetTorqueScheduler1Cruise(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler1CruiseData.lockFlag = 3
        self.torqueScheduler1CruiseData.tSwitch = 60
        self.torqueScheduler1CruiseData.motorTorque1InMsg.subscribeTo(self.platform1ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler1CruiseData.motorTorque2InMsg.subscribeTo(self.platform1ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler1CruiseData.motorTorqueOutMsg, self.platform1TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler1CruiseData.effectorLockOutMsg, self.platform1LockMsg)

    def SetTorqueScheduler2Cruise(self):
        """
        Defines the torque scheduler module that outputs the torque msg for the platform
        """
        self.torqueScheduler2CruiseData.lockFlag = 3
        self.torqueScheduler2CruiseData.tSwitch = 60
        self.torqueScheduler2CruiseData.motorTorque1InMsg.subscribeTo(self.platform1ControllerDataList[0].motorTorqueOutMsg)
        self.torqueScheduler2CruiseData.motorTorque2InMsg.subscribeTo(self.platform1ControllerDataList[1].motorTorqueOutMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.torqueScheduler2CruiseData.motorTorqueOutMsg, self.platform2TorqueMsg)
        messaging.ArrayEffectorLockMsg_C_addAuthor(self.torqueScheduler2CruiseData.effectorLockOutMsg, self.platform2LockMsg)

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
        self.decayTime = 40
        self.xi = 1.25
        self.mrpFeedbackRWsData.Ki = 1e-5  # make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWsData.K = (self.mrpFeedbackRWsData.P / self.xi) * (self.mrpFeedbackRWsData.P / self.xi) / np.max(SimBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1
        self.mrpFeedbackRWsData.controlLawType = 1
        # self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(self.fswVehConfigMsg)
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

    def SetCMEstimation(self, SimBase):
        """
        Defines the module to estimate the center of mass
        """
        self.cmEstimationData.attitudeTol = 1e-5
        self.cmEstimationData.r_CB_B = [0.14, 0.00, 1.25] # Real CoM_B location = [0.113244, 0.025605, 1.239834]
        self.cmEstimationData.P0 = [0.0025, 0.0025, 0.0025]
        self.cmEstimationData.R0 = [4e-10, 4e-10, 4e-10]
        if self.thrusterFlag == 1:
            self.cmEstimationData.thrusterConfigBInMsg.subscribeTo(self.platform1StateData.thrusterConfigBOutMsg)
        else:
            self.cmEstimationData.thrusterConfigBInMsg.subscribeTo(self.platform2StateData.thrusterConfigBOutMsg)
        self.cmEstimationData.intFeedbackTorqueInMsg.subscribeTo(self.mrpFeedbackRWsData.intFeedbackTorqueOutMsg)
        self.cmEstimationData.attGuidInMsg.subscribeTo(self.attGuidMsg)
        self.cmEstimationData.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """
        Initializes all FSW objects
        """
        self.SetRWConfigMsg(SimBase)
        self.SetEarthPointGuidance(SimBase)
        self.SetSEPPointGuidance(SimBase)
        self.SetCruisePointGuidance(SimBase)
        self.SetSolarArrayReference(SimBase)
        self.SetSolarArrayController(SimBase)
        self.SetPlatform1Reference(SimBase)
        self.SetPlatform2Reference(SimBase)
        self.SetPlatform1Controller(SimBase)
        self.SetPlatform2Controller(SimBase)
        self.SetPlatform1State(SimBase)
        self.SetPlatform2State(SimBase)
        self.SetTorqueScheduler1Earth()
        self.SetTorqueScheduler1Earth()
        self.SetTorqueScheduler1SEP()
        self.SetTorqueScheduler2SEP()
        self.SetTorqueScheduler1Cruise()
        self.SetTorqueScheduler2Cruise()
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetRWMotorTorque()
        self.SetCMEstimation(SimBase)

    def setupGatewayMsgs(self, SimBase):
        """
        Creates C-wrapped gateway messages such that different modules can write to and provide a common input msg for
        downstream modules
        """
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.platform1TorqueMsg = messaging.ArrayMotorTorqueMsg_C()
        self.platform1LockMsg = messaging.ArrayEffectorLockMsg_C()
        self.platform2TorqueMsg = messaging.ArrayMotorTorqueMsg_C()
        self.platform2LockMsg = messaging.ArrayEffectorLockMsg_C()
        self.thrConfigFMsg = messaging.THRConfigMsg_C()
        self.fswVehConfigMsg = messaging.VehicleConfigMsg_C()
       
        # Write fsw configuration message
        VehicleConfig = messaging.VehicleConfigMsgPayload()
        VehicleConfig.ISCPntB_B = [5720, -228, 134, -228, 11766, -203, 134, -203, 7570]
        VehicleConfig.CoM_B = [0.1, 0.02, 1.23] # [0.01, -0.01, 1.15]
        VehicleConfig.massSC = 2215
        self.fswVehConfigMsg = messaging.VehicleConfigMsg().write(VehicleConfig)

        # Write THR Config Msg with initial best estimates
        THRConfig = messaging.THRConfigMsgPayload()
        THRConfig.rThrust_B = [0, 0, 0]
        THRConfig.tHatThrust_B = [0, 0, 1]
        THRConfig.maxThrust = 0.27
        self.thrConfigFMsg = messaging.THRConfigMsg().write(THRConfig)

        # Configure a single thruster firing, create a message for it
        ThrustMessage1 = messaging.THRArrayOnTimeCmdMsgPayload()
        ThrustMessage2 = messaging.THRArrayOnTimeCmdMsgPayload()
        if self.thrusterFlag == 1:
            ThrustMessage1.OnTimeRequest = [0.85*7*24*3600]
            ThrustMessage2.OnTimeRequest = [0]
        else:
            ThrustMessage1.OnTimeRequest = [0]
            ThrustMessage2.OnTimeRequest = [0.85*7*24*3600]
        self.thrCmdMsg1 = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage1)
        self.thrCmdMsg2 = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage2)

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.rwMotorTorqueData.rwMotorTorqueOutMsg)
        SimBase.DynModels[self.spacecraftIndex].platform1.motorTorqueInMsg.subscribeTo(self.platform1TorqueMsg)
        SimBase.DynModels[self.spacecraftIndex].platform2.motorTorqueInMsg.subscribeTo(self.platform2TorqueMsg)
        SimBase.DynModels[self.spacecraftIndex].platform1.motorLockInMsg.subscribeTo(self.platform1LockMsg)
        SimBase.DynModels[self.spacecraftIndex].platform2.motorLockInMsg.subscribeTo(self.platform2LockMsg)
        for item in range(SimBase.DynModels[self.spacecraftIndex].numRSA):
            SimBase.DynModels[self.spacecraftIndex].RSAList[item].motorTorqueInMsg.subscribeTo(self.solarArrayControllerDataList[item].motorTorqueOutMsg)
        SimBase.DynModels[self.spacecraftIndex].SEPThruster1.cmdsInMsg.subscribeTo(self.thrCmdMsg1)
        SimBase.DynModels[self.spacecraftIndex].SEPThruster2.cmdsInMsg.subscribeTo(self.thrCmdMsg2)

    def zeroGateWayMsgs(self):
        """
        Zeroes all the FSW gateway message payloads
        """
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.platform1TorqueMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.platform1LockMsg.write(messaging.ArrayEffectorLockMsgPayload())
        self.platform2TorqueMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.platform2LockMsg.write(messaging.ArrayEffectorLockMsgPayload())
