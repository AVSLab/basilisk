#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.utilities import (macros as mc, fswSetupThrusters)
from Basilisk.fswAlgorithms import (inertial3D, locationPointing, attTrackingError, mrpFeedback,
                                    rwMotorTorque, spacecraftReconfig)
from Basilisk.architecture import messaging
import Basilisk.architecture.cMsgCInterfacePy as cMsgPy

import numpy as np
import itertools


class BSKFswModels:
    """Defines the FSW class"""
    def __init__(self, SimBase, fswRate, spacecraftIndex):
        # define empty class variables
        self.spacecraftIndex = spacecraftIndex
        self.decayTime = None
        self.xi = None
        self.modeRequest = "standby"
        self.stationKeeping = "OFF"

        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.fswThrusterConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create tasks
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("inertialPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("sunPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("locPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("spacecraftReconfigTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 15)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 10)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 5)

        # Create module data and module wraps
        self.inertial3DPointData = inertial3D.inertial3DConfig()
        self.inertial3DPointWrap = SimBase.setModelDataWrap(self.inertial3DPointData)
        self.inertial3DPointWrap.ModelTag = "inertial3D"

        self.sunPointData = locationPointing.locationPointingConfig()
        self.sunPointWrap = SimBase.setModelDataWrap(self.sunPointData)
        self.sunPointWrap.ModelTag = "sunPoint"

        self.locPointData = locationPointing.locationPointingConfig()
        self.locPointWrap = SimBase.setModelDataWrap(self.locPointData)
        self.locPointWrap.ModelTag = "locPoint"

        self.spacecraftReconfigData = spacecraftReconfig.spacecraftReconfigConfig()
        self.spacecraftReconfigWrap = SimBase.setModelDataWrap(self.spacecraftReconfigData)
        self.spacecraftReconfigWrap.ModelTag = "spacecraftReconfig"

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
        SimBase.AddModelToTask("inertialPointTask" + str(spacecraftIndex), self.inertial3DPointWrap,
                               self.inertial3DPointData, 10)

        SimBase.AddModelToTask("sunPointTask" + str(spacecraftIndex), self.sunPointWrap, self.sunPointData, 10)

        SimBase.AddModelToTask("locPointTask" + str(spacecraftIndex), self.locPointWrap, self.locPointData, 10)

        SimBase.AddModelToTask("spacecraftReconfigTask" + str(spacecraftIndex), self.spacecraftReconfigWrap,
                               self.spacecraftReconfigData, 10)

        SimBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingErrorWrap,
                               self.trackingErrorData, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWsWrap,
                               self.mrpFeedbackRWsData, 7)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorqueWrap,
                               self.rwMotorTorqueData, 6)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc[spacecraftIndex].disableAllTasks()

        # The standby event should not be active while the station keeping mode is also active. Standby mode disables
        # attitude control and therefore the attitude cannot be corrected for orbital correction burns.
        SimBase.createNewEvent("initiateStandby_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'standby'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.setAllButCurrentEventActivity('initiateStandby_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateInertialPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'inertialPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('inertialPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateInertialPointing_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateSunPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'sunPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('sunPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateSunPointing_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateLocationPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'locationPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('locPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateLocationPointing_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateStationKeeping_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].stationKeeping == 'ON'"],
                               ["self.enableTask('spacecraftReconfigTask" + str(spacecraftIndex) + "')",
                                "self.setEventActivity('stopStationKeeping_" + str(spacecraftIndex) + "', True)"])
        SimBase.createNewEvent("stopStationKeeping_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].stationKeeping == 'OFF'"],
                               ["self.disableTask('spacecraftReconfigTask" + str(spacecraftIndex) + "')",
                                "self.setEventActivity('initiateStationKeeping_" + str(spacecraftIndex) + "', True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        """
        Defines the inertial pointing guidance module.
        """
        self.inertial3DPointData.sigma_R0N = [0.1, 0.2, -0.3]
        cMsgPy.AttRefMsg_C_addAuthor(self.inertial3DPointData.attRefOutMsg, self.attRefMsg)

    def SetSunPointGuidance(self, SimBase):
        """
        Defines the Sun pointing guidance module.
        """
        self.sunPointData.pHat_B = SimBase.DynModels[self.spacecraftIndex].solarPanelAxis
        self.sunPointData.scAttInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.sunPointData.scTransInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.sunPointData.celBodyInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.sun])
        cMsgPy.AttRefMsg_C_addAuthor(self.sunPointData.attRefOutMsg, self.attRefMsg)

    def SetLocationPointGuidance(self, SimBase):
        """
        Defines the Earth location pointing guidance module.
        """
        self.locPointData.pHat_B = [1, 0, 0]
        self.locPointData.scAttInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.locPointData.scTransInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.locPointData.locationInMsg.subscribeTo(SimBase.EnvModel.groundStation.currentGroundStateOutMsg)
        cMsgPy.AttRefMsg_C_addAuthor(self.locPointData.attRefOutMsg, self.attRefMsg)

    def SetSpacecraftOrbitReconfig(self, SimBase):
        """
        Defines the station keeping module.
        """
        self.spacecraftReconfigData.deputyTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.spacecraftReconfigData.attRefInMsg.subscribeTo(self.attRefMsg)
        self.spacecraftReconfigData.thrustConfigInMsg.subscribeTo(self.fswThrusterConfigMsg)
        self.spacecraftReconfigData.scMassDeputy = SimBase.DynModels[self.spacecraftIndex].scObject.hub.mHub  # [kg]
        self.spacecraftReconfigData.mu = SimBase.EnvModel.mu  # [m^3/s^2]
        self.spacecraftReconfigData.attControlTime = 600  # [s]
        cMsgPy.AttRefMsg_C_addAuthor(self.spacecraftReconfigData.attRefOutMsg, self.attRefMsg)

        # connect a blank chief message
        chiefData = messaging.NavTransMsgPayload()
        chiefMsg = messaging.NavTransMsg().write(chiefData)
        self.spacecraftReconfigData.chiefTransInMsg.subscribeTo(chiefMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """
        Defines the module that converts a reference message into a guidance message.
        """
        self.trackingErrorData.attNavInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.trackingErrorData.attRefInMsg.subscribeTo(self.attRefMsg)
        cMsgPy.AttGuidMsg_C_addAuthor(self.trackingErrorData.attGuidOutMsg, self.attGuidMsg)

    def SetMRPFeedbackRWA(self, SimBase):
        """
        Defines the control properties.
        """
        self.decayTime = 100
        self.xi = 0.5
        self.mrpFeedbackRWsData.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWsData.P = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWsData.K = (self.mrpFeedbackRWsData.P / self.xi) * \
                                    (self.mrpFeedbackRWsData.P / self.xi) / np.max(
            SimBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1

        self.mrpFeedbackRWsData.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWsData.rwSpeedsInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWsData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWsData.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRWConfigMsg(self, SimBase):
        """
        Imports the RWs configuration information.
        """
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        # the same msg is used here for both spacecraft
        self.fswRwConfigMsg = SimBase.DynModels[self.spacecraftIndex].rwFactory.getConfigMessage()

    def SetThrustersConfigMsg(self, SimBase):
        """
        Imports the thrusters configuration information.
        """
        fswSetupThrusters.clearSetup()
        for key, th in SimBase.DynModels[self.spacecraftIndex].thrusterFactory.thrusterList.items():
            loc_B_tmp = list(itertools.chain.from_iterable(th.thrLoc_B))
            dir_B_tmp = list(itertools.chain.from_iterable(th.thrDir_B))
            fswSetupThrusters.create(loc_B_tmp, dir_B_tmp, th.MaxThrust)
        self.fswThrusterConfigMsg = fswSetupThrusters.writeConfigMessage()

    def SetRWMotorTorque(self):
        """
        Defines the motor torque from the control law.
        """
        controlAxes_B = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.mrpFeedbackRWsData.cmdTorqueOutMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    def SetVehicleConfiguration(self, SimBase):
        """
        Defines the FSW vehicle configuration message.
        """
        # use the same inertia in the FSW algorithm as in the simulation
        vcData = messaging.VehicleConfigMsgPayload()
        vcData.ISCPntB_B = SimBase.DynModels[self.spacecraftIndex].I_sc
        vcData.massSC = SimBase.DynModels[self.spacecraftIndex].scObject.hub.mHub
        self.vcMsg = messaging.VehicleConfigMsg().write(vcData)

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """
        Initializes all FSW objects.
        """
        self.SetInertial3DPointGuidance()
        self.SetSunPointGuidance(SimBase)
        self.SetLocationPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetVehicleConfiguration(SimBase)
        self.SetRWConfigMsg(SimBase)
        self.SetThrustersConfigMsg(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetSpacecraftOrbitReconfig(SimBase)
        self.SetRWMotorTorque()

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.attRefMsg = cMsgPy.AttRefMsg_C()
        self.attGuidMsg = cMsgPy.AttGuidMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(
            self.rwMotorTorqueData.rwMotorTorqueOutMsg)
        SimBase.DynModels[self.spacecraftIndex].thrusterDynamicEffector.cmdsInMsg.subscribeTo(
            self.spacecraftReconfigData.onTimeOutMsg)

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
