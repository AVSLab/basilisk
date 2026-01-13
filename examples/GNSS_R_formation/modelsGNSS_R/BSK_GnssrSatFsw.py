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

import itertools

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (
    attTrackingError,
    inertial3D,
    locationPointing,
    mrpFeedback,
    rwMotorTorque,
    scanningInstrumentController,
    spacecraftReconfig,
    tamComm,
    mtbMomentumManagement,
    scanningInstrumentController
#    gnssrSensing # TODO add (cpp code) in src/fswAlgorithms/....
)
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros as mc


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
        self.mtbParamsInMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        #--- Create tasks ---#
        # [0] Pointing at a fixed inertial attitude in the {N} frame
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("inertialPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Point the solar panels to the sun
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("solarChargingTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Point nadir towards the Earth
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("nadirPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Reconfigure the spacecraft formation
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("spacecraftReconfigTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 15)
        # Compute the attitude tracking error TODO is this needed?
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex), #TODO when is this needed ?
                                                                       self.processTasksTimeStep), 10)
        # MRP feedback attitude control with reaction wheels
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex), #TODO when is this needed ?
                                                                       self.processTasksTimeStep), 5)
        # Reaction wheel momentum dumping with magnetorquers/TAM
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("rwMomentumDumpTask" + str(spacecraftIndex),
                                                                          self.processTasksTimeStep), 5)
#        # GNSS-R sensing mode
#        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("GnssR" + str(spacecraftIndex),
#                                                                       self.processTasksTimeStep), 10)
        # simple instrument controller
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("scanningInstrumentControllerTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Data transfer mode
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("DataTransferTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)

        #--- Create module data and module wraps ---#
        # FSW "state related" modules
        # Spacecrafts after "release and detumbling"
        self.inertial3DPoint = inertial3D.inertial3D() # Inertial pointing module
        self.inertial3DPoint.ModelTag = "inertial3D"

        # Charging the spacecraft with solar pannels
        self.solCharging = locationPointing.locationPointing()
        self.solCharging.ModelTag = "solarCharging"

        self.nadirPoint = locationPointing.locationPointing()
        self.nadirPoint.ModelTag = "nadirPoint"

        self.spacecraftReconfig = spacecraftReconfig.spacecraftReconfig()
        self.spacecraftReconfig.ModelTag = "spacecraftReconfig"

        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"

        # FSW "actuator/sensor related" modules
        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        self.tamComm = tamComm.tamComm()
        self.tamComm.ModelTag = "tamComm"

        self.mtbMomentumManagement = mtbMomentumManagement.mtbMomentumManagement()
        self.mtbMomentumManagement.ModelTag = "mtbMomentumManagement"

        self.scanningInstrumentController = scanningInstrumentController.scanningInstrumentController() #TODO what is: scanningInstrumentController.scanningInstrumentControllerConfig()
        self.scanningInstrumentController.ModelTag = "scanningInstrumentController"

#        self.gnssrSensing = gnssrSensing.GnssrSensing()
#        self.gnssrSensing.ModelTag = "gnssrSensing"

#        self.dataTransfer = dataTransfer.dataTransfer()
#        self.dataTransfer.ModelTag = "dataTransfer"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertialPointTask" + str(spacecraftIndex), self.inertial3DPoint, 10)

        SimBase.AddModelToTask("solarChargingTask" + str(spacecraftIndex), self.solCharging, 10)

        SimBase.AddModelToTask("nadirPointTask" + str(spacecraftIndex), self.nadirPoint, 10)

        SimBase.AddModelToTask("spacecraftReconfigTask" + str(spacecraftIndex), self.spacecraftReconfig, 10)

        SimBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingError, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWs, 7)
        SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorque, 6)

        SimBase.AddModelToTask("rwMomentumDumpTask" + str(spacecraftIndex), self.mtbMomentumManagement, 5)
        SimBase.AddModelToTask("rwMomentumDumpTask" + str(spacecraftIndex), self.tamComm, 5)

        SimBase.AddModelToTask("scanningInstrumentControllerTask" + str(spacecraftIndex), self.scanningInstrumentController, 8)

#        SimBase.AddModelToTask("gnssRTask" + str(spacecraftIndex), self.gnssrSensing, 9)

#        SimBase.AddModelToTask("GnssR" + str(spacecraftIndex), self.gnssrSensing, 9)

#        SimBase.AddModelToTask("DataTransferTask" + str(spacecraftIndex), self.dataTransfer, 9)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc[spacecraftIndex].disableAllTasks()

        # ------------------------------------------------------------------------------------------- #
        #-- Event Definitions --# -> This are the initialization events for each type of FSW mode

        # The standby event should not be active while the station keeping mode is also active. Standby mode disables
        # attitude control and therefore the attitude cannot be corrected for orbital correction burns.
        SimBase.createNewEvent(
            "initiateStandby_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "standby"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                self.setAllButCurrentEventActivity(
                    f"initiateStandby_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        # Initiate Task: "Inertial Pointing"
        SimBase.createNewEvent(
            "initiateInertialPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "inertialPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                self.enableTask(f"inertialPointTask{spacecraftIndex}"),
                self.enableTask(f"trackingErrorTask{spacecraftIndex}"),
                self.enableTask(f"mrpFeedbackRWsTask{spacecraftIndex}"),
                self.setAllButCurrentEventActivity(
                    f"initiateInertialPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        SimBase.createNewEvent(
            "initiateSolCharging_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "solarCharging"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                self.enableTask(f"solarChargingTask{spacecraftIndex}"),
                self.enableTask(f"trackingErrorTask{spacecraftIndex}"),
                self.enableTask(f"mrpFeedbackRWsTask{spacecraftIndex}"),
                self.setAllButCurrentEventActivity(
                    f"initiateSolCharging_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        SimBase.createNewEvent(
            "initiateStationKeeping_" + str(spacecraftIndex), # TODO dont understand this! why: initiateStationKeeping -> setEventActivity to stopStationKeeping
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].stationKeeping == "ON"
            ),
            actionFunction=lambda self: (
                self.enableTask(f"spacecraftReconfigTask{spacecraftIndex}"),
                self.setEventActivity(f"stopStationKeeping_{spacecraftIndex}", True),
            ),
        )
        SimBase.createNewEvent(
            "stopStationKeeping_" + str(spacecraftIndex),# TODO dont understand this! why: stopStationKeeping -> setEventActivity to initiateStationKeeping
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].stationKeeping == "OFF"
            ),
            actionFunction=lambda self: (
                self.disableTask(f"spacecraftReconfigTask{spacecraftIndex}"),
                self.setEventActivity(
                    f"initiateStationKeeping_{spacecraftIndex}", True
                ),
            ),
        )

        SimBase.createNewEvent(
            "initiateNadirPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "nadirPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                self.enableTask(f"nadirPointTask{spacecraftIndex}"),
                self.enableTask(f"trackingErrorTask{spacecraftIndex}"),
                self.enableTask(f"mrpFeedbackRWsTask{spacecraftIndex}"),
                self.setAllButCurrentEventActivity(
                    f"initiateNadirPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

#TODO should there be a formationReconfiguraton event

#        SimBase.createNewEvent(
#            "initiateGnssRMode_" + str(spacecraftIndex),
#            self.processTasksTimeStep,
#            True,
#            conditionFunction=lambda self: (
#                self.FSWModels[spacecraftIndex].modeRequest == "gnssrSensing"
#            ),
#            actionFunction=lambda self: (
#                self.fswProc[spacecraftIndex].disableAllTasks(),
#                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
#                self.enableTask(f"gnssRTask{spacecraftIndex}"),
#                self.setAllButCurrentEventActivity(
#                    f"initiateGnssRMode_{spacecraftIndex}", True, useIndex=True
#                ),
#            ),
#        )

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        """
        Defines the inertial pointing guidance module.
        """
        self.inertial3DPoint.sigma_R0N = [0.1, 0.2, -0.3]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DPoint.attRefOutMsg, self.attRefMsg)

    def SetSolarChargingGuidance(self, SimBase):
        """
        Defines the solar-cells to sun pointing guidance module.
        """
        self.solCharging.pHat_B = SimBase.DynModels[self.spacecraftIndex].solarPanelAxis
        self.solCharging.scAttInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solCharging.scTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.solCharging.celBodyInMsg.subscribeTo(
            SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.gravBodyList.index('sun')])
        messaging.AttRefMsg_C_addAuthor(self.solCharging.attRefOutMsg, self.attRefMsg)

    def SetNadirPointGuidance(self, SimBase):
        """
        Defines the Earth location pointing guidance module.
        """
        self.nadirPoint.pHat_B = [0, 0, -1] # Pointing "bottom" of the spacecraft to the location (GNSS-R sensors are on the 'flat underside' of the satellite)
        self.nadirPoint.scAttInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.nadirPoint.scTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.nadirPoint.celBodyInMsg.subscribeTo(
            SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.gravBodyList.index('earth')])
        messaging.AttRefMsg_C_addAuthor(self.nadirPoint.attRefOutMsg, self.attRefMsg)

    def SetSpacecraftOrbitReconfig(self, SimBase): #TODO check if this is correct (spacecraft should reconfigure formation in position and attitude)
        """
        Defines the station keeping module.
        """
        self.spacecraftReconfig.deputyTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.spacecraftReconfig.attRefInMsg.subscribeTo(self.attRefMsg)                  #TODO why is this subscribed to attRefMsg? and not to the attOutMsg of the nav module?
        self.spacecraftReconfig.thrustConfigInMsg.subscribeTo(self.fswThrusterConfigMsg) #TODO same here, why subscribe to thrusterConfigMsg from FSW and not to the thruster msg from the dynamics?
        self.spacecraftReconfig.vehicleConfigInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.spacecraftReconfig.mu = SimBase.EnvModel.mu  # [m^3/s^2]
        self.spacecraftReconfig.attControlTime = 400  # [s]
        messaging.AttRefMsg_C_addAuthor(self.spacecraftReconfig.attRefOutMsg, self.attRefMsg)

        # connect a blank chief message
        chiefData = messaging.NavTransMsgPayload()                   #TODO Why is this needed?
        chiefMsg = messaging.NavTransMsg().write(chiefData)          #TODO what is written to this message?
        self.spacecraftReconfig.chiefTransInMsg.subscribeTo(chiefMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """
        Defines the module that converts a reference message into a guidance message.
        """
        self.trackingError.attNavInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)

    def SetMRPFeedbackRWA(self, SimBase):
        """
        Defines the control properties.
        """
        self.decayTime = 50
        self.xi = 0.9
        self.mrpFeedbackRWs.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWs.P = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWs.K = (self.mrpFeedbackRWs.P / self.xi) * \
                                    (self.mrpFeedbackRWs.P / self.xi) / np.max(
            SimBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)

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

    def SetMtbConfigMsg(self, SimBase):
        """
        Imports the MTB configuration information.
        """
        # Configure MTB exactly as it is in the Dynamics
        self.mtbParamsInMsg = SimBase.DynModels[self.spacecraftIndex].mtbParamsInMsg

    def SetRWMotorTorque(self):
        """
        Defines the motor torque from the control law.
        """
        # Make the RW control all three body axes
        controlAxes_B = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.mrpFeedbackRWs.cmdTorqueOutMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    def setupTamComm(self, SimBase):
        """
        Defines the TAM communication module.
        """
        self.tamComm.dcm_BS = [1., 0., 0.,
                               0., 1., 0.,
                               0., 0., 1.]
        self.tamComm.tamInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].tam.tamDataOutMsg)
#        tamCommLog = self.tamComm.tamOutMsg.recorder(samplingTime) # TODO add logging if needed; Put to the right place

    def setupMtbMomentumManagement(self, SimBase):
        """
        Defines the MTB momentum management module.
        """
        self.mtbMomentumManagement.wheelSpeedBiases = [0.0, 0.0, 0.0, 0.0] # TODO is this feasible? confirm
        self.mtbMomentumManagement.cGain = 0.003 # TODO understand what this is

        self.mtbMomentumManagement.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mtbMomentumManagement.mtbParamsInMsg.subscribeTo(self.mtbParamsInMsg)
        self.mtbMomentumManagement.tamSensorBodyInMsg.subscribeTo(self.tamComm.tamOutMsg)
        self.mtbMomentumManagement.rwSpeedsInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mtbMomentumManagement.rwMotorTorqueInMsg.subscribeTo(self.rwMotorTorque.rwMotorTorqueOutMsg)

        SimBase.DynModels[self.spacecraftIndex].mtbEff.mtbParamsInMsg.subscribeTo(self.mtbParamsInMsg)
        SimBase.DynModels[self.spacecraftIndex].mtbEff.mtbCmdInMsg.subscribeTo(self.mtbMomentumManagement.mtbCmdOutMsg)
#        mtbDipoleCmdsLog = self.mtbMomentumManagement.mtbCmdOutMsg.recorder(samplingTime)

    def setupScanningInstrumentControler(self, SimBase):
        """
        Defines the simple instrument controller module.
        """
        self.scanningInstrumentController.useRateTolerance = 1
        self.scanningInstrumentController.rateErrTolerance = 0.01
        self.scanningInstrumentController.attErrTolerance = 0.1
        self.scanningInstrumentController.attGuidInMsg.subscribeTo(self.nadirPoint.attGuidOutMsg)
        self.scanningInstrumentController.accessInMsg.subscribeTo(
            SimBase.EnvModel.groundStationBar.accessOutMsgs[self.spacecraftIndex])

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """
        Initializes all FSW objects.
        """
        # Get configuration messages from the Dynamics modules
        self.SetRWConfigMsg(SimBase)
        self.SetThrustersConfigMsg(SimBase)
        self.SetMtbConfigMsg(SimBase)
        # Initialize all modules
        self.SetInertial3DPointGuidance()
        self.SetSolarChargingGuidance(SimBase)
        self.SetNadirPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetRWMotorTorque()
        self.SetSpacecraftOrbitReconfig(SimBase)
        self.setupTamComm(SimBase)
        self.setupMtbMomentumManagement(SimBase)
        self.setupScanningInstrumentControler(SimBase)

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(
            self.rwMotorTorque.rwMotorTorqueOutMsg)
        SimBase.DynModels[self.spacecraftIndex].thrusterDynamicEffector.cmdsInMsg.subscribeTo(
            self.spacecraftReconfig.onTimeOutMsg) #                             TODO what is the "on Time Out Msg"?
        SimBase.DynModels[self.spacecraftIndex].mtbEff.mtbCmdInMsg.subscribeTo(
            self.mtbMomentumManagement.mtbCmdOutMsg)

    def zeroGateWayMsgs(self):
        """Zero all FSW gateway message payloads"""
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())

        # Zero all actuator commands
        self.rwMotorTorque.rwMotorTorqueOutMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.spacecraftReconfig.onTimeOutMsg.write(messaging.THRArrayOnTimeCmdMsgPayload())
        self.mtbMomentumManagement.mtbCmdOutMsg.write(messaging.MTBCmdMsgPayload())
