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

from Basilisk.simulation.albedo import BSK_ERROR
import numpy as np
from Basilisk.architecture import astroConstants, messaging, sysModel
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
    spacecraftPointing,
    hillPoint,
    meanOEFeedback,#TODO this might not be needed!
    hillFrameRelativeControl,
)
from Basilisk.simulation import simpleAntenna
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros as mc
from Basilisk.utilities import orbitalMotion

#############################################################
###############        HELPER FUNCTIONS       ###############
#############################################################
def format_sim_time(nanos):
    """Convert nanoseconds to D:HH:MM:SS format."""
    total_seconds = int(nanos * mc.NANO2SEC)
    days, remainder = divmod(total_seconds, 86400)
    hours, remainder = divmod(remainder, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{days}:{hours:02d}:{minutes:02d}:{seconds:02d}"

class StateMachineModule(sysModel.SysModel):
    """
    State machine module run as a Basilisk task.
    """
    def __init__(self, fswModel):
        super().__init__()
        self.ModelTag = f"stateMachine_{fswModel.spacecraftIndex}"
        self.fswModel = fswModel
        self.this.disown()  # Prevent memory leaks

    def Reset(self, currentSimNanos):
        pass

    def UpdateState(self, currentSimNanos):
        # Run the state machine every timestep
        if self.fswModel.stateMachine:
            self.fswModel.setModeRequest(currentSimNanos=currentSimNanos)

########################################################
#####             Flight Software                  #####
########################################################
class BSKFswModels:
    """Defines the FSW class"""
    def __init__(self, SimBase, fswRate, spacecraftIndex):
        # define empty class variables
        self.spacecraftIndex = spacecraftIndex
        self.decayTime = None
        self.xi = None

        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.fswThrusterConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None
        self.mtbParamsInMsg = None
        self.antennaStateMsg = None
        self.chiefTransDefaultMsg = None  # zeroed placeholder until setFormationTargets() wires the real chief
        self.simBase = SimBase
        # Status variables
        self.modeRequest = "standby"
        self.stationKeeping = "OFF"
        self.reconfFormation = True
        self.timeInMode = 0
        self.flightTime = 0
        self.stateMachine = False
        self.verboseMode = False
        self.barycenterChiefSwitched = False
        self.reconfigStableTimeNanos = 0

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = self.simBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        #--- Create tasks ---#
        # [0] Pointing at a fixed inertial attitude in the {N} frame
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("inertialPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Point the solar panels to the sun
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("chargeBatteryTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Point nadir towards the Earth
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("nadirPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Point towards a specific location on Earth
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("locPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Reconfigure the spacecraft formation
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("spacecraftReconfigTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 15)
        # Compute the attitude tracking error TODO is this needed?
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex), #TODO when is this needed ?
                                                                       self.processTasksTimeStep), 10)
        # MRP feedback attitude control with reaction wheels
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex), #TODO when is this needed ?
                                                                       self.processTasksTimeStep), 5)
        # Reaction wheel momentum dumping with magnetorquers/TAM
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("rwMomentumDumpTask" + str(spacecraftIndex),
                                                                          self.processTasksTimeStep), 5)
#        # GNSS-R sensing mode
#        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("GnssR" + str(spacecraftIndex),
#                                                                       self.processTasksTimeStep), 10)
        # simple instrument controller
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("scanningInstrumentControllerTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        # Data transfer mode
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("dataTransferTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)

        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("spacecraftPointingTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)

        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("hillPointTask" + str(spacecraftIndex),
                                                                          self.processTasksTimeStep), 20)

        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("meanOEFeedbackTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 15)

        # Hill-frame relative PD station-keeping control
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("hillFrameRelativeControlTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 15)

        # State Machine Task
        self.simBase.fswProc[spacecraftIndex].addTask(self.simBase.CreateNewTask("stateMachineTask" + str(spacecraftIndex),
                                                                          self.processTasksTimeStep), 25)

        #--- Create module data and module wraps ---#
        # FSW "state related" modules
        # Spacecrafts after "release and detumbling"
        self.inertial3DPoint = inertial3D.inertial3D() # Inertial pointing module
        self.inertial3DPoint.ModelTag = "inertial3D"

        # Charging the spacecraft with solar pannels
        self.solCharging = locationPointing.locationPointing()
        self.solCharging.ModelTag = "chargeBattery"

        self.nadirPoint = locationPointing.locationPointing()
        self.nadirPoint.ModelTag = "nadirPoint"

        self.locPoint = locationPointing.locationPointing()
        self.locPoint.ModelTag = "locPoint"

        self.svalbardStationPoint = locationPointing.locationPointing()
        self.svalbardStationPoint.ModelTag = "pointingToSvalbardGroundStation"

        self.spacecraftReconfig = spacecraftReconfig.spacecraftReconfig()
        self.spacecraftReconfig.ModelTag = "spacecraftReconfig"

        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.trackingError2 = attTrackingError.attTrackingError()
        self.trackingError2.ModelTag = "trackingError2"

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

        self.spacecraftPointing = spacecraftPointing.spacecraftPointing()
        self.spacecraftPointing.ModelTag = "spacecraftPointing"

        self.meanOEFeedback = meanOEFeedback.meanOEFeedback()
        self.meanOEFeedback.ModelTag = "meanOEFeedback"

        self.hillFrameRelativeControl = hillFrameRelativeControl.HillFrameRelativeControl()
        self.hillFrameRelativeControl.ModelTag = "hillFrameRelativeControl"

        self.attGuidanceHillPoint = hillPoint.hillPoint()
        self.attGuidanceHillPoint.ModelTag = "hillPoint"

        # State machine module
        self.stateMachineModule = StateMachineModule(self)

#        self.gnssrSensing = gnssrSensing.GnssrSensing()
#        self.gnssrSensing.ModelTag = "gnssrSensing"

#        self.dataTransfer = dataTransfer.dataTransfer()
#        self.dataTransfer.ModelTag = "dataTransfer"

        # create the FSW module gateway messages
        self.setupGatewayMsgs()

        # Initialize all modules
        self.InitAllFSWObjects()

        # Assign initialized modules to tasks
        self.simBase.AddModelToTask("inertialPointTask" + str(spacecraftIndex), self.inertial3DPoint, 10)

        self.simBase.AddModelToTask("chargeBatteryTask" + str(spacecraftIndex), self.solCharging, 10)

        self.simBase.AddModelToTask("nadirPointTask" + str(spacecraftIndex), self.nadirPoint, 10)

        self.simBase.AddModelToTask("locPointTask" + str(spacecraftIndex), self.locPoint, 10)

        self.simBase.AddModelToTask("spacecraftReconfigTask" + str(spacecraftIndex), self.spacecraftReconfig, 10)

        self.simBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingError, 9)

        self.simBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWs, 7)
        self.simBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorque, 6)

        self.simBase.AddModelToTask("rwMomentumDumpTask" + str(spacecraftIndex), self.mtbMomentumManagement, 5)
        self.simBase.AddModelToTask("rwMomentumDumpTask" + str(spacecraftIndex), self.tamComm, 5)

        self.simBase.AddModelToTask("dataTransferTask" + str(spacecraftIndex), self.svalbardStationPoint, 9)

        self.simBase.AddModelToTask("spacecraftPointingTask" + str(spacecraftIndex), self.spacecraftPointing, 10)
        self.simBase.AddModelToTask("spacecraftPointingTask" + str(spacecraftIndex), self.trackingError, 7)

        self.simBase.AddModelToTask("hillPointTask" + str(spacecraftIndex), self.attGuidanceHillPoint, 10)

        self.simBase.AddModelToTask("meanOEFeedbackTask" + str(spacecraftIndex), self.meanOEFeedback, 10)

        self.simBase.AddModelToTask("hillFrameRelativeControlTask" + str(spacecraftIndex), self.hillFrameRelativeControl, 10)

        self.simBase.AddModelToTask("scanningInstrumentControllerTask" + str(spacecraftIndex), self.scanningInstrumentController, 8)

        # Create state machine module
        # State-Machine created as task because they (tasks) are always runing -> Low priority - runs after other modules
        self.simBase.AddModelToTask(f"stateMachineTask" + str(spacecraftIndex), self.stateMachineModule, 1)

#        self.simBase.AddModelToTask("gnssrTask" + str(spacecraftIndex), self.gnssrSensing, 9)

        # Create events to be called for triggering GN&C maneuvers
        self.simBase.fswProc[spacecraftIndex].disableAllTasks()
        self.simBase.enableTask("stateMachineTask" + str(spacecraftIndex)) # Re-enable state-machine task

        # ------------------------------------------------------------------------------------------- #
        #-- Event Definitions --# -> This are the initialization events for each type of FSW mode

        # The standby event should not be active while the station keeping mode is also active. Standby mode disables
        # attitude control and therefore the attitude cannot be corrected for orbital correction burns.
        self.simBase.createNewEvent(
            "initiateStandby_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "standby"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                self.setAllButCurrentEventActivity(
                    f"initiateStandby_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        # Initiate Task: "Inertial Pointing"
        self.simBase.createNewEvent(
            "initiateInertialPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "inertialPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for inertial pointing
                self.enableTask("inertialPointTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.setAllButCurrentEventActivity(
                    f"initiateInertialPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateSolCharging_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "chargeBattery"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for solar charging pointing
                self.enableTask("chargeBatteryTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.setAllButCurrentEventActivity(
                    f"initiateSolCharging_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateStationKeeping_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "initiateStationKeeping"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]),
                self.enableTask("spacecraftReconfigTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)),
                self.setEventActivity(f"stopStationKeeping_{spacecraftIndex}", True),
                setattr(self.FSWModels[spacecraftIndex], 'stationKeeping', 'ON'),
                self.setAllButCurrentEventActivity(
                    f"initiateStationKeeping_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )
        self.simBase.createNewEvent(
            "stopStationKeeping_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "stopStationKeeping"
            ),
            actionFunction=lambda self: (
                self.disableTask(f"spacecraftReconfigTask{spacecraftIndex}"),
                self.setEventActivity(f"initiateStationKeeping_{spacecraftIndex}", True),
                setattr(self.FSWModels[spacecraftIndex], 'stationKeeping', 'OFF'),
            ),
        )

        self.simBase.createNewEvent(
            "initiateNadirPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "nadirPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for nadir pointing
                self.enableTask("nadirPointTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.setAllButCurrentEventActivity(
                    f"initiateNadirPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateLocationPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "locationPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for location pointing
                self.enableTask(f"locPointTask{spacecraftIndex}"),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.setAllButCurrentEventActivity(
                    f"initiateLocationPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateDataTransfer_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "dataTransfer"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for data transfer pointing (pointing to ground station)
                self.enableTask("dataTransferTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.FSWModels[spacecraftIndex].antennaStateMsg.write(
                    messaging.AntennaStateMsgPayload(antennaState=simpleAntenna.ANTENNA_TX)
                ),
                self.setAllButCurrentEventActivity(
                    f"initiateDataTransfer_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateSpacecraftPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "spacecraftPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]), # Reset sigma_R0R to default value for generic spacecraft pointing
                self.enableTask("spacecraftPointingTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)),
                self.setAllButCurrentEventActivity(
                    f"initiateSpacecraftPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiateHillPointing_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "hillPointing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R',
                        [-1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0]), # Set sigma_R0R for hill pointing
                self.FSWModels[spacecraftIndex].switchToLiveBarycenterChief(
                    verbose=self.FSWModels[spacecraftIndex].verboseMode
                ),
                self.enableTask("hillPointTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)),
                self.setAllButCurrentEventActivity(
                    f"initiateHillPointing_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "startMeanOEFeedback_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "startMeanOEFeedback"
            ),
            actionFunction=lambda self: (
                self.enableTask("meanOEFeedbackTask" + str(spacecraftIndex)),
                self.setEventActivity(f"stopStationKeeping_{spacecraftIndex}", True),
                setattr(self.FSWModels[spacecraftIndex], 'stationKeeping', 'ON'),
            ),
        )
        self.simBase.createNewEvent(
            "stopMeanOEFeedback_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "stopMeanOEFeedback"
            ),
            actionFunction=lambda self: (
                self.disableTask(f"meanOEFeedbackTask{spacecraftIndex}"),
                self.setEventActivity(f"initiateStationKeeping_{spacecraftIndex}", True),
                setattr(self.FSWModels[spacecraftIndex], 'stationKeeping', 'OFF'),
            ),
        )

        self.simBase.createNewEvent(
            "initiateGnssRMode_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "startGnssrSensing"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]),
                self.enableTask(f"nadirPointTask{spacecraftIndex}"),
                self.enableTask(f"trackingErrorTask{spacecraftIndex}"),
                self.enableTask(f"mrpFeedbackRWsTask{spacecraftIndex}"),
                self.enableTask(f"scanningInstrumentControllerTask{spacecraftIndex}"),  # Enable instrument task
                self.setEventActivity("stopStationKeeping_" + str(spacecraftIndex), True),   # Stop station keeping
                self.enableTask("stateMachineTask" + str(spacecraftIndex)), # Re-enable state-machine task
                self.setAllButCurrentEventActivity(
                    f"initiateGnssRMode_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

        self.simBase.createNewEvent(
            "initiatePidStationKeeping_" + str(spacecraftIndex),
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.FSWModels[spacecraftIndex].modeRequest == "pidStationKeeping"
            ),
            actionFunction=lambda self: (
                self.fswProc[spacecraftIndex].disableAllTasks(),
                self.FSWModels[spacecraftIndex].zeroGateWayMsgs(),
                setattr(self.FSWModels[spacecraftIndex].trackingError, 'sigma_R0R', [0.0, 0.0, 0.0]),
                self.FSWModels[spacecraftIndex].updateHillFrameReferenceFromFormationTargets(),
                self.enableTask("hillFrameRelativeControlTask" + str(spacecraftIndex)),
                self.enableTask("hillPointTask" + str(spacecraftIndex)),
                self.enableTask("trackingErrorTask" + str(spacecraftIndex)),
                self.enableTask("mrpFeedbackRWsTask" + str(spacecraftIndex)),
                self.enableTask("stateMachineTask" + str(spacecraftIndex)),
                setattr(self.FSWModels[spacecraftIndex], 'stationKeeping', 'ON'),
                self.setAllButCurrentEventActivity(
                    f"initiatePidStationKeeping_{spacecraftIndex}", True, useIndex=True
                ),
            ),
        )

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        """
        Defines the inertial pointing guidance module.
        """
        self.inertial3DPoint.sigma_R0N = [0.1, 0.2, -0.3]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DPoint.attRefOutMsg, self.attRefMsg)

    def SetchargeBatteryGuidance(self):
        """
        Defines the solar-cells to sun pointing guidance module.
        """
        self.solCharging.pHat_B = self.simBase.DynModels[self.spacecraftIndex].solarPanelAxis
        self.solCharging.scAttInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.solCharging.scTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.solCharging.celBodyInMsg.subscribeTo(
            self.simBase.EnvModel.ephemObject.ephemOutMsgs[self.simBase.EnvModel.gravBodyList.index('sun')])
        messaging.AttRefMsg_C_addAuthor(self.solCharging.attRefOutMsg, self.attRefMsg)

    def SetNadirPointGuidance(self):
        """
        Defines the Earth location pointing guidance module (Pointing to NADIR).
        """
        self.nadirPoint.pHat_B = [0, 0, 1] # GNSS-R sensors are on the 'flat underside' of the satellite
        self.nadirPoint.scAttInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.nadirPoint.scTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.nadirPoint.celBodyInMsg.subscribeTo(
            self.simBase.EnvModel.ephemObject.ephemOutMsgs[self.simBase.EnvModel.gravBodyList.index('earth')])
        messaging.AttRefMsg_C_addAuthor(self.nadirPoint.attRefOutMsg, self.attRefMsg)

    def SetLocationPointGuidance(self): # TODO change this to take a location as input
        """
        Defines the Earth location pointing guidance module.
        """
        self.locPoint.pHat_B = [0, 0, -1] # TODO check if this is correct
        self.locPoint.scAttInMsg.subscribeTo(self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.locPoint.scTransInMsg.subscribeTo(self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.locPoint.locationInMsg.subscribeTo(self.simBase.EnvModel.groundStationBar.currentGroundStateOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.locPoint.attRefOutMsg, self.attRefMsg)

    def SetSpacecraftOrbitReconfig(self): #TODO check if this is correct (spacecraft should reconfigure formation in position and attitude)
        """
        Defines the station keeping module.
        """
        # Provide a zeroed default so chiefTransInMsg is never unlinked at construction time.
        # A zero r/v vector will cause rv2elem to produce invalid orbital elements, making any
        # misconfiguration fail loudly rather than silently using the wrong spacecraft state.
        # setFormationTargets() / set_barycenter_chief_source() overwrites this with the real
        # chief source before InitializeSimulation() is called.
        self.chiefTransDefaultMsg = messaging.NavTransMsg().write(messaging.NavTransMsgPayload())
        self.spacecraftReconfig.chiefTransInMsg.subscribeTo(self.chiefTransDefaultMsg)
        self.spacecraftReconfig.deputyTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.spacecraftReconfig.attRefInMsg.subscribeTo(self.attRefMsg)                  #TODO why is this subscribed to attRefMsg? and not to the attOutMsg of the nav module?
        self.spacecraftReconfig.thrustConfigInMsg.subscribeTo(self.fswThrusterConfigMsg) #TODO same here, why subscribe to thrusterConfigMsg from FSW and not to the thruster msg from the dynamics?
        self.spacecraftReconfig.vehicleConfigInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.spacecraftReconfig.mu = self.simBase.EnvModel.mu  # [m^3/s^2]
        self.spacecraftReconfig.attControlTime = 400  # [s]
        messaging.AttRefMsg_C_addAuthor(self.spacecraftReconfig.attRefOutMsg, self.attRefMsg)

    def SetAttitudeTrackingError(self):
        """
        Defines the module that converts a reference message into a guidance message.
        """
        # Default: no fixed body-to-reference offset unless a mode overrides it.
        self.trackingError.sigma_R0R = [0.0, 0.0, 0.0]
        self.trackingError.attNavInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)

    def SetMRPFeedbackRWA(self):
        """
        Defines the control properties.
        """
        self.decayTime = 50
        self.xi = 0.9
        self.mrpFeedbackRWs.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWs.P = 2 * np.max(self.simBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWs.K = (self.mrpFeedbackRWs.P / self.xi) * \
                                    (self.mrpFeedbackRWs.P / self.xi) / np.max(
            self.simBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRWConfigMsg(self):
        """
        Imports the RWs configuration information.
        """
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        # the same msg is used here for both spacecraft
        self.fswRwConfigMsg = self.simBase.DynModels[self.spacecraftIndex].rwFactory.getConfigMessage()

    def SetThrustersConfigMsg(self):
        """
        Imports the thrusters configuration information.
        """
        self.fswThrusterConfigMsg = self.simBase.DynModels[self.spacecraftIndex].thrusterFactory.getConfigMessage()
#        fswSetupThrusters.clearSetup()
#        for key, th in self.simBase.DynModels[self.spacecraftIndex].thrusterFactory.thrusterList.items():
#            loc_B_tmp = list(itertools.chain.from_iterable(th.thrLoc_B))
#            dir_B_tmp = list(itertools.chain.from_iterable(th.thrDir_B))
#            fswSetupThrusters.create(loc_B_tmp, dir_B_tmp, th.MaxThrust)
#        self.fswThrusterConfigMsg = fswSetupThrusters.writeConfigMessage()

    def SetMtbConfigMsg(self):
        """
        Imports the MTB configuration information.
        """
        # Configure MTB exactly as it is in the Dynamics
        self.mtbParamsInMsg = self.simBase.DynModels[self.spacecraftIndex].mtbParamsInMsg

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

    def setupTamComm(self):
        """
        Defines the TAM communication module.
        """
        self.tamComm.dcm_BS = [1., 0., 0.,
                               0., 1., 0.,
                               0., 0., 1.]
        self.tamComm.tamInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].tam.tamDataOutMsg)
#        tamCommLog = self.tamComm.tamOutMsg.recorder(samplingTime) # TODO add logging if needed; Put to the right place

    def setupMtbMomentumManagement(self):
        """
        Defines the MTB momentum management module.
        """
        self.mtbMomentumManagement.wheelSpeedBiases = [0.0, 0.0, 0.0, 0.0]  #TODO is this feasible? confirm
        self.mtbMomentumManagement.cGain = 0.003                            #TODO understand what this is

        self.mtbMomentumManagement.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mtbMomentumManagement.mtbParamsInMsg.subscribeTo(self.mtbParamsInMsg)
        self.mtbMomentumManagement.tamSensorBodyInMsg.subscribeTo(self.tamComm.tamOutMsg)
        self.mtbMomentumManagement.rwSpeedsInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mtbMomentumManagement.rwMotorTorqueInMsg.subscribeTo(self.rwMotorTorque.rwMotorTorqueOutMsg)

        self.simBase.DynModels[self.spacecraftIndex].mtbEff.mtbParamsInMsg.subscribeTo(self.mtbParamsInMsg)
        self.simBase.DynModels[self.spacecraftIndex].mtbEff.mtbCmdInMsg.subscribeTo(self.mtbMomentumManagement.mtbCmdOutMsg)
#        mtbDipoleCmdsLog = self.mtbMomentumManagement.mtbCmdOutMsg.recorder(samplingTime)

    def setupDataTransferSvalbard(self):
        """
        Defines the data transfer module.
        """
        self.svalbardStationPoint.pHat_B = [1, 0, 0]  # Point antenna axis toward ground station (adjust based on your antenna mounting)
        self.svalbardStationPoint.scAttInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.svalbardStationPoint.scTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        # Use ground station location message
        self.svalbardStationPoint.locationInMsg.subscribeTo(
            self.simBase.EnvModel.groundStationSval.currentGroundStateOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.svalbardStationPoint.attRefOutMsg, self.attRefMsg)

    def setSpacecraftPointing(self, chiefIndex=None, useBarycenter=False):
        self.spacecraftPointing.deputyPositionInMsg.subscribeTo(self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)

        # Chief depends on formation type
        if useBarycenter:
            # Point at barycenter
            self.spacecraftPointing.chiefPositionInMsg.subscribeTo(
                self.simBase.relativeNavigationModule.transOutMsg
            )
        elif chiefIndex is not None:
            # Point at another spacecraft
            self.spacecraftPointing.chiefPositionInMsg.subscribeTo(
                self.simBase.DynModels[chiefIndex].simpleNavObject.transOutMsg
            )
        # Define body axis pointing at the 'chief'
        self.spacecraftPointing.alignmentVector_B = [1.0, 2.0, 3.0]
        messaging.AttRefMsg_C_addAuthor(self.spacecraftPointing.attReferenceOutMsg, self.attRefMsg)

    def SetHillPointGuidance(self):
        """
        Defines the Hill-frame pointing guidance module.
        """
        self.attGuidanceHillPoint.transNavInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg
        )
        self.attGuidanceHillPoint.celBodyInMsg.subscribeTo(
            self.simBase.EnvModel.ephemObject.ephemOutMsgs[
                self.simBase.EnvModel.gravBodyList.index('earth')
            ]
        )
        messaging.AttRefMsg_C_addAuthor(self.attGuidanceHillPoint.attRefOutMsg, self.attRefMsg)

    def SetHillFrameRelativeControl(self):
        """
        Defines the Hill-frame relative PD station-keeping controller.

        This controller acts as the **fine maintenance loop**, applied after
        ``spacecraftReconfig`` has completed a large formation reconfiguration.
        Its role is to counteract slow differential perturbations (J2, drag) that
        would otherwise cause the achieved formation to drift over multiple orbits.
        Required corrections are typically in the µN–mN range.

        The output ``forceOutMsg`` (``CmdForceInertialMsgPayload``) is fed into the
        ``extForceHillPd`` ``ExtForceTorque`` effector, which applies the commanded
        3-DOF inertial force directly to the spacecraft dynamics.  This models an
        **idealized multi-directional micro-thruster set** (not the physical EPSS C2
        thruster, which is single-axis).

        See ``SetExtForceOEFeedback`` in ``BSK_GnssrSatDynamics.py`` for the
        FUTURE IMPROVEMENT note on replacing this with a physically realistic
        single-thruster cascade.
        """
        self.hillFrameRelativeControl.setMu(self.simBase.get_EnvModel().mu)  # [m^3/s^2]
        self.hillFrameRelativeControl.setK([
            2.0e-6, 0.0, 0.0,
            0.0, 2.0e-6, 0.0,
            0.0, 0.0, 2.0e-6,
        ])  # [1/s^2]
        self.hillFrameRelativeControl.setP([
            2.0e-3, 0.0, 0.0,
            0.0, 2.0e-3, 0.0,
            0.0, 0.0, 2.0e-3,
        ])  # [1/s]

        # Default reference is overwritten at runtime by updateHillFrameReferenceFromFormationTargets.
        self.hillFrameRelativeControl.setReferencePosition([0.0, 0.0, 0.0])  # [m]
        self.hillFrameRelativeControl.setReferenceVelocity([0.0, 0.0, 0.0])  # [m/s]

        # Provide a zeroed default so chiefTransInMsg is never unlinked at Reset() time.
        # setFormationTargets() / set_barycenter_chief_source() overwrites this with the
        # correct chief source before InitializeSimulation() is called.
        self.hillFrameRelativeControl.chiefTransInMsg.subscribeTo(self.chiefTransDefaultMsg)

        self.hillFrameRelativeControl.deputyTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg
        )
        self.hillFrameRelativeControl.deputyVehicleConfigInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg
        )

        # Wire to the dedicated Hill-PD effector, keeping the meanOEFeedback
        # channel (extForceOEFeedback) independent.
        self.simBase.DynModels[self.spacecraftIndex].extForceHillPd.cmdForceInertialInMsg.subscribeTo(
            self.hillFrameRelativeControl.forceOutMsg
        )

    def setupScanningInstrumentControler(self):
        """
        Defines the simple instrument controller module.
        """
        self.scanningInstrumentController.useRateTolerance = 1
        self.scanningInstrumentController.rateErrTolerance = 0.01
        self.scanningInstrumentController.attErrTolerance = 0.1
        self.scanningInstrumentController.controllerStatus = 1        # Enable the controller (default is 0 = disabled)
        self.scanningInstrumentController.attGuidInMsg.subscribeTo(self.attGuidMsg)
        self.scanningInstrumentController.accessInMsg.subscribeTo(
            self.simBase.EnvModel.groundEarthCenter.accessOutMsgs[self.spacecraftIndex])

        # Connect controller output to simpleInstrument
        self.simBase.DynModels[self.spacecraftIndex].instrument.nodeStatusInMsg.subscribeTo(
            self.scanningInstrumentController.deviceCmdOutMsg)

    def setmeanOEFeedback(self):
        """
        Defines the mean orbital elements feedback module.
        """
        # Set gains
        self.meanOEFeedback.K = [1e7, 0, 0, 0, 0, 0,
                                 0, 1e7, 0, 0, 0, 0,
                                 0, 0, 1e7, 0, 0, 0,
                                 0, 0, 0, 1e7, 0, 0,
                                 0, 0, 0, 0, 1e7, 0,
                                 0, 0, 0, 0, 0, 1e7]
        self.meanOEFeedback.targetDiffOeMean = [0, 0, 0, 0, 0, 0]
        self.meanOEFeedback.oeType = 1 # 0 for classic elements, 1 for equinoctial elements
        self.meanOEFeedback.mu = self.simBase.get_EnvModel().mu
        self.meanOEFeedback.req = self.simBase.get_EnvModel().planetRadius
        self.meanOEFeedback.J2 = astroConstants.J2_EARTH

        self.meanOEFeedback.deputyTransInMsg.subscribeTo(
            self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)

        # Connect force output to extForceTorque dynamics effector
        self.simBase.DynModels[self.spacecraftIndex].extForceOEFeedback.cmdForceInertialInMsg.subscribeTo(
            self.meanOEFeedback.forceOutMsg)

        # Blank chief message (overwritten in scenario configure() for CPO)
#        chiefData = messaging.NavTransMsgPayload()                   #TODO Why is this needed?
#        chiefMsg = messaging.NavTransMsg().write(chiefData)          #TODO what is written to this message (blank) and is this correct?
#        self.meanOEFeedback.chiefTransInMsg.subscribeTo(chiefMsg)

    # Global call to initialize every module
    def InitAllFSWObjects(self):
        """
        Initializes all FSW objects.
        """
        # Get configuration messages from the Dynamics modules
        self.SetRWConfigMsg()
        self.SetThrustersConfigMsg()
        self.SetMtbConfigMsg()
        # Initialize all modules
        self.SetInertial3DPointGuidance()
        self.SetchargeBatteryGuidance()
        self.SetNadirPointGuidance()
        self.SetLocationPointGuidance()
        self.SetAttitudeTrackingError()
        self.SetMRPFeedbackRWA()
        self.SetRWMotorTorque()
        self.SetSpacecraftOrbitReconfig()
        self.setupTamComm()
        self.setupMtbMomentumManagement()
        self.setupScanningInstrumentControler()
        self.setupDataTransferSvalbard()
        self.setSpacecraftPointing()
        self.SetHillPointGuidance()
        self.setmeanOEFeedback()
        self.SetHillFrameRelativeControl()

    def switchToLiveBarycenterChief(self, verbose=False):
        """
        Switch from frozen to live barycenter chief once after reconfiguration.
        """
        if self.spacecraftIndex != 0:
            return
        if self.barycenterChiefSwitched:
            return
        if not getattr(self.simBase, "useBarycenter", False):
            return
        if not hasattr(self.simBase, "set_barycenter_chief_source"):
            return

        self.simBase.set_barycenter_chief_source("live", verbose=verbose)
        if hasattr(self.simBase, "chiefSwitchTimeNanos"):
            self.simBase.chiefSwitchTimeNanos = self.simBase.TotalSim.CurrentNanos
        self.barycenterChiefSwitched = True

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _formation_target_to_hill_reference(self, chief_oe, target_oed):
        """
        Convert classical orbital-element differences into instantaneous Hill-frame
        position/velocity references near a chief orbit.
        """
        delta_a = float(target_oed[0])
        delta_e = float(target_oed[1])
        delta_i = float(target_oed[2])
        delta_omega = float(target_oed[4])
        delta_M = float(target_oed[5])

        chief_inc = float(chief_oe.i)
        chief_omega = float(chief_oe.omega)
        chief_true_anomaly = float(chief_oe.f)
        chief_e = float(chief_oe.e)
        chief_a = float(chief_oe.a)

        # Recover delta_Omega from small-angle mapping delta_i_y = sin(i)*delta_Omega.
        sin_inc = np.sin(chief_inc)
        if np.abs(sin_inc) < 1.0e-8:
            delta_Omega = 0.0
        else:
            delta_Omega = float(target_oed[3])

        # Convert classical delta-e and delta-omega to relative eccentricity vector.
        chief_ex = chief_e * np.cos(chief_omega)
        chief_ey = chief_e * np.sin(chief_omega)
        deputy_e = chief_e + delta_e
        deputy_omega = chief_omega + delta_omega
        deputy_ex = deputy_e * np.cos(deputy_omega)
        deputy_ey = deputy_e * np.sin(deputy_omega)
        delta_ex = deputy_ex - chief_ex
        delta_ey = deputy_ey - chief_ey

        # Relative inclination vector.
        delta_ix = delta_i
        delta_iy = sin_inc * delta_Omega

        # Mean longitude difference.
        delta_lambda = delta_M + delta_omega + np.cos(chief_inc) * delta_Omega

        arg_latitude = chief_omega + chief_true_anomaly
        mean_motion = np.sqrt(self.simBase.get_EnvModel().mu / (chief_a ** 3))

        radial_ref = chief_a * (delta_a - delta_ex * np.cos(arg_latitude) - delta_ey * np.sin(arg_latitude))
        along_ref = chief_a * (
            delta_lambda
            + 2.0 * (delta_ex * np.sin(arg_latitude) - delta_ey * np.cos(arg_latitude))
        )
        cross_ref = chief_a * (delta_ix * np.sin(arg_latitude) - delta_iy * np.cos(arg_latitude))

        radial_rate_ref = chief_a * mean_motion * (delta_ex * np.sin(arg_latitude) - delta_ey * np.cos(arg_latitude))
        along_rate_ref = chief_a * mean_motion * (
            -1.5 * delta_a + 2.0 * (delta_ex * np.cos(arg_latitude) + delta_ey * np.sin(arg_latitude))
        )
        cross_rate_ref = chief_a * mean_motion * (delta_ix * np.cos(arg_latitude) + delta_iy * np.sin(arg_latitude))

        return [radial_ref, along_ref, cross_ref], [radial_rate_ref, along_rate_ref, cross_rate_ref]

    def updateHillFrameReferenceFromFormationTargets(self):
        """
        Update Hill-frame reference from the current formation OED target.
        """
        chief_msg = self.hillFrameRelativeControl.chiefTransInMsg()
        if np.linalg.norm(chief_msg.r_BN_N) < 1.0:
            return False

        chief_oe = orbitalMotion.rv2elem(
            self.simBase.get_EnvModel().mu,
            np.array(chief_msg.r_BN_N),
            np.array(chief_msg.v_BN_N),
        )
        target_oed = list(self.spacecraftReconfig.targetClassicOED)
        reference_position, reference_velocity = self._formation_target_to_hill_reference(chief_oe, target_oed)
        self.hillFrameRelativeControl.setReferencePosition(reference_position)
        self.hillFrameRelativeControl.setReferenceVelocity(reference_velocity)
        return True

    def formationErrorWithinTolerance(self):
        """
        Check if deputy orbital-element differences match the current target.
        """
        chief_msg = self.spacecraftReconfig.chiefTransInMsg()
        deputy_msg = self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg.read()

        if np.linalg.norm(chief_msg.r_BN_N) < 1.0 or np.linalg.norm(deputy_msg.r_BN_N) < 1.0:
            self.reconfigStableTimeNanos = 0
            return False

        chief_oe = orbitalMotion.rv2elem(
            self.simBase.get_EnvModel().mu,
            np.array(chief_msg.r_BN_N),
            np.array(chief_msg.v_BN_N),
        )
        deputy_oe = orbitalMotion.rv2elem(
            self.simBase.get_EnvModel().mu,
            np.array(deputy_msg.r_BN_N),
            np.array(deputy_msg.v_BN_N),
        )

        current_oed = [
            (deputy_oe.a - chief_oe.a) / chief_oe.a,
            deputy_oe.e - chief_oe.e,
            deputy_oe.i - chief_oe.i,
            self._wrap_to_pi(deputy_oe.Omega - chief_oe.Omega),
            self._wrap_to_pi(deputy_oe.omega - chief_oe.omega),
            self._wrap_to_pi(
                orbitalMotion.E2M(orbitalMotion.f2E(deputy_oe.f, deputy_oe.e), deputy_oe.e)
                - orbitalMotion.E2M(orbitalMotion.f2E(chief_oe.f, chief_oe.e), chief_oe.e)
            ),
        ]

        target_oed = list(self.spacecraftReconfig.targetClassicOED)
        tolerances = [
            5.0e-6,
            2.0e-5,
            2.0e-5,
            2.0e-4,
            2.0e-4,
            2.0e-4,
        ]

        in_tolerance = all(np.abs(curr - target) <= tol for curr, target, tol in zip(current_oed, target_oed, tolerances))

        if in_tolerance:
            self.reconfigStableTimeNanos += self.processTasksTimeStep
        else:
            self.reconfigStableTimeNanos = 0

        return self.reconfigStableTimeNanos >= mc.min2nano(30.0)

    def setModeRequest(self, modeRequest=None, verbose=None, currentSimNanos=None):
        """State machine to set the modeRequest variable based on time and conditions"""
        previousMode = self.modeRequest
        if currentSimNanos is not None:
            self.flightTime = int(currentSimNanos)
        else:
            currentNanos = int(self.simBase.TotalSim.CurrentNanos)
            if currentNanos == np.iinfo(np.uint64).max:
                self.flightTime = 0
                print("WARNING: overflow detected in sim time, resetting flight time to 0")
            else:
                self.flightTime = currentNanos

        # Update verboseMode if explicitly set
        if verbose is not None:
            self.verboseMode = verbose

        # ===== FORCED MODE CHANGE =====
        if modeRequest is not None and modeRequest != 'autonomous':
            self.modeRequest = modeRequest
            self.timeInMode = 0
            self.reconfigStableTimeNanos = 0
            if self.verboseMode and previousMode != self.modeRequest:
                print(f" SC-{self.spacecraftIndex}: {previousMode} -> {self.modeRequest} at {format_sim_time(self.flightTime)} [MANUAL]")
            return

        # ===== AUTOMATIC MODE CHANGE =====
        if modeRequest is None or modeRequest == 'autonomous':
            # Example automatic mode changes based on time
            if (self.modeRequest == "standby" and
                self.flightTime >= mc.min2nano(5.0)):
                self.modeRequest = "chargeBattery"
                self.stationKeeping = "OFF"

            # CHARGE_BATTERY -> STATION_KEEPING
            elif (self.modeRequest == "chargeBattery" and
                  self.fleetBatteryLevelAboveThreshold(threshold=0.95)):
                if self.reconfFormation == True:
                    self.modeRequest = "initiateStationKeeping"
                    self.stationKeeping = "ON"
                    self.reconfFormation = False
                else:
                    self.modeRequest = "pidStationKeeping"
                    self.stationKeeping = "ON"

            # INITIATE_STATION_KEEPING (Reconfig) -> GNSSR_SENSING
            elif (self.modeRequest == "initiateStationKeeping" and
                 not self.batteryLevelAboveThreshold(threshold=0.40)):
                    self.modeRequest = "chargeBattery"
                    self.stationKeeping = "OFF"
                    self.reconfigStableTimeNanos = 0

            # INITIATE_STATION_KEEPING (Reconfig) -> PID_STATION_KEEPING
            elif self.modeRequest == "initiateStationKeeping":
                self.updateHillFrameReferenceFromFormationTargets()
                if self.formationErrorWithinTolerance():
                    self.modeRequest = "pidStationKeeping"
                    self.stationKeeping = "ON"
                    self.reconfigFormation = False

            # PID_STATION_KEEPING -> GNSSR_SENSING
            elif self.modeRequest == "pidStationKeeping":
                self.updateHillFrameReferenceFromFormationTargets()
                if (self.reachedLatitude(latLimit=55) and
                    self.timeInMode >= mc.hour2nano(24.0)):
                    self.modeRequest = "startGnssrSensing"
                    self.stationKeeping = "OFF"

            # GNSSR_SENSING -> DATA_TRANSFER
            elif self.modeRequest == "startGnssrSensing":
                if (self.dataBufferLevelAboveThreshold(threshold=0.8) and
                    self.inCommunicationRangeSval()):
                    self.modeRequest = "dataTransfer"
                    self.stationKeeping = "OFF"

            # GNSSR_SENSING or DATA_TRANSFER -> CHARGE_BATTERY
            elif (self.modeRequest in ["startGnssrSensing", "dataTransfer", "pidStationKeeping"] and
                    not self.batteryLevelAboveThreshold(threshold=0.5)):
                    self.modeRequest = "chargeBattery"
                    self.stationKeeping = "OFF"

            # DATA_TRANSFER -> PID_STATIONKEEPING at end of comm window
            elif (self.modeRequest == "dataTransfer" and
                    not self.inCommunicationRangeSval()):
                self.modeRequest = "pidStationKeeping"
                self.stationKeeping = "ON"

            # PID_STATION_KEEPING -> GNSSR_SENSING
            elif (self.modeRequest == "pidStationKeeping" and
                self.reachedLatitude(latLimit=55) and
                self.timeInMode >= mc.hour2nano(24.0)):
                self.modeRequest = "startGnssrSensing"
                self.stationKeeping = "OFF"

        if self.modeRequest != previousMode:
            self.timeInMode = 0
            if self.verboseMode:
                print(f" SC-{self.spacecraftIndex}: {previousMode} -> {self.modeRequest} at {format_sim_time(self.flightTime)} [AUTO]")
        # Update time in current mode
        elif self.modeRequest == previousMode:
            self.timeInMode += self.processTasksTimeStep
        else:
            BSK_ERROR("FSW mode switching error.")

    def setupGatewayMsgs(self):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()

        # Create antenna state message BEFORE zeroGateWayMsgs (default OFF)
        antennaStateData = messaging.AntennaStateMsgPayload()
        antennaStateData.antennaState = simpleAntenna.ANTENNA_OFF
        self.antennaStateMsg = messaging.AntennaStateMsg().write(antennaStateData)

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        self.simBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(
            self.rwMotorTorque.rwMotorTorqueOutMsg)
        self.simBase.DynModels[self.spacecraftIndex].thrusterStateEffector.cmdsInMsg.subscribeTo(
            self.spacecraftReconfig.onTimeOutMsg) #                             TODO what is the "on Time Out Msg"?
        self.simBase.DynModels[self.spacecraftIndex].mtbEff.mtbCmdInMsg.subscribeTo(
            self.mtbMomentumManagement.mtbCmdOutMsg)
        # Connect antenna state message to dynamics
        self.simBase.DynModels[self.spacecraftIndex].simpleAntenna.antennaSetStateInMsg.subscribeTo(
            self.antennaStateMsg)

    def zeroGateWayMsgs(self):
        """Zero all FSW gateway message payloads"""
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())

        # Zero all actuator commands
        self.rwMotorTorque.rwMotorTorqueOutMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.spacecraftReconfig.onTimeOutMsg.write(messaging.THRArrayOnTimeCmdMsgPayload())
        self.mtbMomentumManagement.mtbCmdOutMsg.write(messaging.MTBCmdMsgPayload())
        # Zero the Hill-PD force output so stale commands are not applied
        # when switching away from pidStationKeeping mode.
        self.hillFrameRelativeControl.forceOutMsg.write(messaging.CmdForceInertialMsgPayload())
        # Turn antenna OFF
        self.antennaStateMsg.write(
            messaging.AntennaStateMsgPayload(antennaState=simpleAntenna.ANTENNA_OFF)
        )

    #######################################################
    #####             Helper functions                 #####
    ########################################################
    def illuminatedBySun(self):
        """
        Check if the spacecraft is illuminated by the sun.

        Returns:
            bool: True if illuminated (not in eclipse), False if in shadow.
        """
        eclipseMsg = self.simBase.EnvModel.eclipseObject.eclipseOutMsgs[self.spacecraftIndex].read()
        return eclipseMsg.shadowFactor > 0
    #    return eclipseMsg.illuminationFactor > 0 # TODO update once possible in Basilisk

    def batteryLevelAboveThreshold(self, threshold):
        """
        Check if the spacecraft battery level is above a certain threshold.
        """
        batteryMsg = self.simBase.DynModels[self.spacecraftIndex].powerMonitor.batPowerOutMsg.read()

        # Handle case where message hasn't been written yet
        if batteryMsg.storageCapacity == 0:
            # Fall back to initial values before simulation has run
            chargeFraction = self.simBase.DynModels[self.spacecraftIndex].powerMonitor.storedCharge_Init / self.simBase.DynModels[self.spacecraftIndex].powerMonitor.storageCapacity
        else:
            chargeFraction = batteryMsg.storageLevel / batteryMsg.storageCapacity
        return chargeFraction >= threshold

    def inCommunicationRangeSval(self):
        """
        Check if the spacecraft is in communication window with the ground station.
        """
        accessMsg = self.simBase.EnvModel.groundStationSval.accessOutMsgs[self.spacecraftIndex].read()
        return accessMsg.hasAccess > 0

    def fleetBatteryLevelAboveThreshold(self, threshold):
        """
        Check if all spacecraft battery levels are above a threshold.
        """
        if hasattr(self.simBase, "FSWModels"):
            return all(model.batteryLevelAboveThreshold(threshold) for model in self.simBase.FSWModels)
        return self.batteryLevelAboveThreshold(threshold)

    def reachedLatitude(self, latLimit):
        """
        Check if the spacecraft has reached a certain latitude.
        """
        navMsg = self.simBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg.read()
        r_BN_N = navMsg.r_BN_N
        # Convert position to spherical coordinates to get latitude
        r = np.linalg.norm(r_BN_N)
        lat = np.arcsin(r_BN_N[2] / r) * mc.R2D  # in degrees
        return abs(lat) >= latLimit

    def dataBufferLevelBelowThreshold(self, threshold):
        """Check if the data buffer level is below a certain threshold."""
        dataMsg = self.simBase.DynModels[self.spacecraftIndex].dataMonitor.storageUnitDataOutMsg.read()
        if dataMsg.storageCapacity == 0:
            return True
        fillFraction = dataMsg.storageLevel / dataMsg.storageCapacity
        return fillFraction <= threshold

    def dataBufferLevelAboveThreshold(self, threshold):
        """Check if the data buffer level is above a certain threshold."""
        dataMsg = self.simBase.DynModels[self.spacecraftIndex].dataMonitor.storageUnitDataOutMsg.read()
        if dataMsg.storageCapacity == 0:
            return False
        fillFraction = dataMsg.storageLevel / dataMsg.storageCapacity
        return fillFraction >= threshold

    def getFlightTime(self):
        """Get current simulation time in nanoseconds."""
        return self.simBase.TotalSim.CurrentNanos
