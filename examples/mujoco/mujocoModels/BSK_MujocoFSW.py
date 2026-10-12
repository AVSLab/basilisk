#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (
    hingedJointArrayMotor,
    jointMotionCompensator,
    thrJointCompensation,
    inertial3D,
    attTrackingError,
    locationPointing,
    mrpFeedback,
    inertialCartFeedback,
    jointThrAllocation,
    inertialCartFeedback,
    thrFiringRound,
    arcLineHoldRef,
)
from Basilisk.simulation import (
    thrOnTimeToForce,
    simpleNav,
    stateMerge,
    jointArrayRefProfiler,
)


class BSKMujocoFSWModels:
    """Defines the bskSim FSW class"""
    def __init__(self, SimBase, fswRate, config):
        self.controlObjective = getattr(config, "controlObjective", None)
        if self.controlObjective not in ("regulation", "tracking"):
            SimBase.bskLogger.error(
                "BSKMujocoFSWModels supports only regulation and tracking control objectives."
            )
        self.isTracking = self.controlObjective == "tracking"
        self.config = config

        # define empty class variables
        self.vcMsg = None
        self.thrArmConfigMsg = None
        self.transRefMsg = None
        self.scTargetMsg = None
        self.jointTorqueGateways = []
        self.thrForceGateways = []
        self.bodyTorqueGateway = []
        self.thrFiringTimeGateway = None

        # Define process name and task time-steps
        self.processName = SimBase.FSWProcessName
        self.innerLoopTimeStep = mc.sec2nano(fswRate)
        self.outerLoopTimeStep = mc.sec2nano(config.outerLoopPeriod)

        self.numJoints = config.numJoints
        self.numThrusters = config.numThrusters
        self.vehicleMass = getattr(config, "vehicleMass", None)
        if self.vehicleMass is None:
            self.vehicleMass = config.scMass
        self.hubMass = config.hubMass
        self.hubHalfLengths = config.hubHalfLengths
        self.thrForces = config.thrForces
        self.maxJointTorques = config.maxJointTorques
        self.controlWeight = config.controlWeight
        self.thrustWeight = config.thrustWeight
        self.pTrans = config.pTrans
        self.kTrans = config.kTrans
        self.pRot = config.pRot
        self.kRot = config.kRot
        self.kiRot = config.kiRot
        self.kTheta = config.kTheta
        self.pTheta = config.pTheta
        self.hubName = config.hubName
        self.jointRefs = config.jointRefs
        self.jointActNames = config.jointActNames
        self.thrActNames = config.thrActNames
        self.bodyActNames = config.bodyActNames
        self.useJointProfiler = getattr(config, "useJointProfiler", False)
        self.useJointMotionCompensation = getattr(
            config, "useJointMotionCompensation", True
        )
        self.useThetaAllocPenalty = getattr(config, "useThetaAllocPenalty", False)
        self.thetaWeight = getattr(config, "thetaWeight", None)
        self.maxPreTorques = getattr(config, "maxPreTorques", None)
        if self.useThetaAllocPenalty and self.thetaWeight is None:
            SimBase.bskLogger.error(
                "A theta allocation penalty requires a configured thetaWeight."
            )
        if self.useJointProfiler and config.jointProfileType is None:
            SimBase.bskLogger.error(
                "A joint motion profile requires a configured jointProfileType."
            )

        # Define initial event check values
        self.lastDesAngles = [0.0]*self.numJoints
        self.lastDesAngleRates = [0.0]*self.numJoints
        self.lastDesCmdWriteTime = -1
        self.pendingOnTimes = [0.0]*self.numThrusters  # [s]
        self.pendingFiring = False
        self.firingCancellationPending = False
        self.jointMotionPhaseActive = False
        self.thrustJustActivated = False

        # Instantiate FSW modules as objects
        self.inertial3D = None
        self.transRef = None
        self.attRef = None
        self.stateMerge = stateMerge.StateMerge()
        self.navConverter = simpleNav.SimpleNav()
        self.attError = None
        self.mrpFeedback = mrpFeedback.mrpFeedback()
        self.inertialCartFeedback = inertialCartFeedback.InertialCartFeedback()
        self.jointThrAllocation = jointThrAllocation.JointThrAllocation()
        self.thrFiringRound = thrFiringRound.ThrFiringRound()
        self.hingedJointArrayMotor = hingedJointArrayMotor.HingedJointArrayMotor()
        self.jointMotionCompensator = None
        if self.useJointMotionCompensation:
            self.jointMotionCompensator = jointMotionCompensator.JointMotionCompensator()
        self.thrOnTimeToForce = thrOnTimeToForce.ThrOnTimeToForce()
        self.thrJointCompensation = thrJointCompensation.ThrJointCompensation()
        self.jointProfiler = None
        if self.useJointProfiler:
            self.jointProfiler = jointArrayRefProfiler.JointArrayRefProfiler()
        if self.isTracking:
            self.transRef = arcLineHoldRef.ArcLineHoldRef()
            self.attRef = locationPointing.locationPointing()
        else:
            self.inertial3D = inertial3D.inertial3D()
            self.attError = attTrackingError.attTrackingError()

        # Create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        self.outerLoopTaskName = "outerLoopTask"
        self.jointMotionTaskName = "jointMotionTask"
        self.thrFiringTaskName = "thrFiringTask"
        self.recorderTaskName = "recorderTask"
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.outerLoopTaskName, self.outerLoopTimeStep), 100)
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.jointMotionTaskName, self.innerLoopTimeStep), 99)
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.thrFiringTaskName, self.innerLoopTimeStep), 98)
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.recorderTaskName, self.innerLoopTimeStep), 97)

        # Assign initialized modules to tasks
        if self.isTracking:
            SimBase.AddModelToTask(self.outerLoopTaskName, self.stateMerge, 50)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.navConverter, 49)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.attRef, 48)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.mrpFeedback, 47)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.transRef, 46)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.inertialCartFeedback, 45)
        else:
            SimBase.AddModelToTask(self.outerLoopTaskName, self.inertial3D, 50)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.stateMerge, 49)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.navConverter, 48)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.attError, 47)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.mrpFeedback, 46)
            SimBase.AddModelToTask(self.outerLoopTaskName, self.inertialCartFeedback, 45)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.jointThrAllocation, 44)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.thrFiringRound, 43)

        if self.useJointProfiler:
            SimBase.AddModelToTask(self.jointMotionTaskName, self.jointProfiler, 51)
            SimBase.AddModelToTask(self.jointMotionTaskName, self.hingedJointArrayMotor, 50)
        else:
            SimBase.AddModelToTask(self.jointMotionTaskName, self.hingedJointArrayMotor, 50)
        if self.useJointMotionCompensation:
            SimBase.AddModelToTask(self.jointMotionTaskName, self.jointMotionCompensator, 49)

        SimBase.AddModelToTask(self.thrFiringTaskName, self.thrOnTimeToForce, 50)
        SimBase.AddModelToTask(self.thrFiringTaskName, self.thrJointCompensation, 49)

        # Create events to to be called for changing active controllers
        SimBase.fswProc.disableAllTasks()
        self.zeroGatewayMsgs()
        SimBase.enableTask(self.outerLoopTaskName)
        SimBase.enableTask(self.recorderTaskName)

        def conditionJointMotion(self):
            """Detect and cache one newly allocated control-period command."""
            desAngles = list(self.FSWModels.desAngleReader().states)
            desAngleRates = list(self.FSWModels.desAngleReader().stateDots)
            currentWriteTime = self.FSWModels.desAngleReader.timeWritten()
            if len(desAngles) != self.FSWModels.numJoints or len(desAngleRates) != self.FSWModels.numJoints:
                return False
            if currentWriteTime == self.FSWModels.lastDesCmdWriteTime:
                return False

            self.FSWModels.lastDesAngles = desAngles[:]
            self.FSWModels.lastDesAngleRates = desAngleRates[:]
            self.FSWModels.lastDesCmdWriteTime = currentWriteTime
            self.FSWModels.pendingOnTimes = list(
                self.FSWModels.thrTimeReader().OnTimeRequest
            )
            self.FSWModels.pendingFiring = any(
                onTime > 0.0 for onTime in self.FSWModels.pendingOnTimes
            )
            return True

        def actionJointMotion(self):
            """Start the next pulse sequence or finish joint motion for a zero pulse."""
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
            self.enableTask(self.FSWModels.recorderTaskName)

            if not self.FSWModels.pendingFiring:
                if self.FSWModels.jointMotionPhaseActive:
                    self.enableTask(self.FSWModels.jointMotionTaskName)
                    self.setEventActivity("jointMotion", True)
                    self.setEventActivity("thrFiring", True)
                    self.setEventActivity("coast", False)
                    return

                self.FSWModels.jointMotionPhaseActive = False
                self.FSWModels.startFiringCancellation(self.TotalSim.CurrentNanos)
                self.enableTask(self.FSWModels.thrFiringTaskName)
                self.setEventActivity("jointMotion", True)
                self.setEventActivity("thrFiring", False)
                self.setEventActivity("coast", True)
                return

            self.FSWModels.jointMotionPhaseActive = True
            self.enableTask(self.FSWModels.jointMotionTaskName)
            self.setEventActivity("jointMotion", True)
            self.setEventActivity("thrFiring", True)
            self.setEventActivity("coast", False)

        SimBase.createNewEvent(
            "jointMotion",
            self.innerLoopTimeStep,
            True,
            conditionFunction=conditionJointMotion,
            actionFunction=actionJointMotion,
        )

        def conditionThrFiring(self):
            """Detect when the joints have reached their commanded states."""
            desStates = np.asarray(self.FSWModels.lastDesAngles, dtype=float)
            desStatesDot = np.asarray(self.FSWModels.lastDesAngleRates, dtype=float)
            curStates = np.zeros(self.FSWModels.numJoints)
            curStatesDot = np.zeros(self.FSWModels.numJoints)
            for i in range(self.FSWModels.numJoints):
                curStates[i] = self.FSWModels.anglesReaders[i]().state
                curStatesDot[i] = self.FSWModels.angleRatesReaders[i]().state
            angError = self.FSWModels.wrapAngle(desStates - curStates)
            stateError = np.max(np.abs(angError))
            rateError = np.max(np.abs(desStatesDot - curStatesDot))
            return (
                stateError < self.FSWModels.config.jointPositionTolerance
                and rateError < self.FSWModels.config.jointRateTolerance
            )

        def actionThrFiring(self):
            """Begin the requested thruster pulse after joint motion completes."""
            self.FSWModels.thrustJustActivated = True
            self.FSWModels.jointMotionPhaseActive = False
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
            self.enableTask(self.FSWModels.thrFiringTaskName)
            self.enableTask(self.FSWModels.recorderTaskName)
            CurrentSimNanos = self.TotalSim.CurrentNanos
            payload = messaging.THRArrayOnTimeCmdMsgPayload()
            payload.OnTimeRequest = self.FSWModels.pendingOnTimes[:]
            self.FSWModels.thrFiringTimeGateway.write(payload, CurrentSimNanos)
            self.setEventActivity("jointMotion", True)
            self.setEventActivity("thrFiring", False)
            self.setEventActivity("coast", True)

        SimBase.createNewEvent(
            "thrFiring",
            self.innerLoopTimeStep,
            False,
            conditionFunction=conditionThrFiring,
            actionFunction=actionThrFiring,
        )

        def conditionCoast(self):
            """Event condition to coast (no commands) between the time
            when the thrusters are done firing and the next outer loop update"""
            thrForces = np.zeros(self.FSWModels.numThrusters)
            for i, reader in enumerate(self.FSWModels.thrForceReaders):
                thrForces[i] = reader().input
            if self.FSWModels.firingCancellationPending:
                self.FSWModels.firingCancellationPending = False
                return False
            if self.FSWModels.thrustJustActivated:
                self.FSWModels.thrustJustActivated = False
                return False
            return np.max(np.abs(thrForces)) < self.FSWModels.config.thrusterZeroTolerance

        def actionCoast(self):
            """Event action to activate coasting (no commands) between the time
            when the thrusters are done firing and the next outer loop update"""
            self.FSWModels.jointMotionPhaseActive = False
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
            self.enableTask(self.FSWModels.recorderTaskName)
            self.setEventActivity("jointMotion", True)
            self.setEventActivity("thrFiring", False)
            self.setEventActivity("coast", False)

        SimBase.createNewEvent(
            "coast",
            self.innerLoopTimeStep,
            False,
            conditionFunction=conditionCoast,
            actionFunction=actionCoast,
        )

    # ------------------------------------------------------------------------------------------- #
    # These are event-control helper methods

    def startFiringCancellation(self, currentSimNanos):
        """Latch a zero on-time command to clear internal firing timers."""
        zeroOnTimePayload = messaging.THRArrayOnTimeCmdMsgPayload()
        self.thrFiringTimeGateway.write(zeroOnTimePayload, currentSimNanos)
        self.firingCancellationPending = True

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetVehicleConfiguration(self):
        """Set the spacecraft configuration information"""
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        vehicleConfigOut.massSC = self.vehicleMass
        # use the hub inertia for mrp feedback module since will not have instantaneous inertia otherwise
        halfLengthX, halfLengthY, halfLengthZ = self.hubHalfLengths
        vehicleConfigOut.ISCPntB_B = [
            self.hubMass / 3.0 * (halfLengthY ** 2 + halfLengthZ ** 2), 0.0, 0.0,
            0.0, self.hubMass / 3.0 * (halfLengthX ** 2 + halfLengthZ ** 2), 0.0,
            0.0, 0.0, self.hubMass / 3.0 * (halfLengthX ** 2 + halfLengthY ** 2),
        ]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetThrArmConfig(self):
        """Set the thruster arm configuration information"""
        thrArmConfigOut = messaging.THRArmConfigMsgPayload()

        thrArmConfigOut.bodyArmIdx.clear()
        thrArmConfigOut.bodyJointIdx.clear()
        thrArmConfigOut.bodyMass.clear()
        thrArmConfigOut.r_LcP_P.clear()
        thrArmConfigOut.thrArmIdx.clear()
        thrArmConfigOut.thrArmJointIdx.clear()
        thrArmConfigOut.armTreeIdx.clear()
        thrArmConfigOut.armJointCount.clear()
        thrArmConfigOut.r_CP_P.clear()
        thrArmConfigOut.r_TP_P.clear()
        thrArmConfigOut.shat_P.clear()
        thrArmConfigOut.fhat_P.clear()
        thrArmConfigOut.dcm_C0P.clear()

        thrArmConfigOut.hubMass = self.hubMass
        for v in self.config.r_BcB_B:
            thrArmConfigOut.r_BcB_B.push_back(v)
        for v in self.config.bodyArmIdx:
            thrArmConfigOut.bodyArmIdx.push_back(v)
        for v in self.config.bodyJointIdx:
            thrArmConfigOut.bodyJointIdx.push_back(v)
        for v in self.config.bodyMass:
            thrArmConfigOut.bodyMass.push_back(v)
        for v in self.config.r_LcP_P:
            thrArmConfigOut.r_LcP_P.push_back(v)
        for v in self.config.thrArmIdx:
            thrArmConfigOut.thrArmIdx.push_back(v)
        for v in self.config.thrArmJointIdx:
            thrArmConfigOut.thrArmJointIdx.push_back(v)
        for v in self.config.armTreeIdx:
            thrArmConfigOut.armTreeIdx.push_back(v)
        for v in self.config.armJointCount:
            thrArmConfigOut.armJointCount.push_back(v)
        for v in self.config.r_CP_P:
            thrArmConfigOut.r_CP_P.push_back(v)
        for v in self.config.r_TP_P:
            thrArmConfigOut.r_TP_P.push_back(v)
        for v in self.config.shat_P:
            thrArmConfigOut.shat_P.push_back(v)
        for v in self.config.fhat_P:
            thrArmConfigOut.fhat_P.push_back(v)
        for v in self.config.dcm_C0P:
            thrArmConfigOut.dcm_C0P.push_back(v)
        self.thrArmConfigMsg = messaging.THRArmConfigMsg().write(thrArmConfigOut)

    def SetTransRef(self):
        """Set the translational reference state information"""
        if self.isTracking:
            self.transRef.ModelTag = "arcLineHoldRef"
            self.transRef.setRadius(self.config.transRadius)
            self.transRef.setFinalPosition(self.config.finalPosition)
            self.transRef.setBlendEndPosition(self.config.blendEndPosition)
            self.transRef.setTCircle(self.config.tCircle)
            self.transRef.setTTransition(self.config.tTransition)
            self.transRef.setTLinear(self.config.tLinear)
            return

        transRefOut = messaging.TransRefMsgPayload()
        transRefOut.r_RN_N = self.config.transRefPosition
        transRefOut.v_RN_N = self.config.transRefVelocity
        transRefOut.a_RN_N = self.config.transRefAcceleration
        self.transRefMsg = messaging.TransRefMsg().write(transRefOut)

    def SetInertial3D(self):
        """Defined the inertial 3D guidance module"""
        self.inertial3D.ModelTag = "inertial3D"
        self.inertial3D.sigma_R0N = self.config.attRefSigma_R0N

    def SetAttRef(self):
        """Define the tracking attitude reference pointing module."""
        self.attRef.ModelTag = "attRef"
        self.attRef.scAttInMsg.subscribeTo(self.navConverter.attOutMsg)
        self.attRef.scTransInMsg.subscribeTo(self.navConverter.transOutMsg)
        targetPayload = messaging.NavTransMsgPayload()
        targetPayload.r_BN_N = self.config.scTargetPosition
        self.scTargetMsg = messaging.NavTransMsg().write(targetPayload)
        self.attRef.scTargetInMsg.subscribeTo(self.scTargetMsg)
        self.attRef.pHat_B = self.config.pHat_B
        if self.config.pointingSmallAngle is not None:
            self.attRef.smallAngle = self.config.pointingSmallAngle
        self.attRef.useBoresightRateDamping = self.config.useBoresightRateDamping

    def SetStateMerge(self, SimBase):
        """Define the state merge module"""
        self.stateMerge.ModelTag = "stateMerge"
        self.stateMerge.attStateInMsg.subscribeTo(SimBase.DynModels.scene.getBody(self.hubName).getOrigin().stateOutMsg)
        self.stateMerge.transStateInMsg.subscribeTo(SimBase.DynModels.systemCoM.comStatesOutMsg)

    def SetNavConverter(self):
        """Define the nav converter module"""
        self.navConverter.ModelTag = "navConverter"
        self.navConverter.scStateInMsg.subscribeTo(self.stateMerge.stateOutMsg)

    def SetAttError(self):
        """Define the attitude tracking error module"""
        self.attError.ModelTag = "attError"
        self.attError.attRefInMsg.subscribeTo(self.inertial3D.attRefOutMsg)
        self.attError.attNavInMsg.subscribeTo(self.navConverter.attOutMsg)

    def SetMrpFeedback(self):
        """Define the MRP feedback module"""
        self.mrpFeedback.ModelTag = "mrpFeedback"
        if self.isTracking:
            self.mrpFeedback.guidInMsg.subscribeTo(self.attRef.attGuidOutMsg)
        else:
            self.mrpFeedback.guidInMsg.subscribeTo(self.attError.attGuidOutMsg)
        self.mrpFeedback.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedback.K = self.kRot
        self.mrpFeedback.Ki = self.kiRot
        self.mrpFeedback.P = self.pRot

    def SetInertialCartFeedback(self):
        """Define the inertial cart feedback module"""
        self.inertialCartFeedback.ModelTag = "inertialCartFeedback"
        self.inertialCartFeedback.deputyNavInMsg.subscribeTo(self.navConverter.transOutMsg)
        if self.isTracking:
            self.inertialCartFeedback.deputyRefInMsg.subscribeTo(self.transRef.transRefOutMsg)
        else:
            self.inertialCartFeedback.deputyRefInMsg.subscribeTo(self.transRefMsg)
        self.inertialCartFeedback.deputyVehicleConfigInMsg.subscribeTo(self.vcMsg)
        self.inertialCartFeedback.setK(self.kTrans)
        self.inertialCartFeedback.setP(self.pTrans)

    def SetJointThrAllocation(self, SimBase):
        """Define the joint and thruster allocation module"""
        self.jointThrAllocation.ModelTag = "jointThrAllocation"
        self.jointThrAllocation.armConfigInMsg.subscribeTo(self.thrArmConfigMsg)
        self.jointThrAllocation.hubStatesInMsg.subscribeTo(SimBase.DynModels.scene.getBody(self.hubName).getOrigin().stateOutMsg)
        self.jointThrAllocation.transForceInMsg.subscribeTo(self.inertialCartFeedback.forceOutMsg)
        self.jointThrAllocation.rotTorqueInMsg.subscribeTo(self.mrpFeedback.cmdTorqueOutMsg)
        self.jointThrAllocation.setWc(self.controlWeight)
        self.jointThrAllocation.setWf(self.thrustWeight)
        if self.config.allocationMaxIterations is not None:
            self.jointThrAllocation.maxiter = self.config.allocationMaxIterations
        if self.config.allocationFunctionTolerance is not None:
            self.jointThrAllocation.ftol = self.config.allocationFunctionTolerance
        if self.useThetaAllocPenalty:
            self.jointThrAllocation.setWtheta(self.thetaWeight)
            for index, (bodyName, jointName) in enumerate(self.jointRefs):
                self.jointThrAllocation.addHingedJoint()
                joint = SimBase.DynModels.scene.getBody(bodyName).getScalarJoint(jointName)
                self.jointThrAllocation.jointStatesInMsgs[index].subscribeTo(joint.stateOutMsg)
        self.jointThrAllocation.setThrForceMax(self.thrForces)
        self.desAngleReader = messaging.JointArrayStateMsgReader()
        self.desAngleReader.subscribeTo(self.jointThrAllocation.desJointAnglesOutMsg)

    def SetThrFiringRound(self):
        """Define the thruster firing logic module"""
        self.thrFiringRound.ModelTag = "thrFiringRound"
        self.thrFiringRound.thrForceInMsg.subscribeTo(self.jointThrAllocation.thrForceOutMsg)
        self.thrFiringRound.setControlPeriodSec(self.outerLoopTimeStep*1e-9)
        self.thrFiringRound.setOnTimeResolutionSec(self.config.pulseResolution)
        if self.config.minimumOnTime is not None:
            self.thrFiringRound.setThrMinFireTime(self.config.minimumOnTime)
        self.thrFiringRound.setNumThrusters(self.numThrusters)
        self.thrFiringRound.setThrForceMax(self.thrForces)
        self.thrTimeReader = messaging.THRArrayOnTimeCmdMsgReader()
        self.thrTimeReader.subscribeTo(self.thrFiringRound.thrOnTimeOutMsg)

    def SetHingedJointArrayMotor(self, SimBase):
        """Define the hinged joint array motor module"""
        self.hingedJointArrayMotor.ModelTag = "hingedJointArrayMotor"
        self.hingedJointArrayMotor.massMatrixInMsg.subscribeTo(SimBase.DynModels.systemMassMatrix.massMatrixOutMsg)
        self.hingedJointArrayMotor.reactionForcesInMsg.subscribeTo(SimBase.DynModels.jointReactionForces.reactionForcesOutMsg)
        if self.useJointProfiler:
            self.hingedJointArrayMotor.desJointStatesInMsg.subscribeTo(
                self.jointProfiler.desJointStatesOutMsg
            )
        else:
            self.hingedJointArrayMotor.desJointStatesInMsg.subscribeTo(
                self.jointThrAllocation.desJointAnglesOutMsg
            )
        self.anglesReaders = []
        self.angleRatesReaders = []
        for i, (bodyName, jointName) in enumerate(self.jointRefs):
            self.hingedJointArrayMotor.addHingedJoint()
            joint = SimBase.DynModels.scene.getBody(bodyName).getScalarJoint(jointName)
            self.hingedJointArrayMotor.jointStatesInMsgs[i].subscribeTo(joint.stateOutMsg)
            self.hingedJointArrayMotor.jointStateDotsInMsgs[i].subscribeTo(joint.stateDotOutMsg)
            # Redirect the module outputs to each joint motor torque gateway message
            self.hingedJointArrayMotor.motorTorquesOutMsgs[i] = self.jointTorqueGateways[i]
            # Set up readers for the angles for use in the event system
            angleReader = messaging.ScalarJointStateMsgReader()
            angleRateReader = messaging.ScalarJointStateMsgReader()
            angleReader.subscribeTo(joint.stateOutMsg)
            angleRateReader.subscribeTo(joint.stateDotOutMsg)
            self.anglesReaders.append(angleReader)
            self.angleRatesReaders.append(angleRateReader)
        self.hingedJointArrayMotor.setKtheta(self.kTheta)
        self.hingedJointArrayMotor.setPtheta(self.pTheta)
        self.hingedJointArrayMotor.setUMax(self.maxJointTorques)

    def SetJointMotionCompensator(self, SimBase):
        """Define the joint motion compensator module"""
        self.jointMotionCompensator.ModelTag = "jointMotionCompensator"
        self.jointMotionCompensator.massMatrixInMsg.subscribeTo(SimBase.DynModels.systemMassMatrix.massMatrixOutMsg)
        self.jointMotionCompensator.reactionForcesInMsg.subscribeTo(SimBase.DynModels.jointReactionForces.reactionForcesOutMsg)
        self.jointMotionCompensator.addSpacecraft()
        for i in range(self.numJoints):
            self.jointMotionCompensator.addHingedJoint()
            self.jointMotionCompensator.jointTorqueInMsgs[i].subscribeTo(self.jointTorqueGateways[i])
        for i in range(3):
            # Redirect the module outputs to each hub motor torque gateway message
            self.jointMotionCompensator.hubTorqueOutMsgs[i] = self.bodyTorqueGateway[i]
        if self.maxPreTorques is not None:
            self.jointMotionCompensator.setUMax(self.maxPreTorques)

    def SetThrOnTimeToForce(self):
        """Define the thruster on-time to force conversion module"""
        self.thrOnTimeToForce.ModelTag = "thrOnTimeToForce"
        self.thrOnTimeToForce.onTimeInMsg.subscribeTo(self.thrFiringTimeGateway)
        for i in range(self.numThrusters):
            self.thrOnTimeToForce.addThruster()
            # Redirect the module outputs to each thruster force gateway message
            self.thrOnTimeToForce.thrusterForceOutMsgs[i] = self.thrForceGateways[i]
        self.thrOnTimeToForce.setThrMag(self.thrForces)

    def SetThrJointCompensation(self, SimBase):
        """Define the thruster-joint compensation module"""
        self.thrJointCompensation.ModelTag = "thrJointCompensation"
        self.thrJointCompensation.armConfigInMsg.subscribeTo(self.thrArmConfigMsg)
        self.thrJointCompensation.massMatrixInMsg.subscribeTo(SimBase.DynModels.systemMassMatrix.massMatrixOutMsg)
        self.thrJointCompensation.reactionForcesInMsg.subscribeTo(SimBase.DynModels.jointReactionForces.reactionForcesOutMsg)
        for i in range(self.numThrusters):
            self.thrJointCompensation.addThruster()
            self.thrJointCompensation.thrForcesInMsgs[i].subscribeTo(self.thrForceGateways[i])
        for i, (bodyName, jointName) in enumerate(self.jointRefs):
            self.thrJointCompensation.addHingedJoint()
            joint = SimBase.DynModels.scene.getBody(bodyName).getScalarJoint(jointName)
            self.thrJointCompensation.jointStatesInMsgs[i].subscribeTo(joint.stateOutMsg)
            # Redirect the module outputs to each joint motor torque gateway message
            self.thrJointCompensation.motorTorquesOutMsgs[i] = self.jointTorqueGateways[i]
        self.thrJointCompensation.setUMax(self.maxJointTorques)

    def SetJointProfiler(self, SimBase):
        """Define the optional joint reference profiler."""
        self.jointProfiler.ModelTag = "jointProfiler"
        self.jointProfiler.setProfileType(self.config.jointProfileType)
        self.jointProfiler.setUseShortestPath(self.config.jointProfileUseShortestPath)
        if self.config.jointProfileWrapStart is not None:
            self.jointProfiler.setJointAngleWrapStart(self.config.jointProfileWrapStart)
        if self.config.jointProfileType == "lowPass":
            self.jointProfiler.setWc(self.config.jointProfileCutoffFrequency)
            self.jointProfiler.setFilterDt(self.config.jointProfileFilterTimeStep)
        else:
            self.jointProfiler.setProfileDuration(self.config.jointProfileDuration)
        for index, (bodyName, jointName) in enumerate(self.jointRefs):
            self.jointProfiler.addHingedJoint()
            joint = SimBase.DynModels.scene.getBody(bodyName).getScalarJoint(jointName)
            self.jointProfiler.jointStatesInMsgs[index].subscribeTo(joint.stateOutMsg)
            self.jointProfiler.jointStateDotsInMsgs[index].subscribeTo(joint.stateDotOutMsg)
        self.jointProfiler.desJointStatesInMsg.subscribeTo(
            self.jointThrAllocation.desJointAnglesOutMsg
        )

    def InitAllFSWObjects(self, SimBase):
        """Initialize all FSW objects in the simulation"""
        self.SetVehicleConfiguration()
        self.SetThrArmConfig()
        self.SetTransRef()
        self.SetStateMerge(SimBase)
        self.SetNavConverter()
        if self.isTracking:
            self.SetAttRef()
        else:
            self.SetInertial3D()
            self.SetAttError()
        self.SetMrpFeedback()
        self.SetInertialCartFeedback()
        self.SetJointThrAllocation(SimBase)
        self.SetThrFiringRound()
        self.SetHingedJointArrayMotor(SimBase)
        if self.useJointMotionCompensation:
            self.SetJointMotionCompensator(SimBase)
        self.SetThrOnTimeToForce()
        self.SetThrJointCompensation(SimBase)
        if self.useJointProfiler:
            self.SetJointProfiler(SimBase)

    def setupGatewayMsgs(self, SimBase):
        """create stand-alone actuator gateway messages such that different modules can write this message (or it can be left as zero)
        and provide a common input message for the mujoco actuators"""
        self.jointTorqueGateways = [messaging.SingleActuatorMsg() for _ in range(self.numJoints)]
        self.thrForceGateways = [messaging.SingleActuatorMsg() for _ in range(self.numThrusters)]
        self.bodyTorqueGateway = [messaging.SingleActuatorMsg() for _ in range(3)]
        self.thrFiringTimeGateway = messaging.THRArrayOnTimeCmdMsg()

        self.zeroGatewayMsgs()

        # connect the gateway messages to the appropriate mujoco actuator input messages
        for i, name in enumerate(self.jointActNames):
            SimBase.DynModels.scene.getSingleActuator(name).actuatorInMsg.subscribeTo(self.jointTorqueGateways[i])
        for i, name in enumerate(self.thrActNames):
            SimBase.DynModels.scene.getSingleActuator(name).actuatorInMsg.subscribeTo(self.thrForceGateways[i])
        for i, name in enumerate(self.bodyActNames):
            SimBase.DynModels.scene.getSingleActuator(name).actuatorInMsg.subscribeTo(self.bodyTorqueGateway[i])

        # Setup readers for the thruster force gateway messages
        self.thrForceReaders = []
        for i in range(self.numThrusters):
            reader = messaging.SingleActuatorMsgReader()
            reader.subscribeTo(self.thrForceGateways[i])
            self.thrForceReaders.append(reader)

    def zeroGatewayMsgs(self):
        zeroActMsg = messaging.SingleActuatorMsgPayload(input=0.0)
        zeroTimeMsg = messaging.THRArrayOnTimeCmdMsgPayload()
        for m in self.jointTorqueGateways:
            m.write(zeroActMsg)
        for m in self.thrForceGateways:
            m.write(zeroActMsg)
        for m in self.bodyTorqueGateway:
            m.write(zeroActMsg)
        self.thrFiringTimeGateway.write(zeroTimeMsg)

    def wrapAngle(self, angle):
        """Helper function to wrap angles to the range [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
