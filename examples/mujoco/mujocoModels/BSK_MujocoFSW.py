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

import os
import sys
import numpy as np
supportFilePath = os.path.join(os.path.dirname(__file__), "..", "thrArmControlSupport")
sys.path.append(supportFilePath)
import jointThrAllocation
import stateMerge
import thrFiringRound
from Basilisk.architecture import messaging
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (
    hingedJointArrayMotor,
    jointMotionCompensator,
    thrJointCompensation,
    inertial3D,
    attTrackingError,
    mrpFeedback,
    inertialCartFeedback,
)
from Basilisk.simulation import (
    thrOnTimeToForce,
    simpleNav,
)


class BSKMujocoFSWModels:
    """Defines the bskSim FSW class"""
    def __init__(self, SimBase, fswRate):
        # define empty class variables
        self.vcMsg = None
        self.thrArmConfigMsg = None
        self.transRefMsg = None
        self.jointTorqueGateways = []
        self.thrForceGateways = []
        self.bodyTorqueGateway = []
        self.thrFiringTimeGateway = None

        # Define process name and task time-steps
        self.processName = SimBase.FSWProcessName
        self.innerLoopTimeStep = mc.sec2nano(fswRate)
        self.outerLoopTimeStep = mc.sec2nano(10.0)

        # Define default values for the scenario
        self.numJoints = 8
        self.numThrusters = 2
        self.thrForces = [2.5]*self.numThrusters # N
        self.maxJointTorques = [22.5]*self.numJoints # Nm
        self.controlWeight = 1.0
        self.thrustWeight = 1e-6

        # Define the default control gains
        self.Tpos = 27.0
        self.Tatt = 10.0
        self.omegaOuter = 2*np.pi/10
        self.omegaInner = 5*self.omegaOuter
        piRot = (2*1/3*330.0*(0.79**2+0.69**2))/self.Tatt
        self.pRot = piRot
        xiRot = 1.0
        self.kRot = piRot**2/(4*xiRot**2*(1/3*330.0*(0.79**2+0.69**2)))
        self.kiRot = -1.0
        piPos = 2.0*350.0/self.Tpos
        self.pTrans = (piPos * np.eye(3)).flatten().tolist()
        xiPos = 1.0
        self.kTrans = (piPos**2/(4*xiPos**2*350.0) * np.eye(3)).flatten().tolist()
        kiMotor = self.omegaInner**2
        self.kTheta = (kiMotor * np.eye(8)).flatten().tolist()
        xiMotor = 1.5
        piMotor = 2 * xiMotor * self.omegaInner
        self.pTheta = (piMotor * np.eye(8)).flatten().tolist()


        # Define the XML object names for later use
        self.hubName = "hub"
        self.jointRefs = [
            ("arm_1_base", "arm_1_joint_1"),
            ("arm_1_base", "arm_1_joint_2"),
            ("arm_1_tip",  "arm_1_joint_3"),
            ("arm_1_tip",  "arm_1_joint_4"),
            ("arm_2_base", "arm_2_joint_1"),
            ("arm_2_base", "arm_2_joint_2"),
            ("arm_2_tip",  "arm_2_joint_3"),
            ("arm_2_tip",  "arm_2_joint_4"),
        ]
        self.jointActNames = ["u_arm_1_joint_1","u_arm_1_joint_2","u_arm_1_joint_3","u_arm_1_joint_4",
                              "u_arm_2_joint_1","u_arm_2_joint_2","u_arm_2_joint_3","u_arm_2_joint_4"]
        self.thrActNames = ["F_thruster_1","F_thruster_2"]
        self.bodyActNames = ["tau_prescribed_1","tau_prescribed_2","tau_prescribed_3"]

        # Define initial event check values
        self.lastDesAngles = [0.0]*self.numJoints
        self.lastDesAngleRates = [0.0]*self.numJoints
        self.lastDesCmdWriteTime = -1
        self.thrustJustActivated = False

        # Instantiate FSW modules as objects
        self.inertial3D = inertial3D.inertial3D()
        self.stateMerge = stateMerge.StateMerge()
        self.navConverter = simpleNav.SimpleNav()
        self.attError = attTrackingError.attTrackingError()
        self.mrpFeedback = mrpFeedback.mrpFeedback()
        self.inertialCartFeedback = inertialCartFeedback.InertialCartFeedback()
        self.jointThrAllocation = jointThrAllocation.JointThrAllocation()
        self.thrFiringRound = thrFiringRound.ThrFiringRound()
        self.hingedJointArrayMotor = hingedJointArrayMotor.HingedJointArrayMotor()
        self.jointMotionCompensator = jointMotionCompensator.JointMotionCompensator()
        self.thrOnTimeToForce = thrOnTimeToForce.ThrOnTimeToForce()
        self.thrJointCompensation = thrJointCompensation.ThrJointCompensation()

        # Create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        self.outerLoopTaskName = "outerLoopTask"
        self.jointMotionTaskName = "jointMotionTask"
        self.thrFiringTaskName = "thrFiringTask"
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.outerLoopTaskName, self.outerLoopTimeStep), 100)
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.jointMotionTaskName, self.innerLoopTimeStep), 99)
        SimBase.fswProc.addTask(SimBase.CreateNewTask(self.thrFiringTaskName, self.innerLoopTimeStep), 98)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.outerLoopTaskName, self.inertial3D, 50)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.stateMerge, 49)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.navConverter, 48)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.attError, 47)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.mrpFeedback, 46)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.inertialCartFeedback, 45)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.jointThrAllocation, 44)
        SimBase.AddModelToTask(self.outerLoopTaskName, self.thrFiringRound, 43)

        SimBase.AddModelToTask(self.jointMotionTaskName, self.hingedJointArrayMotor, 50)
        SimBase.AddModelToTask(self.jointMotionTaskName, self.jointMotionCompensator, 49)

        SimBase.AddModelToTask(self.thrFiringTaskName, self.thrOnTimeToForce, 50)
        SimBase.AddModelToTask(self.thrFiringTaskName, self.thrJointCompensation, 49)

        # Create events to to be called for changing active controllers
        SimBase.fswProc.disableAllTasks()
        self.zeroGatewayMsgs()
        SimBase.enableTask(self.outerLoopTaskName)

        def conditionJointMotion(self):
            """Event condition to use the joint motion controller after the outer
            control loop has been updated"""
            desAngles = list(self.FSWModels.desAngleReader().states)
            desAngleRates = list(self.FSWModels.desAngleReader().stateDots)
            currentWriteTime = self.FSWModels.desAngleReader.timeWritten()
            if len(desAngles) != self.FSWModels.numJoints or len(desAngleRates) != self.FSWModels.numJoints:
                return False
            changed = currentWriteTime != self.FSWModels.lastDesCmdWriteTime
            if changed:
                self.FSWModels.lastDesAngles = desAngles[:]
                self.FSWModels.lastDesAngleRates = desAngleRates[:]
                self.FSWModels.lastDesCmdWriteTime = currentWriteTime
            return changed

        def actionJointMotion(self):
            """Event action to activate the joint motion controller when there
            is a change in the desired joint angles or thruster forces"""
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
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
            """Event condition to fire the thrusters once the joints have
            reached the desired angles and stopped"""
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
            return (stateError < 1e-3) and (rateError < 1e-3)

        def actionThrFiring(self):
            """Event action to activate the thruster firing when the joints have
            reached the desired angles and stopped"""
            self.FSWModels.thrustJustActivated = True
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
            self.enableTask(self.FSWModels.thrFiringTaskName)
            CurrentSimNanos = self.TotalSim.CurrentNanos
            thrOnTimes = self.FSWModels.thrTimeReader().OnTimeRequest
            payload = messaging.THRArrayOnTimeCmdMsgPayload()
            payload.OnTimeRequest = list(thrOnTimes)
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
            if self.FSWModels.thrustJustActivated:
                self.FSWModels.thrustJustActivated = False
                return False
            return (np.max(np.abs(thrForces)) < 1e-12)

        def actionCoast(self):
            """Event action to activate coasting (no commands) between the time
            when the thrusters are done firing and the next outer loop update"""
            self.fswProc.disableAllTasks()
            self.FSWModels.zeroGatewayMsgs()
            self.enableTask(self.FSWModels.outerLoopTaskName)
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
    # These are module-initialization methods

    def SetVehicleConfiguration(self):
        """Set the spacecraft configuration information"""
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        vehicleConfigOut.massSC = 350.0
        # use the hub inertia for mrp feedback module since will not have instantaneous inertia otherwise
        vehicleConfigOut.ISCPntB_B = [1/3*330.0*(0.69**2+0.52**2), 0.0, 0.0, 0.0, 1/3*330.0*(0.79**2+0.52**2), 0.0, 0.0, 0.0, 1/3*330.0*(0.79**2+0.69**2)]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetThrArmConfig(self):
        """Set the thruster arm configuration information"""
        thrArmConfigOut = messaging.THRArmConfigMsgPayload()
        thrArmIdx = [0, 1]
        thrArmJointIdx = [3, 3]
        armTreeIdx = [0, 0]
        armJointCount = [4, 4]
        r_CP_P = [
            0.79, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            -0.79, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]
        r_TP_P = [
            0.1, 0.0, 0.0,
            0.1, 0.0, 0.0,
        ]
        shat_P = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
        ]
        fhat_P = [
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
        ]
        identity = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        arm2BaseDcm = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        dcm_C0P = (
            identity + identity + identity + identity
            + arm2BaseDcm + identity + identity + identity
        )

        thrArmConfigOut.thrArmIdx.clear()
        thrArmConfigOut.thrArmJointIdx.clear()
        thrArmConfigOut.armTreeIdx.clear()
        thrArmConfigOut.armJointCount.clear()
        thrArmConfigOut.r_CP_P.clear()
        thrArmConfigOut.r_TP_P.clear()
        thrArmConfigOut.shat_P.clear()
        thrArmConfigOut.fhat_P.clear()
        thrArmConfigOut.dcm_C0P.clear()
        for v in thrArmIdx:
            thrArmConfigOut.thrArmIdx.push_back(v)
        for v in thrArmJointIdx:
            thrArmConfigOut.thrArmJointIdx.push_back(v)
        for v in armTreeIdx:
            thrArmConfigOut.armTreeIdx.push_back(v)
        for v in armJointCount:
            thrArmConfigOut.armJointCount.push_back(v)
        for v in r_CP_P:
            thrArmConfigOut.r_CP_P.push_back(v)
        for v in r_TP_P:
            thrArmConfigOut.r_TP_P.push_back(v)
        for v in shat_P:
            thrArmConfigOut.shat_P.push_back(v)
        for v in fhat_P:
            thrArmConfigOut.fhat_P.push_back(v)
        for v in dcm_C0P:
            thrArmConfigOut.dcm_C0P.push_back(v)
        self.thrArmConfigMsg = messaging.THRArmConfigMsg().write(thrArmConfigOut)

    def SetTransRef(self):
        """Set the translational reference state information"""
        transRefOut = messaging.TransRefMsgPayload()
        transRefOut.r_RN_N = [0.0, 0.0, 0.0]
        transRefOut.v_RN_N = [0.0, 0.0, 0.0]
        transRefOut.a_RN_N = [0.0, 0.0, 0.0]
        self.transRefMsg = messaging.TransRefMsg().write(transRefOut)

    def SetInertial3D(self):
        """Defined the inertial 3D guidance module"""
        self.inertial3D.ModelTag = "inertial3D"
        self.inertial3D.sigma_R0N = [0.0,0.0,0.0]

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
        self.mrpFeedback.guidInMsg.subscribeTo(self.attError.attGuidOutMsg)
        self.mrpFeedback.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedback.K = self.kRot
        self.mrpFeedback.Ki = self.kiRot
        self.mrpFeedback.P = self.pRot

    def SetInertialCartFeedback(self):
        """Define the inertial cart feedback module"""
        self.inertialCartFeedback.ModelTag = "inertialCartFeedback"
        self.inertialCartFeedback.deputyNavInMsg.subscribeTo(self.navConverter.transOutMsg)
        self.inertialCartFeedback.deputyRefInMsg.subscribeTo(self.transRefMsg)
        self.inertialCartFeedback.deputyVehicleConfigInMsg.subscribeTo(self.vcMsg)
        self.inertialCartFeedback.setK(self.kTrans)
        self.inertialCartFeedback.setP(self.pTrans)

    def SetJointThrAllocation(self, SimBase):
        """Define the joint and thruster allocation module"""
        self.jointThrAllocation.ModelTag = "jointThrAllocation"
        self.jointThrAllocation.armConfigInMsg.subscribeTo(self.thrArmConfigMsg)
        self.jointThrAllocation.CoMStatesInMsg.subscribeTo(SimBase.DynModels.systemCoM.comStatesOutMsg)
        self.jointThrAllocation.hubStatesInMsg.subscribeTo(SimBase.DynModels.scene.getBody(self.hubName).getOrigin().stateOutMsg)
        self.jointThrAllocation.transForceInMsg.subscribeTo(self.inertialCartFeedback.forceOutMsg)
        self.jointThrAllocation.rotTorqueInMsg.subscribeTo(self.mrpFeedback.cmdTorqueOutMsg)

        self.jointThrAllocation.setWc(self.controlWeight)
        self.jointThrAllocation.setWf(self.thrustWeight)
        self.jointThrAllocation.setThrForceMax(self.thrForces)

        self.desAngleReader = messaging.JointArrayStateMsgReader()
        self.desAngleReader.subscribeTo(self.jointThrAllocation.desJointAnglesOutMsg)

    def SetThrFiringRound(self):
        """Define the thruster firing logic module"""
        self.thrFiringRound.ModelTag = "thrFiringRound"
        self.thrFiringRound.thrForceInMsg.subscribeTo(self.jointThrAllocation.thrForceOutMsg)
        self.thrFiringRound.setControlPeriodSec(self.outerLoopTimeStep*1e-9)
        self.thrFiringRound.setOnTimeResolutionSec(self.innerLoopTimeStep*1e-9)
        self.thrFiringRound.setNumThrusters(self.numThrusters)
        self.thrFiringRound.setThrForceMax(self.thrForces)
        self.thrTimeReader = messaging.THRArrayOnTimeCmdMsgReader()
        self.thrTimeReader.subscribeTo(self.thrFiringRound.thrOnTimeOutMsg)

    def SetHingedJointArrayMotor(self, SimBase):
        """Define the hinged joint array motor module"""
        self.hingedJointArrayMotor.ModelTag = "hingedJointArrayMotor"
        self.hingedJointArrayMotor.massMatrixInMsg.subscribeTo(SimBase.DynModels.systemMassMatrix.massMatrixOutMsg)
        self.hingedJointArrayMotor.reactionForcesInMsg.subscribeTo(SimBase.DynModels.jointReactionForces.reactionForcesOutMsg)
        self.hingedJointArrayMotor.desJointStatesInMsg.subscribeTo(self.jointThrAllocation.desJointAnglesOutMsg)
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

    def InitAllFSWObjects(self, SimBase):
        """Initialize all FSW objects in the simulation"""
        self.SetVehicleConfiguration()
        self.SetThrArmConfig()
        self.SetTransRef()
        self.SetInertial3D()
        self.SetStateMerge(SimBase)
        self.SetNavConverter()
        self.SetAttError()
        self.SetMrpFeedback()
        self.SetInertialCartFeedback()
        self.SetJointThrAllocation(SimBase)
        self.SetThrFiringRound()
        self.SetHingedJointArrayMotor(SimBase)
        self.SetJointMotionCompensator(SimBase)
        self.SetThrOnTimeToForce()
        self.SetThrJointCompensation(SimBase)

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
