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
from scipy.optimize import minimize

from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk

def mapMatrix(rVec_B: np.ndarray, fHatVec_B: np.ndarray, r_ComB_B: np.ndarray) -> np.ndarray:
    """Build wrench map U=[F;L]=T@f where f are thruster magnitudes."""
    rVec_B = np.asarray(rVec_B, dtype=float)
    fHatVec_B = np.asarray(fHatVec_B, dtype=float)
    r_ComB_B = np.asarray(r_ComB_B, dtype=float).reshape(3)
    forceBlock = fHatVec_B.T
    torqueBlock = np.cross(rVec_B - r_ComB_B.reshape(1, 3), fHatVec_B, axis=1).T
    return np.vstack((forceBlock, torqueBlock))


class JointThrAllocation(sysModel.SysModel):
    """
    Example support module for thruster-on-arm allocation.

    This implementation is intentionally example-oriented with explicit assumptions:
    - Arms are serial chains packed in arm order.
    - Arm geometry/config comes from THRArmConfigMsgPayload.
    - One spacecraft tree is supported (all arms in same kinematic tree).
    - Exactly one thruster per arm (configurable check).
    - Thruster parent joint is the last joint in each arm (configurable check).
    """

    def __init__(self):
        super().__init__()

        # Input messages
        self.armConfigInMsg = messaging.THRArmConfigMsgReader()
        self.CoMStatesInMsg = messaging.SCStatesMsgReader()
        self.hubStatesInMsg = messaging.SCStatesMsgReader()
        self.transForceInMsg = messaging.CmdForceInertialMsgReader()
        self.rotTorqueInMsg = messaging.CmdTorqueBodyMsgReader()

        # Output messages
        self.thrForceOutMsg = messaging.THRArrayCmdForceMsg()
        self.desJointAnglesOutMsg = messaging.JointArrayStateMsg()
        self.thrForcePayload = messaging.THRArrayCmdForceMsgPayload()
        self.jointAnglePayload = messaging.JointArrayStateMsgPayload()

        # Cost settings
        self.Wc = np.eye(6)
        self.WfScale = 1e-6
        self.Wf = None

        # Optimization settings
        self.maxiter = 3000
        self.ftol = 1e-10
        self.errTol = 1e-4
        self.thrForceMax = 2.5

        # Runtime data
        self.nArms = 0
        self.nThr = 0
        self.nJoint = 0
        self.armJointCount = None
        self.armJointStart = None
        self.thrArmIdx = None
        self.thrArmJointIdx = None
        self.r_CP_P = None
        self.r_TP_P = None
        self.sHat_P = None
        self.fHat_P = None
        self.dcm_C0P = None
        self.x0 = None

    def setWf(self, wfIn):
        """
        Set thrust-weight term for the cost function.

        Accepted inputs:
        - scalar: applies same weight to all thrusters
        - vector length nThr: per-thruster weights
        """
        wfArr = np.asarray(wfIn, dtype=float)
        if wfArr.ndim == 0:
            self.WfScale = float(wfArr)
            self.Wf = None
            return
        if wfArr.ndim != 1:
            raise ValueError("setWf expects a scalar or 1D vector.")
        self.Wf = wfArr.copy()

    def setWc(self, wcIn):
        """
        Set wrench tracking weights for the cost function.

        Accepted inputs:
        - scalar: wc * I6
        - length-6 vector: diag(wc)
        - 6x6 matrix
        """
        wcArr = np.asarray(wcIn, dtype=float)
        if wcArr.ndim == 0:
            self.Wc = float(wcArr) * np.eye(6)
            return
        if wcArr.ndim == 1 and wcArr.size == 6:
            self.Wc = np.diag(wcArr)
            return
        if wcArr.shape == (6, 6):
            self.Wc = wcArr.copy()
            return
        raise ValueError("setWc expects scalar, length-6 vector, or 6x6 matrix.")

    def resolveWf(self):
        """Resolve Wf to a length-nThr vector after nThr is known."""
        if self.Wf is None:
            self.Wf = np.full(self.nThr, self.WfScale, dtype=float)
            return

        wfArr = np.asarray(self.Wf, dtype=float)
        if wfArr.ndim == 0:
            self.Wf = np.full(self.nThr, float(wfArr), dtype=float)
            return
        if wfArr.ndim != 1:
            raise ValueError("Wf must be scalar or 1D vector.")
        if wfArr.size == 1:
            self.Wf = np.full(self.nThr, float(wfArr[0]), dtype=float)
            return
        if wfArr.size != self.nThr:
            raise ValueError(f"Wf vector length {wfArr.size} does not match nThr {self.nThr}.")
        self.Wf = wfArr.copy()

    def setThrForceMax(self, thrForceMaxIn):
        """
        Set thrust upper bounds.

        Accepted inputs:
        - scalar: same upper bound for all thrusters
        - vector length nThr: per-thruster upper bounds
        """
        thrForceMaxArr = np.asarray(thrForceMaxIn, dtype=float)
        if thrForceMaxArr.ndim == 0:
            self.thrForceMax = float(thrForceMaxArr)
            return
        if thrForceMaxArr.ndim != 1:
            raise ValueError("setThrForceMax expects a scalar or 1D vector.")
        self.thrForceMax = thrForceMaxArr.copy()

    def resolveThrForceMax(self):
        """Resolve thrForceMax to a length-nThr vector after nThr is known."""
        thrForceMaxArr = np.asarray(self.thrForceMax, dtype=float)
        if thrForceMaxArr.ndim == 0:
            return np.full(self.nThr, float(thrForceMaxArr), dtype=float)
        if thrForceMaxArr.ndim != 1:
            raise ValueError("thrForceMax must be scalar or 1D vector.")
        if thrForceMaxArr.size == 1:
            return np.full(self.nThr, float(thrForceMaxArr[0]), dtype=float)
        if thrForceMaxArr.size != self.nThr:
            raise ValueError(f"thrForceMax vector length {thrForceMaxArr.size} does not match nThr {self.nThr}.")
        return thrForceMaxArr.copy()

    def parseArmConfig(self):
        cfgMsg = self.armConfigInMsg()

        self.thrArmIdx = np.asarray(cfgMsg.thrArmIdx, dtype=int)
        self.thrArmJointIdx = np.asarray(cfgMsg.thrArmJointIdx, dtype=int)
        self.armJointCount = np.asarray(cfgMsg.armJointCount, dtype=int)

        self.nArms = int(self.armJointCount.size)
        self.nThr = int(self.thrArmIdx.size)
        self.nJoint = int(np.sum(self.armJointCount))

        self.armJointStart = np.zeros(self.nArms, dtype=int)
        cumulativeCount = 0
        for armIdx in range(self.nArms):
            self.armJointStart[armIdx] = cumulativeCount
            cumulativeCount += int(self.armJointCount[armIdx])

        self.r_CP_P = np.asarray(cfgMsg.r_CP_P, dtype=float).reshape(-1, 3)
        self.r_TP_P = np.asarray(cfgMsg.r_TP_P, dtype=float).reshape(-1, 3)
        self.sHat_P = np.asarray(cfgMsg.shat_P, dtype=float).reshape(-1, 3)
        self.fHat_P = np.asarray(cfgMsg.fhat_P, dtype=float).reshape(-1, 3)
        self.dcm_C0P = np.asarray(cfgMsg.dcm_C0P, dtype=float).reshape(-1, 9)

        # Convert each flat 9-entry block from column-major to 3x3 matrix.
        self.dcm_C0P = np.array([dcmFlat.reshape(3, 3, order="F") for dcmFlat in self.dcm_C0P], dtype=float)

    def initialGuesses(self):
        nDecision = self.nJoint + self.nThr
        seedList = []

        guess = np.zeros(nDecision)
        guess[self.nJoint:] = 1.0
        seedList.append(guess)

        for angleSeed in (np.pi / 4.0, -np.pi / 4.0, np.pi / 2.0, -np.pi / 2.0):
            guess = np.zeros(nDecision)
            guess[: self.nJoint] = angleSeed
            guess[self.nJoint:] = 1.0
            seedList.append(guess)

        self.x0 = np.vstack(seedList)

    def bounds(self):
        nDecision = self.nJoint + self.nThr
        lowerBound = np.zeros(nDecision)
        upperBound = np.zeros(nDecision)
        lowerBound[: self.nJoint] = -np.pi
        upperBound[: self.nJoint] = np.pi
        lowerBound[self.nJoint:] = 0.0
        upperBound[self.nJoint:] = self.resolveThrForceMax()
        return tuple((float(low), float(high)) for low, high in zip(lowerBound, upperBound))

    def jointPoseFromTheta(self, theta: np.ndarray):
        """
        Return joint frame poses in B:
        - dcm_cb[k]: DCM from joint frame C_k to body frame B
        - r_cb_b[k]: position of joint frame C_k origin in B
        """
        dcm_CB = [np.eye(3) for _ in range(self.nJoint)]
        r_CB_B = [np.zeros(3) for _ in range(self.nJoint)]

        for armIdx in range(self.nArms):
            armStart = int(self.armJointStart[armIdx])
            armJointNum = int(self.armJointCount[armIdx])

            for jointLocalIdx in range(armJointNum):
                jointFlatIdx = armStart + jointLocalIdx
                if jointLocalIdx == 0:
                    dcm_PB = np.eye(3)
                    r_PB_B = np.zeros(3)
                else:
                    priorJointIdx = jointFlatIdx - 1
                    dcm_PB = dcm_CB[priorJointIdx]
                    r_PB_B = r_CB_B[priorJointIdx]

                dcm_CC0 = rbk.PRV2C(theta[jointFlatIdx] * self.sHat_P[jointFlatIdx])
                dcm_CP = dcm_CC0 @ self.dcm_C0P[jointFlatIdx]

                r_CB_B[jointFlatIdx] = r_PB_B + dcm_PB.T @ self.r_CP_P[jointFlatIdx]
                dcm_CB[jointFlatIdx] = dcm_CP @ dcm_PB

        return dcm_CB, r_CB_B

    def mapping(self, theta: np.ndarray, r_ComB_B: np.ndarray):
        dcm_CB, r_CB_B = self.jointPoseFromTheta(theta)

        r_TB_B = np.zeros((self.nThr, 3))
        fHatVec_B = np.zeros((self.nThr, 3))

        for thrIdx in range(self.nThr):
            armIdx = int(self.thrArmIdx[thrIdx])
            jointLocalIdx = int(self.thrArmJointIdx[thrIdx])
            jointFlatIdx = int(self.armJointStart[armIdx] + jointLocalIdx)

            r_TB_B[thrIdx] = r_CB_B[jointFlatIdx] + dcm_CB[jointFlatIdx].T @ self.r_TP_P[thrIdx]
            fHatVec_B[thrIdx] = dcm_CB[jointFlatIdx].T @ self.fHat_P[thrIdx]

        return mapMatrix(r_TB_B, fHatVec_B, r_ComB_B)

    def cost(self, decisionVar: np.ndarray, r_ComB_B: np.ndarray, desiredWrench_B: np.ndarray) -> float:
        theta = decisionVar[: self.nJoint]
        thrForces = decisionVar[self.nJoint:]
        wrenchMap = self.mapping(theta, r_ComB_B)
        wrenchError = desiredWrench_B - wrenchMap @ thrForces
        return float(wrenchError.T @ self.Wc @ wrenchError + self.Wf.T @ thrForces)

    def Reset(self, CurrentSimNanos):
        self.parseArmConfig()
        self.resolveWf()
        self.initialGuesses()

        self.desJointAnglesOutMsg.write(messaging.JointArrayStateMsgPayload())
        self.thrForceOutMsg.write(messaging.THRArrayCmdForceMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        comStates = self.CoMStatesInMsg()
        hubStates = self.hubStatesInMsg()

        r_ComB_N = np.array(comStates.r_CN_N).reshape(3) - np.array(hubStates.r_BN_N).reshape(3)
        sigmaBN = np.array(hubStates.sigma_BN).reshape(3)
        dcm_BN = rbk.MRP2C(sigmaBN)
        r_ComB_B = dcm_BN @ r_ComB_N

        forceInertialMsg = self.transForceInMsg()
        torqueBodyMsg = self.rotTorqueInMsg()

        desiredForce_N = np.array(forceInertialMsg.forceRequestInertial).reshape(3)
        desiredForce_B = dcm_BN @ desiredForce_N
        desiredTorque_B = np.array(torqueBodyMsg.torqueRequestBody).reshape(3)
        desiredWrench_B = np.hstack((desiredForce_B, desiredTorque_B))

        optOptions = {"maxiter": self.maxiter, "ftol": self.ftol, "disp": False}
        boundTuple = self.bounds()

        bestDecision = None
        bestErrInf = np.inf

        for initialDecision in self.x0:
            optResult = minimize(
                fun=lambda decision: self.cost(decision, r_ComB_B, desiredWrench_B),
                x0=initialDecision,
                bounds=boundTuple,
                method="SLSQP",
                options=optOptions,
            )
            if not optResult.success:
                continue

            decisionOpt = optResult.x
            wrenchError = (
                desiredWrench_B
                - self.mapping(decisionOpt[: self.nJoint], r_ComB_B) @ decisionOpt[self.nJoint:]
            )
            errInf = float(np.linalg.norm(wrenchError, ord=np.inf))
            if errInf < bestErrInf:
                bestErrInf = errInf
                bestDecision = decisionOpt
            if bestErrInf <= self.errTol:
                break

        if bestDecision is None:
            bestDecision = np.zeros(self.nJoint + self.nThr)

        self.thrForcePayload.thrForce = bestDecision[self.nJoint:].tolist()
        self.thrForceOutMsg.write(self.thrForcePayload, CurrentSimNanos, self.moduleID)

        self.jointAnglePayload.states.clear()
        self.jointAnglePayload.stateDots.clear()
        for angleCmd in bestDecision[: self.nJoint]:
            self.jointAnglePayload.states.push_back(float(angleCmd))
            self.jointAnglePayload.stateDots.push_back(0.0)
        self.desJointAnglesOutMsg.write(self.jointAnglePayload, CurrentSimNanos, self.moduleID)
