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

from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk


def _get_optimizer():
    try:
        from scipy.optimize import minimize
    except ImportError as exc:
        raise ImportError(
            "JointThrAllocation requires SciPy. Install Basilisk with "
            "`pip install -e .[examples]` to use this module."
        ) from exc

    return minimize


def wrapAngle(angles: np.ndarray) -> np.ndarray:
    """Wrap an angle or angle array to the range :math:`[-\\pi, \\pi]`."""
    angleWrapped = (np.asarray(angles) + np.pi) % (2.0 * np.pi) - np.pi
    if np.ndim(angleWrapped) == 0:
        return float(angleWrapped)
    return angleWrapped


def mapMatrix(
    rVec_B: np.ndarray, fHatVec_B: np.ndarray, r_ComB_B: np.ndarray
) -> np.ndarray:
    """
    Build the thruster force-to-wrench map.

    :param rVec_B: Thruster locations in body-frame coordinates.
    :param fHatVec_B: Thruster unit force directions in body-frame coordinates.
    :param r_ComB_B: Center-of-mass location relative to the hub origin in
        body-frame coordinates.
    :return: Matrix mapping thruster magnitudes to stacked force and torque.
    """
    rVec_B = np.asarray(rVec_B, dtype=float)
    fHatVec_B = np.asarray(fHatVec_B, dtype=float)
    r_ComB_B = np.asarray(r_ComB_B, dtype=float).reshape(3)
    forceBlock = fHatVec_B.T
    torqueBlock = np.cross(rVec_B - r_ComB_B.reshape(1, 3), fHatVec_B, axis=1).T
    return np.vstack((forceBlock, torqueBlock))


class JointThrAllocation(sysModel.SysModel):
    """
    Allocate thruster forces and joint angles for thrusters mounted on arms.

    This implementation is intentionally example-oriented with explicit assumptions:

    - Arms are serial chains packed in arm order.
    - Arm geometry/config comes from ``THRArmConfigMsgPayload``.
    - One spacecraft tree is supported (all arms in same kinematic tree).
    - Exactly one thruster per arm (configurable check).
    - Thruster parent joint is the last joint in each arm (configurable check).
    """

    def __init__(self):
        super().__init__()

        # Input messages
        self.armConfigInMsg = messaging.THRArmConfigMsgReader()
        self.hubStatesInMsg = messaging.SCStatesMsgReader()
        self.transForceInMsg = messaging.CmdForceInertialMsgReader()
        self.rotTorqueInMsg = messaging.CmdTorqueBodyMsgReader()
        self.jointStatesInMsgs = []

        # Output messages
        self.thrForceOutMsg = messaging.THRArrayCmdForceMsg()
        self.desJointAnglesOutMsg = messaging.JointArrayStateMsg()
        self.thrForcePayload = messaging.THRArrayCmdForceMsgPayload()
        self.jointAnglePayload = messaging.JointArrayStateMsgPayload()

        # Optimization result diagnostics
        self.solutionFound = 0
        self.bestErrInf = np.nan
        self.wrenchError = np.full(6, np.nan)
        self.costVal = np.nan

        # Cost settings
        self.Wc = np.eye(6)
        self.WfScale = 1e-6
        self.Wf = None
        self.Wtheta = None
        self.useThetaPenalty = False

        # Optimization settings
        self.maxiter = 3000
        self.ftol = 1e-10
        self.thrForceMax = 2.5  # [N]

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

    def addHingedJoint(self):
        """Add a joint-state input used by the optional joint-motion penalty."""
        self.jointStatesInMsgs.append(messaging.ScalarJointStateMsgReader())

    def validateInputMessages(self):
        """Raise ``BasiliskError`` if a required input message is not linked."""
        requiredInputMessages = [
            ("armConfigInMsg", self.armConfigInMsg),
            ("hubStatesInMsg", self.hubStatesInMsg),
            ("transForceInMsg", self.transForceInMsg),
            ("rotTorqueInMsg", self.rotTorqueInMsg),
        ]
        for msgName, msgReader in requiredInputMessages:
            if not msgReader.isLinked():
                self.bskLogger.error(f"JointThrAllocation.{msgName} was not linked.")

    def setWf(self, wfIn):
        """
        Set thrust-weight term for the cost function.

        Accepted inputs are:

        - scalar: applies same weight to all thrusters
        - vector length ``nThr``: per-thruster weights
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

        Accepted inputs are:

        - scalar: ``wc * I6``
        - length-6 vector: ``diag(wc)``
        - 6-by-6 matrix
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

    def setWtheta(self, wThetaIn):
        """
        Configure the optional joint-angle deviation penalty.

        Validation occurs in ``resolveWtheta()`` during ``Reset()``, after the
        number of joints is known. All weights must be finite. Scalar and
        vector weights must be nonnegative; a matrix must be symmetric positive
        semidefinite to floating-point roundoff.

        :param wThetaIn: Scalar weight, length-nJoint vector of diagonal weights,
            or nJoint-by-nJoint weighting matrix. Zero weights are allowed.
        """
        self.Wtheta = wThetaIn
        self.useThetaPenalty = True

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
            raise ValueError(
                f"Wf vector length {wfArr.size} does not match nThr {self.nThr}."
            )
        self.Wf = wfArr.copy()

    def resolveWtheta(self):
        """
        Validate and resolve Wtheta to an nJoint-by-nJoint weighting matrix.

        Matrix symmetry and eigenvalue checks use a tolerance of
        ``10 * nJoint * numpy.finfo(float).eps`` after scaling by the largest
        absolute matrix entry. Negative diagonal entries are always rejected.
        Roundoff-level asymmetry is removed by averaging unequal transpose
        entries; already symmetric entries are preserved. Negative eigenvalues
        within the tolerance are projected to zero. Zero and singular
        positive-semidefinite matrices are accepted.

        :raises ValueError: If the shape is invalid, an entry is non-finite,
            a scalar/vector weight or matrix diagonal entry is negative, or a
            matrix is asymmetric or has a negative eigenvalue beyond the
            roundoff tolerance.
        """
        wThetaArr = np.asarray(self.Wtheta, dtype=float)
        if not np.all(np.isfinite(wThetaArr)):
            raise ValueError("Wtheta weights must be finite.")
        if wThetaArr.ndim == 0:
            if wThetaArr < 0.0:
                raise ValueError("Wtheta scalar weight must be nonnegative.")
            self.Wtheta = float(wThetaArr) * np.eye(self.nJoint)
            return
        if wThetaArr.ndim == 1 and wThetaArr.size == self.nJoint:
            if np.any(wThetaArr < 0.0):
                raise ValueError("Wtheta vector weights must be nonnegative.")
            self.Wtheta = np.diag(wThetaArr)
            return
        if wThetaArr.shape == (self.nJoint, self.nJoint):
            if np.any(np.diag(wThetaArr) < 0.0):
                raise ValueError(
                    "Wtheta matrix must be positive semidefinite: "
                    "diagonal entries must be nonnegative."
                )
            resolved_weights = wThetaArr.copy()
            # Scaling keeps validation relative to the weights and avoids
            # overflow when checking large but finite matrix entries.
            weight_scale = np.max(np.abs(wThetaArr), initial=0.0)
            if weight_scale > 0.0:
                scaled_weights = wThetaArr / weight_scale
                roundoff_tol = 10.0 * self.nJoint * np.finfo(float).eps
                if not np.allclose(
                    scaled_weights, scaled_weights.T, rtol=0.0, atol=roundoff_tol
                ):
                    raise ValueError("Wtheta matrix must be symmetric.")
                unequal_entries = wThetaArr != wThetaArr.T
                resolved_weights[unequal_entries] = (
                    0.5 * wThetaArr[unequal_entries]
                    + 0.5 * wThetaArr.T[unequal_entries]
                )
                eigenvalues, eigenvectors = np.linalg.eigh(resolved_weights / weight_scale)
                if np.any(eigenvalues < -roundoff_tol):
                    raise ValueError("Wtheta matrix must be positive semidefinite.")
                if np.any(eigenvalues < 0.0):
                    projected_weights = (
                        eigenvectors * np.maximum(eigenvalues, 0.0)
                    ) @ eigenvectors.T
                    projected_weights = 0.5 * projected_weights + 0.5 * projected_weights.T
                    # Preserve finiteness if projection raises an entry just
                    # above the original scale near the floating-point limit.
                    projected_weights /= max(1.0, np.max(np.abs(projected_weights)))
                    resolved_weights = projected_weights * weight_scale
            self.Wtheta = resolved_weights
            return
        raise ValueError(
            "setWtheta expects scalar, length-nJoint vector, or nJoint x nJoint matrix."
        )

    def setThrForceMax(self, thrForceMaxIn):
        """
        Set thrust upper bounds.

        Accepted inputs are:

        - scalar: same upper bound for all thrusters
        - vector length ``nThr``: per-thruster upper bounds
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
            raise ValueError(
                f"thrForceMax vector length {thrForceMaxArr.size} does not match nThr {self.nThr}."
            )
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

        self.hubMass = float(cfgMsg.hubMass)
        self.r_BcB_B = np.asarray(cfgMsg.r_BcB_B, dtype=float).reshape(3)
        self.bodyArmIdx = np.asarray(cfgMsg.bodyArmIdx, dtype=int)
        self.bodyJointIdx = np.asarray(cfgMsg.bodyJointIdx, dtype=int)
        self.bodyMass = np.asarray(cfgMsg.bodyMass, dtype=float)
        self.r_LcP_P = np.asarray(cfgMsg.r_LcP_P, dtype=float).reshape(-1, 3)

        # Convert each flat 9-entry block from column-major to 3x3 matrix.
        self.dcm_C0P = np.array(
            [dcmFlat.reshape(3, 3, order="F") for dcmFlat in self.dcm_C0P], dtype=float
        )

    def initialGuesses(self):
        nDecision = self.nJoint + self.nThr
        seedList = []

        guess = np.zeros(nDecision)
        guess[self.nJoint :] = 1.0  # [N]
        seedList.append(guess)

        for angleSeed in (np.pi / 4.0, -np.pi / 4.0, np.pi / 2.0, -np.pi / 2.0):
            guess = np.zeros(nDecision)
            guess[: self.nJoint] = angleSeed
            guess[self.nJoint :] = 1.0  # [N]
            seedList.append(guess)

        self.x0 = np.vstack(seedList)

    def bounds(self):
        nDecision = self.nJoint + self.nThr
        lowerBound = np.zeros(nDecision)
        upperBound = np.zeros(nDecision)
        lowerBound[: self.nJoint] = -np.pi  # [rad]
        upperBound[: self.nJoint] = np.pi  # [rad]
        lowerBound[self.nJoint :] = 0.0  # [N]
        upperBound[self.nJoint :] = self.resolveThrForceMax()
        return tuple(
            (float(low), float(high)) for low, high in zip(lowerBound, upperBound)
        )

    def jointPoseFromTheta(self, theta: np.ndarray):
        """
        Return joint frame poses in body-frame coordinates.

        :param theta: Joint angle vector.
        :return: Direction cosine matrices and joint-frame origins in body-frame
            coordinates.
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

                axis_P = self.sHat_P[jointFlatIdx]
                axisNorm = np.linalg.norm(axis_P)
                if axisNorm <= 1e-12:  # [-]
                    raise ValueError("Joint axis has near-zero norm.")
                axis_P = axis_P / axisNorm
                dcm_CC0 = rbk.PRV2C(theta[jointFlatIdx] * axis_P)
                dcm_CP = dcm_CC0 @ self.dcm_C0P[jointFlatIdx]

                r_CB_B[jointFlatIdx] = r_PB_B + dcm_PB.T @ self.r_CP_P[jointFlatIdx]
                dcm_CB[jointFlatIdx] = dcm_CP @ dcm_PB

        return dcm_CB, r_CB_B

    def computeComFromTheta(self, dcm_CB, r_CB_B):
        """
        Compute the position of the system CoM relative to the body frame origin.

        :param dcm_CB: List of direction cosine matrices for each joint.
        :param r_CB_B: List of joint-frame origins in body-frame coordinates.
        :return: Position vector of the system CoM relative to the body frame origin.
        """
        totalMass = self.hubMass + np.sum(self.bodyMass)  # [kg]
        if totalMass <= 0.0:
            raise ValueError("Total spacecraft mass must be positive.")
        comNumerator = self.hubMass * self.r_BcB_B  # [kg*m]

        for bodyIdx in range(self.bodyMass.size):
            armIdx = int(self.bodyArmIdx[bodyIdx])
            jointLocalIdx = int(self.bodyJointIdx[bodyIdx])
            jointFlatIdx = int(self.armJointStart[armIdx] + jointLocalIdx)

            comNumerator += self.bodyMass[bodyIdx] * (
                r_CB_B[jointFlatIdx] + dcm_CB[jointFlatIdx].T @ self.r_LcP_P[bodyIdx]
            )  # [kg*m]

        return comNumerator / totalMass  # [m]

    def mapping(self, theta: np.ndarray,):
        """
        Compute the thruster force-to-wrench map for given joint angles.

        :param theta: Joint angle vector.
        :return: Thruster force-to-wrench map for the given joint angles.
        """
        dcm_CB, r_CB_B = self.jointPoseFromTheta(theta)
        r_ComB_B = self.computeComFromTheta(dcm_CB, r_CB_B)

        r_TB_B = np.zeros((self.nThr, 3))
        fHatVec_B = np.zeros((self.nThr, 3))

        for thrIdx in range(self.nThr):
            armIdx = int(self.thrArmIdx[thrIdx])
            jointLocalIdx = int(self.thrArmJointIdx[thrIdx])
            jointFlatIdx = int(self.armJointStart[armIdx] + jointLocalIdx)

            r_TB_B[thrIdx] = (
                r_CB_B[jointFlatIdx] + dcm_CB[jointFlatIdx].T @ self.r_TP_P[thrIdx]
            )
            fHatVec_B[thrIdx] = dcm_CB[jointFlatIdx].T @ self.fHat_P[thrIdx]

        return mapMatrix(r_TB_B, fHatVec_B, r_ComB_B)

    def cost(
        self,
        decisionVar: np.ndarray,
        desiredWrench_B: np.ndarray,
        currentJointAngles: np.ndarray = None,
    ) -> float:
        """
        Compute the cost function for the given decision variables and desired wrench.

        :param decisionVar: Concatenated vector of joint angles and thruster forces.
        :param desiredWrench_B: Desired force and torque in body-frame coordinates.
        :param currentJointAngles: Current joint angles when using the joint-motion
            penalty.
        :return: Cost function value.
        """
        theta = decisionVar[: self.nJoint]
        thrForces = decisionVar[self.nJoint:]
        wrenchMap = self.mapping(theta)
        wrenchError = desiredWrench_B - wrenchMap @ thrForces
        costValue = wrenchError.T @ self.Wc @ wrenchError + self.Wf.T @ thrForces
        if self.useThetaPenalty:
            if currentJointAngles is None:
                raise ValueError("currentJointAngles is required when using Wtheta.")
            deltaTheta = wrapAngle(theta - np.asarray(currentJointAngles))
            motion_cost = deltaTheta.T @ self.Wtheta @ deltaTheta
            if np.isinf(motion_cost):
                # Either sign of overflow must rule out the candidate.
                motion_cost = np.inf
            # Cancellation near a null direction can leave a negative residual
            # even after PSD projection. NaN still propagates through maximum.
            costValue += np.maximum(motion_cost, 0.0)
        return float(costValue)

    def Reset(self, CurrentSimNanos):
        """
        Reinitialize configuration and clear allocation outputs and diagnostics.

        :param CurrentSimNanos: Current simulation time [ns].
        """
        self.validateInputMessages()
        self.parseArmConfig()
        self.resolveWf()
        if self.useThetaPenalty:
            self.resolveWtheta()
        self.initialGuesses()

        self.solutionFound = 0
        self.bestErrInf = np.nan
        self.wrenchError = np.full(6, np.nan)
        self.costVal = np.nan

        self.desJointAnglesOutMsg.write(messaging.JointArrayStateMsgPayload())
        self.thrForceOutMsg.write(messaging.THRArrayCmdForceMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        minimize = _get_optimizer()

        hubStates = self.hubStatesInMsg()
        sigmaBN = np.array(hubStates.sigma_BN).reshape(3)
        dcm_BN = rbk.MRP2C(sigmaBN)

        forceInertialMsg = self.transForceInMsg()
        torqueBodyMsg = self.rotTorqueInMsg()

        desiredForce_N = np.array(forceInertialMsg.forceRequestInertial).reshape(3)
        desiredForce_B = dcm_BN @ desiredForce_N
        desiredTorque_B = np.array(torqueBodyMsg.torqueRequestBody).reshape(3)
        desiredWrench_B = np.hstack((desiredForce_B, desiredTorque_B))

        optOptions = {"maxiter": self.maxiter, "ftol": self.ftol, "disp": False}
        boundTuple = self.bounds()

        currentJointAngles = None
        if self.useThetaPenalty:
            if len(self.jointStatesInMsgs) != self.nJoint:
                raise ValueError(
                    "The number of joint-state inputs must match the number of joints."
                )
            currentJointAngles = np.array(
                [jointStateInMsg().state for jointStateInMsg in self.jointStatesInMsgs]
            )

        bestDecision = None
        bestCost = np.inf

        for initialDecision in self.x0:
            optResult = minimize(
                fun=lambda decision: self.cost(
                    decision, desiredWrench_B, currentJointAngles
                ),
                x0=initialDecision,
                bounds=boundTuple,
                method="SLSQP",
                options=optOptions,
            )
            if not optResult.success:
                continue

            decisionOpt = optResult.x
            costOpt = self.cost(decisionOpt, desiredWrench_B, currentJointAngles)
            if costOpt < bestCost:
                bestCost = costOpt
                bestDecision = decisionOpt

        if bestDecision is None:
            self.solutionFound = 0
            self.bestErrInf = np.inf
            self.wrenchError = desiredWrench_B.copy()
            self.costVal = np.nan
            bestDecision = np.zeros(self.nJoint + self.nThr)
            if currentJointAngles is not None:
                bestDecision[: self.nJoint] = currentJointAngles
        else:
            self.solutionFound = 1
            self.wrenchError = (
                desiredWrench_B
                - self.mapping(bestDecision[: self.nJoint])
                @ bestDecision[self.nJoint :]
            )
            self.bestErrInf = float(np.linalg.norm(self.wrenchError, ord=np.inf))
            self.costVal = bestCost

        self.thrForcePayload.thrForce = bestDecision[self.nJoint :].tolist()
        self.thrForceOutMsg.write(self.thrForcePayload, CurrentSimNanos, self.moduleID)

        self.jointAnglePayload.states.clear()
        self.jointAnglePayload.stateDots.clear()
        self.jointAnglePayload.stateDDots.clear()
        for angleCmd in bestDecision[: self.nJoint]:
            self.jointAnglePayload.states.push_back(float(angleCmd))
            self.jointAnglePayload.stateDots.push_back(0.0)  # [rad/s]
            self.jointAnglePayload.stateDDots.push_back(0.0)  # [rad/s^2]
        self.desJointAnglesOutMsg.write(
            self.jointAnglePayload, CurrentSimNanos, self.moduleID
        )
