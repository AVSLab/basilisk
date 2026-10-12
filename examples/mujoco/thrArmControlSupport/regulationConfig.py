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

"""Configuration data for the thruster-arm regulation scenario."""

import os
import numpy as np


class RegulationConfig:
    """Store the spacecraft, actuator, and controller settings for regulation."""

    def __init__(self):
        # Scenario identity and the MuJoCo model selected for this case.
        self.controlObjective = "regulation"
        self.xmlPath = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "sat_w_thruster_arms_regulation.xml"
        ))

        # Integrator accuracy settings applied by the shared dynamics model.
        self.integratorRelativeTolerance = None  # [-]
        self.integratorAbsoluteTolerance = None  # [-]

        # Spacecraft mass properties and articulated-system dimensions.
        self.numJoints = 8
        self.numThrusters = 2
        self.hubMass = 330.0  # [kg]
        self.vehicleMass = 350.0  # [kg]
        self.hubHalfLengths = [0.79, 0.69, 0.52]  # [m]

        # FSW, dynamics, and outer-loop timing.
        self.fswTimeStep = 0.01  # [s]
        self.dynTimeStep = 0.01  # [s]
        self.outerLoopPeriod = 10.0  # [s]

        # Fixed regulation references; and event tolerances.
        self.transRefPosition = [0.0, 0.0, 0.0]  # [m]
        self.transRefVelocity = [0.0, 0.0, 0.0]  # [m/s]
        self.transRefAcceleration = [0.0, 0.0, 0.0]  # [m/s^2]
        self.attRefSigma_R0N = [0.0, 0.0, 0.0]  # [-]
        self.jointPositionTolerance = 1e-3  # [rad]
        self.jointRateTolerance = 1e-3  # [rad/s]
        self.thrusterZeroTolerance = 1e-12  # [N]

        # Thruster pulse timing, actuator capability, allocation weighting, and optional FSW features.
        self.pulseResolution = 0.01  # [s]
        self.minimumOnTime = 0.01  # [s], uses pulse resolution when unset
        self.thrForces = [2.5] * self.numThrusters  # [N]
        self.maxJointTorques = [5.0] * self.numJoints  # [N*m]
        self.controlWeight = 1.0  # [-]
        self.thrustWeight = 1e-6  # [-]
        self.thetaWeight = None   # [-]
        self.useThetaAllocPenalty = False
        self.allocationMaxIterations = None  # [-], use allocator default when unset
        self.allocationFunctionTolerance = None  # [-], use allocator default when unset
        self.useJointMotionCompensation = True
        self.useJointProfiler = False
        self.jointProfileType = None
        self.jointProfileCutoffFrequency = None
        self.jointProfileFilterTimeStep = None
        self.jointProfileDuration = None
        self.jointProfileUseShortestPath = True
        self.jointProfileWrapStart = -np.pi  # [rad], None leaves output angles unwrapped

        # MuJoCo body, joint, and actuator names used by the shared models.
        self.hubName = "hub"
        self.jointRefs = [
            ("arm_1_base", "arm_1_joint_1"),
            ("arm_1_base", "arm_1_joint_2"),
            ("arm_1_tip", "arm_1_joint_3"),
            ("arm_1_tip", "arm_1_joint_4"),
            ("arm_2_base", "arm_2_joint_1"),
            ("arm_2_base", "arm_2_joint_2"),
            ("arm_2_tip", "arm_2_joint_3"),
            ("arm_2_tip", "arm_2_joint_4"),
        ]
        self.jointActNames = [
            "u_arm_1_joint_1", "u_arm_1_joint_2", "u_arm_1_joint_3", "u_arm_1_joint_4",
            "u_arm_2_joint_1", "u_arm_2_joint_2", "u_arm_2_joint_3", "u_arm_2_joint_4",
        ]
        self.thrActNames = ["F_thruster_1", "F_thruster_2"]
        self.bodyActNames = ["tau_prescribed_1", "tau_prescribed_2", "tau_prescribed_3"]

        # Regulation feedback gains.
        hubHalfLengthX, hubHalfLengthY, _ = self.hubHalfLengths  # [m]
        hubInertiaZ = (
            self.hubMass * (hubHalfLengthX ** 2 + hubHalfLengthY ** 2) / 3.0
        )  # [kg*m^2]
        self.kiRot = -1.0  # [N*m/s]

        translationTimeConstant = 25.0  # [s]
        attitudeTimeConstant = 15.0  # [s]
        self.pTrans = (
            2.0 * self.vehicleMass / translationTimeConstant * np.eye(3)
        ).flatten().tolist()  # [N*s/m]
        self.kTrans = (
            (2.0 * self.vehicleMass / translationTimeConstant) ** 2
            / (4.0 * self.vehicleMass) * np.eye(3)
        ).flatten().tolist()  # [N/m]
        self.pRot = 2.0 * hubInertiaZ / attitudeTimeConstant  # [N*m*s]
        self.kRot = self.pRot ** 2 / (4.0 * hubInertiaZ)  # [N*m]
        self.kTheta = (
            np.pi ** 2 * np.eye(self.numJoints)
        ).flatten().tolist()  # [N*m/rad]
        self.pTheta = (
            2.0 * 1.5 * np.pi * np.eye(self.numJoints)
        ).flatten().tolist()  # [N*m*s/rad]

        # Thruster-arm mass, joint-tree, and geometry data for the allocator.
        self.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
        self.bodyArmIdx = [0, 0, 1, 1]
        self.bodyJointIdx = [1, 3, 1, 3]
        self.bodyMass = [8.0, 2.0, 8.0, 2.0]  # [kg]
        self.r_LcP_P = [
            0.5, 0.0, 0.0,
            0.05, 0.0, 0.0,
            0.5, 0.0, 0.0,
            0.05, 0.0, 0.0,
        ]  # [m]
        self.thrArmIdx = [0, 1]
        self.thrArmJointIdx = [3, 3]
        self.armTreeIdx = [0, 0]
        self.armJointCount = [4, 4]
        self.r_CP_P = [
            0.79, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            -0.79, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]  # [m]
        self.r_TP_P = [
            0.1, 0.0, 0.0,
            0.1, 0.0, 0.0,
        ]  # [m]
        self.shat_P = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
        ]
        self.fhat_P = [
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
        self.dcm_C0P = (
            identity + identity + identity + identity
            + arm2BaseDcm + identity + identity + identity
        )
