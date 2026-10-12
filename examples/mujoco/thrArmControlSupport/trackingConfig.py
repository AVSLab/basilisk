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

"""Configuration data for the thruster-arm tracking scenario."""

import os

import numpy as np


class TrackingConfig:
    """Store the spacecraft, trajectory, actuator, and controller tracking settings."""

    def __init__(self):
        # Scenario identity and the MuJoCo model selected for this case.
        self.controlObjective = "tracking"
        self.xmlPath = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "sat_w_thruster_arms_tracking.xml"
        ))

        # Integrator accuracy settings applied by the shared dynamics model.
        self.integratorRelativeTolerance = None
        self.integratorAbsoluteTolerance = None

        # Spacecraft mass properties and articulated-system dimensions.
        self.numJoints = 8
        self.numThrusters = 2
        self.scMass = 1000.0  # [kg]
        self.hubMass = 980.0  # [kg]
        self.hubHalfLengths = [0.79, 0.69, 0.52]  # [m]

        # FSW, dynamics, and outer-loop timing.
        self.fswTimeStep = 0.01  # [s]
        self.dynTimeStep = 0.01  # [s]
        self.outerLoopPeriod = 10.0  # [s]

        # Translational tracking trajectory parameters.
        self.transRadius = 20.0  # [m]
        self.finalPosition = [0.0, -5.0, 0.0]  # [m]
        self.blendEndPosition = [0.0, -16.0, 0.0]  # [m]
        self.tCircle = 5.0 * 3600.0  # [s]
        self.tTransition = 1.0 * 3600.0  # [s]
        self.tLinear = 110.0 * 60.0  # [s]
        self.jointPositionTolerance = 1e-3  # [rad]
        self.jointRateTolerance = 1e-3  # [rad/s]
        self.thrusterZeroTolerance = 1e-12  # [N]

        # Attitude-pointing reference parameters for the moving spacecraft.
        self.pHat_B = [1.0, 0.0, 0.0]  # [-]
        self.scTargetPosition = [0.0, 0.0, 0.0]  # [m]
        self.pointingSmallAngle = None  # [rad], set to enable near-alignment handling
        self.useBoresightRateDamping = True

        # Thruster pulse timing, actuator capability, allocation weighting, and optional FSW features.
        self.pulseResolution = 0.01  # [s]
        self.minimumOnTime = 0.01  # [s], uses pulse resolution when unset
        self.thrForces = [0.25] * self.numThrusters  # [N]
        self.maxJointTorques = [2.5] * self.numJoints  # [N*m]
        self.maxPreTorques = None
        self.controlWeight = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # [-]
        self.thrustWeight = 1e-4  # [-]
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

        # Tracking feedback gains for translation, attitude, and joint motion.
        self.kRot = 1.75  # [N*m]
        self.pRot = 15.0  # [N*m*s]
        self.kiRot = -1.0  # [N*m/s]
        self.kTrans = (0.00002 * self.scMass * np.eye(3)).flatten().tolist()  # [N/m]
        self.pTrans = (0.05 * self.scMass * np.eye(3)).flatten().tolist()  # [N*s/m]
        self.kTheta = (22.0 * np.eye(self.numJoints)).flatten().tolist()  # [N*m/rad]
        self.pTheta = (14.0 * np.eye(self.numJoints)).flatten().tolist()  # [N*m*s/rad]

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
