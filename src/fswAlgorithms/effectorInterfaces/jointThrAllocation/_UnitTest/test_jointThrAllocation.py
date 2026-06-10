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
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.fswAlgorithms import jointThrAllocation
from Basilisk.utilities import RigidBodyKinematics as rbk


def _linked_input_messages():
    armConfigMsg = messaging.THRArmConfigMsg().write(messaging.THRArmConfigMsgPayload())
    hubStatesMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    transForceMsg = messaging.CmdForceInertialMsg().write(messaging.CmdForceInertialMsgPayload())
    rotTorqueMsg = messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    return {
        "armConfigInMsg": armConfigMsg,
        "hubStatesInMsg": hubStatesMsg,
        "transForceInMsg": transForceMsg,
        "rotTorqueInMsg": rotTorqueMsg,
    }


def _configured_allocation():
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.nArms = 2
    allocation.nThr = 2
    allocation.nJoint = 8
    allocation.armJointCount = np.array([4, 4], dtype=int)
    allocation.armJointStart = np.array([0, 4], dtype=int)
    allocation.thrArmIdx = np.array([0, 1], dtype=int)
    allocation.thrArmJointIdx = np.array([3, 3], dtype=int)
    allocation.r_CP_P = np.array(
        [
            [0.79, 0.0, 0.0],  # [m]
            [0.0, 0.0, 0.0],  # [m]
            [1.0, 0.0, 0.0],  # [m]
            [0.0, 0.0, 0.0],  # [m]
            [-0.79, 0.0, 0.0],  # [m]
            [0.0, 0.0, 0.0],  # [m]
            [1.0, 0.0, 0.0],  # [m]
            [0.0, 0.0, 0.0],  # [m]
        ]
    )
    allocation.r_TP_P = np.array(
        [
            [0.1, 0.0, 0.0],  # [m]
            [0.1, 0.0, 0.0],  # [m]
        ]
    )
    allocation.sHat_P = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ]
    )
    allocation.fHat_P = np.array(
        [
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    arm2_base_dcm = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    allocation.dcm_C0P = np.array(
        [
            np.eye(3),
            np.eye(3),
            np.eye(3),
            np.eye(3),
            arm2_base_dcm,
            np.eye(3),
            np.eye(3),
            np.eye(3),
        ]
    )
    allocation.hubMass = 980.0  # [kg]
    allocation.r_BcB_B = np.array([0.0, 0.0, 0.0])  # [m]
    allocation.bodyArmIdx = np.array([0, 0, 1, 1], dtype=int)
    allocation.bodyJointIdx = np.array([1, 3, 1, 3], dtype=int)
    allocation.bodyMass = np.array([8.0, 2.0, 8.0, 2.0])  # [kg]
    allocation.r_LcP_P = np.array(
        [
            [0.5, 0.0, 0.0],  # [m]
            [0.05, 0.0, 0.0],  # [m]
            [0.5, 0.0, 0.0],  # [m]
            [0.05, 0.0, 0.0],  # [m]
        ]
    )
    return allocation


def _expected_four_joint_layout(theta):
    theta = np.asarray(theta, dtype=float)
    arm2_base_dcm = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    def rot_x(angle):
        return rbk.PRV2C(np.array([angle, 0.0, 0.0]))

    def rot_y(angle):
        return rbk.PRV2C(np.array([0.0, angle, 0.0]))

    def rot_z(angle):
        return rbk.PRV2C(np.array([0.0, 0.0, angle]))

    dcm_CB = [np.eye(3) for _ in range(8)]
    r_CB_B = [np.zeros(3) for _ in range(8)]

    dcm_CB[0] = rot_x(theta[0])
    r_CB_B[0] = np.array([0.79, 0.0, 0.0])  # [m]
    dcm_CB[1] = rot_y(theta[1]) @ dcm_CB[0]
    r_CB_B[1] = r_CB_B[0].copy()
    dcm_CB[2] = rot_z(theta[2]) @ dcm_CB[1]
    r_CB_B[2] = r_CB_B[1] + dcm_CB[1].T @ np.array([1.0, 0.0, 0.0])  # [m]
    dcm_CB[3] = rot_y(theta[3]) @ dcm_CB[2]
    r_CB_B[3] = r_CB_B[2].copy()

    dcm_CB[4] = rot_x(theta[4]) @ arm2_base_dcm
    r_CB_B[4] = np.array([-0.79, 0.0, 0.0])  # [m]
    dcm_CB[5] = rot_y(theta[5]) @ dcm_CB[4]
    r_CB_B[5] = r_CB_B[4].copy()
    dcm_CB[6] = rot_z(theta[6]) @ dcm_CB[5]
    r_CB_B[6] = r_CB_B[5] + dcm_CB[5].T @ np.array([1.0, 0.0, 0.0])  # [m]
    dcm_CB[7] = rot_y(theta[7]) @ dcm_CB[6]
    r_CB_B[7] = r_CB_B[6].copy()

    boom_mass = 8.0  # [kg]
    tip_mass = 2.0  # [kg]
    hub_mass = 980.0  # [kg]
    boom_com_p = np.array([0.5, 0.0, 0.0])  # [m]
    tip_com_p = np.array([0.05, 0.0, 0.0])  # [m]

    com_numerator = np.zeros(3)
    com_numerator += boom_mass * (r_CB_B[1] + dcm_CB[1].T @ boom_com_p)  # [kg*m]
    com_numerator += tip_mass * (r_CB_B[3] + dcm_CB[3].T @ tip_com_p)  # [kg*m]
    com_numerator += boom_mass * (r_CB_B[5] + dcm_CB[5].T @ boom_com_p)  # [kg*m]
    com_numerator += tip_mass * (r_CB_B[7] + dcm_CB[7].T @ tip_com_p)  # [kg*m]
    r_ComB_B = com_numerator / (hub_mass + 2.0 * (boom_mass + tip_mass))  # [m]

    return dcm_CB, r_CB_B, r_ComB_B


def test_map_matrix():
    """
    **Validation Test Description**

    This unit test verifies that :func:`jointThrAllocation.mapMatrix` maps
    thruster force magnitudes into the expected stacked force and torque
    wrench.

    **Description of Variables Being Tested**

    This unit test checks the force block and torque block of the returned
    wrench mapping matrix.
    """
    rVec_B = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # [m]
    fHatVec_B = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    r_ComB_B = np.array([0.0, 0.0, 0.0])  # [m]

    mapping = jointThrAllocation.mapMatrix(rVec_B, fHatVec_B, r_ComB_B)
    expected = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [1.0, 0.0],
        ]
    )

    np.testing.assert_allclose(mapping, expected)

@pytest.mark.parametrize(
    "theta",
    [
        np.zeros(8),  # [rad]
        np.array([0.0, np.pi / 2.0, 0.0, 0.0, 0.0, -np.pi / 2.0, 0.0, 0.0]),  # [rad]
        np.array([np.pi / 6.0, -np.pi / 4.0, np.pi / 3.0, -np.pi / 6.0,
                  -np.pi / 5.0, np.pi / 7.0, -np.pi / 8.0, np.pi / 9.0]),  # [rad]
    ],
)
def test_spacecraft_layout(theta):
    """
    **Validation Test Description**

    This unit test verifies that :func:`jointThrAllocation.jointPoseFromTheta`
    computes the correct joint frame poses and vectors for a given set of
    joint angles. It also verifies that :func:`jointThrAllocation.computeComFromTheta`
    computes the correct center of mass vector for a given set of joint angles.

    **Description of Variables Being Tested**

    This unit test checks the DCM and position vectors for each joint. It
    also checks the computed center of mass vector.
    """
    allocation = _configured_allocation()

    dcm_CB, r_CB_B = allocation.jointPoseFromTheta(theta)
    r_ComB_B = allocation.computeComFromTheta(dcm_CB, r_CB_B)
    expected_dcm, expected_r_CB_B, expected_r_ComB_B = _expected_four_joint_layout(theta)

    for dcm_actual, dcm_expected in zip(dcm_CB, expected_dcm):
        np.testing.assert_allclose(dcm_actual, dcm_expected)
    for pos_actual, pos_expected in zip(r_CB_B, expected_r_CB_B):
        np.testing.assert_allclose(pos_actual, pos_expected)
    np.testing.assert_allclose(r_ComB_B, expected_r_ComB_B)


def test_spacecraft_layout_includes_hub_com_offset():
    """
    **Validation Test Description**

    This unit test verifies that :func:`jointThrAllocation.computeComFromTheta`
    includes the hub center-of-mass offset contribution when computing the
    system center of mass.

    **Description of Variables Being Tested**

    This unit test checks the computed center-of-mass vector for a nonzero hub
    center-of-mass offset.
    """
    allocation = _configured_allocation()
    allocation.r_BcB_B = np.array([0.12, -0.04, 0.03])  # [m]
    theta = np.zeros(allocation.nJoint)  # [rad]

    dcm_CB, r_CB_B = allocation.jointPoseFromTheta(theta)
    r_ComB_B = allocation.computeComFromTheta(dcm_CB, r_CB_B)

    total_mass = allocation.hubMass + np.sum(allocation.bodyMass)  # [kg]
    expected_r_ComB_B = allocation.hubMass * allocation.r_BcB_B  # [kg*m]
    for body_idx in range(allocation.bodyMass.size):
        arm_idx = int(allocation.bodyArmIdx[body_idx])
        joint_local_idx = int(allocation.bodyJointIdx[body_idx])
        joint_flat_idx = int(allocation.armJointStart[arm_idx] + joint_local_idx)
        expected_r_ComB_B += allocation.bodyMass[body_idx] * (
            r_CB_B[joint_flat_idx] + dcm_CB[joint_flat_idx].T @ allocation.r_LcP_P[body_idx]
        )  # [kg*m]
    expected_r_ComB_B = expected_r_ComB_B / total_mass  # [m]

    np.testing.assert_allclose(r_ComB_B, expected_r_ComB_B)


def test_spacecraft_layout_asymmetric_body_masses_shift_com():
    """
    **Validation Test Description**

    This unit test verifies that :func:`jointThrAllocation.computeComFromTheta`
    responds correctly to asymmetric arm-body masses by shifting the system
    center of mass toward the heavier side.

    **Description of Variables Being Tested**

    This unit test checks the computed center-of-mass vector for an asymmetric
    set of arm-body masses.
    """
    allocation = _configured_allocation()
    allocation.bodyMass = np.array([12.0, 3.0, 4.0, 1.0])  # [kg]
    theta = np.zeros(allocation.nJoint)  # [rad]

    dcm_CB, r_CB_B = allocation.jointPoseFromTheta(theta)
    r_ComB_B = allocation.computeComFromTheta(dcm_CB, r_CB_B)

    total_mass = allocation.hubMass + np.sum(allocation.bodyMass)  # [kg]
    expected_r_ComB_B = allocation.hubMass * allocation.r_BcB_B  # [kg*m]
    for body_idx in range(allocation.bodyMass.size):
        arm_idx = int(allocation.bodyArmIdx[body_idx])
        joint_local_idx = int(allocation.bodyJointIdx[body_idx])
        joint_flat_idx = int(allocation.armJointStart[arm_idx] + joint_local_idx)
        expected_r_ComB_B += allocation.bodyMass[body_idx] * (
            r_CB_B[joint_flat_idx] + dcm_CB[joint_flat_idx].T @ allocation.r_LcP_P[body_idx]
        )  # [kg*m]
    expected_r_ComB_B = expected_r_ComB_B / total_mass  # [m]

    np.testing.assert_allclose(r_ComB_B, expected_r_ComB_B)


def test_allocation_configuration_helpers():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointThrAllocation.JointThrAllocation`
    can be constructed from the standard ``Basilisk.fswAlgorithms`` import path
    and that the helper methods resolve scalar and vector configuration values.

    **Description of Variables Being Tested**

    This unit test checks the resolved thrust force bounds, thrust weights, and
    initial decision vector shape.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.nJoint = 2
    allocation.nThr = 2

    allocation.setThrForceMax(4.0)  # [N]
    np.testing.assert_allclose(allocation.resolveThrForceMax(), np.array([4.0, 4.0]))  # [N]

    allocation.setWf(1.0e-5)
    allocation.resolveWf()
    np.testing.assert_allclose(allocation.Wf, np.array([1.0e-5, 1.0e-5]))

    allocation.initialGuesses()
    assert allocation.x0.shape == (5, 4)

    bounds = allocation.bounds()
    assert bounds[0] == (-np.pi, np.pi)
    assert bounds[1] == (-np.pi, np.pi)
    assert bounds[2] == (0.0, 4.0)  # [N]
    assert bounds[3] == (0.0, 4.0)  # [N]


@pytest.mark.parametrize(
    "missing_msg_name",
    [
        "armConfigInMsg",
        "hubStatesInMsg",
        "transForceInMsg",
        "rotTorqueInMsg",
    ],
)
def test_reset_rejects_missing_input_message(missing_msg_name):
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointThrAllocation.JointThrAllocation`
    rejects reset calls when any required input message is not connected.

    **Description of Variables Being Tested**

    This unit test checks each required input message reader.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.bskLogger = bskLogging.BSKLogger()

    for msgName, msg in _linked_input_messages().items():
        if msgName != missing_msg_name:
            getattr(allocation, msgName).subscribeTo(msg)

    with pytest.raises(bskLogging.BasiliskError, match=f"JointThrAllocation.{missing_msg_name}"):
        allocation.Reset(0)
