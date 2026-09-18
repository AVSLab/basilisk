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

from types import SimpleNamespace

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


@pytest.fixture
def single_arm_allocation():
    """
    Connect a two-joint arm with an analytically known force-to-wrench map.

    Both hinges rotate about the body x axis. The thruster points along that
    axis and acts at the center of mass, so any pair of joint angles produces
    the same body-x force and zero torque. This makes the wrench-tracking and
    joint-motion contributions to the cost independently predictable.

    The hub attitude is zero, the requested force is 1 N along body x, and the
    requested torque is zero. Two writable joint-state messages are connected
    even when the optional motion penalty is disabled. Each test configures
    its weights before calling ``Reset()``.

    :return: Allocation module and a dictionary retaining its writable input
        messages. The ``jointStatesInMsgs`` entry contains the two joint-state
        messages in configuration order.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    input_messages = _linked_input_messages()
    config = messaging.THRArmConfigMsgPayload()
    config.hubMass = 1.0  # [kg]
    config_fields = {
        "armJointCount": [2],
        "thrArmIdx": [0],
        "thrArmJointIdx": [1],
        "r_CP_P": [0.0] * 6,  # [m]
        "r_TP_P": [0.0] * 3,  # [m]
        "r_BcB_B": [0.0] * 3,  # [m]
        "shat_P": [1.0, 0.0, 0.0] * 2,
        "fhat_P": [1.0, 0.0, 0.0],
        "dcm_C0P": np.tile(np.eye(3).flatten(), 2),
    }
    for name, values in config_fields.items():
        for value in values:
            getattr(config, name).push_back(value)
    input_messages["armConfigInMsg"].write(config)
    force = messaging.CmdForceInertialMsgPayload()
    force.forceRequestInertial = [1.0, 0.0, 0.0]  # [N]
    input_messages["transForceInMsg"].write(force)
    for name, message in input_messages.items():
        getattr(allocation, name).subscribeTo(message)

    joint_messages = []
    for _ in range(2):
        message = messaging.ScalarJointStateMsg().write(
            messaging.ScalarJointStateMsgPayload()
        )
        allocation.addHingedJoint()
        allocation.jointStatesInMsgs[-1].subscribeTo(message)
        joint_messages.append(message)
    input_messages["jointStatesInMsgs"] = joint_messages
    return allocation, input_messages


def _stub_optimizer(monkeypatch, decisions, successes):
    """
    Replace SciPy minimization with prescribed results for each optimizer seed.

    Each call evaluates the objective supplied by ``UpdateState()`` at the
    prescribed decision, then returns that decision and its success flag. The
    sequence repeats for subsequent updates. The real module cost, geometry,
    input readers, selection logic, and output writers remain in use; these
    tests validate allocation behavior rather than SciPy convergence.

    :param monkeypatch: Pytest fixture that restores the optimizer after the test.
    :param decisions: Candidate rows containing two angles [rad] and one force [N].
    :param successes: Mutable list of success flags, one per candidate.
    :return: List populated with each initial seed and its candidate's cost.
    """
    calls = []

    def minimize(fun, x0, **kwargs):
        index = len(calls) % len(decisions)
        decision = np.asarray(decisions[index], dtype=float)
        calls.append((x0.copy(), fun(decision)))
        return SimpleNamespace(success=successes[index], x=decision.copy())

    monkeypatch.setattr(jointThrAllocation, "_get_optimizer", lambda: minimize)
    return calls


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


def test_joint_motion_penalty_configuration():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointThrAllocation.JointThrAllocation`
    configures the optional joint-motion penalty and its joint-state readers.

    **Description of Variables Being Tested**

    This unit test checks scalar, vector, and matrix joint-motion weights and
    the optional joint-state message connections.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.nJoint = 2

    allocation.setWtheta(2.0)
    allocation.resolveWtheta()
    np.testing.assert_allclose(allocation.Wtheta, 2.0 * np.eye(2))

    allocation.setWtheta(np.array([1.0, 3.0]))
    allocation.resolveWtheta()
    np.testing.assert_allclose(allocation.Wtheta, np.diag([1.0, 3.0]))

    w_theta = np.array([[1.0, 0.5], [0.5, 2.0]])
    allocation.setWtheta(w_theta)
    allocation.resolveWtheta()
    np.testing.assert_allclose(allocation.Wtheta, w_theta)

    for i in range(allocation.nJoint):
        jointStateMsg = messaging.ScalarJointStateMsg().write(
            messaging.ScalarJointStateMsgPayload()
        )
        allocation.addHingedJoint()
        allocation.jointStatesInMsgs[i].subscribeTo(jointStateMsg)

    assert allocation.useThetaPenalty
    assert len(allocation.jointStatesInMsgs) == allocation.nJoint
    assert all(jointStateInMsg.isLinked() for jointStateInMsg in allocation.jointStatesInMsgs)


@pytest.mark.parametrize(
    "weight, error_message",
    [
        pytest.param(-1.0, "nonnegative", id="negative-scalar"),
        pytest.param(-1.0e-20, "nonnegative", id="small-negative-scalar"),
        pytest.param(np.nan, "finite", id="nan-scalar"),
        pytest.param(np.inf, "finite", id="infinite-scalar"),
        pytest.param(-np.inf, "finite", id="negative-infinite-scalar"),
        pytest.param([1.0, -1.0], "nonnegative", id="negative-vector-entry"),
        pytest.param([1.0, np.nan], "finite", id="nan-vector-entry"),
        pytest.param([1.0, np.inf], "finite", id="infinite-vector-entry"),
        pytest.param([[1.0, np.nan], [np.nan, 1.0]], "finite", id="nan-matrix-entry"),
        pytest.param([[1.0, np.inf], [np.inf, 1.0]], "finite", id="infinite-matrix-entry"),
        pytest.param([[1.0, 1.0], [0.0, 1.0]], "symmetric", id="asymmetric-matrix"),
        pytest.param([[1.0, 2.0], [2.0, 1.0]], "positive semidefinite", id="indefinite-matrix"),
        pytest.param(
            1.0e-20 * np.array([[1.0, 2.0], [2.0, 1.0]]),
            "positive semidefinite", id="small-indefinite-matrix",
        ),
        pytest.param(
            -1.0e-20 * np.eye(2), "positive semidefinite", id="small-negative-matrix",
        ),
        pytest.param(
            np.diag([1.0e15, -1.0]), "nonnegative", id="negative-diagonal-large-scale",
        ),
        pytest.param(
            np.diag([1.0, -np.nextafter(0.0, 1.0)]), "nonnegative",
            id="negative-subnormal-diagonal",
        ),
        pytest.param([1.0], "length-nJoint", id="wrong-vector-length"),
        pytest.param(np.ones((2, 3)), "nJoint x nJoint", id="wrong-matrix-shape"),
    ],
)
def test_reset_rejects_invalid_motion_weights(single_arm_allocation, weight, error_message):
    """
    **Validation Test Description**

    Configure an invalid motion weight on a connected two-joint allocation and
    call ``Reset()``. Validation must fail during initialization, before any
    optimizer can use a negative or non-finite motion penalty.

    **Test Parameter Discussion**

    ``weight`` covers negative scalar/vector entries, NaN and infinity in each
    supported input form, an asymmetric matrix, indefinite matrices, and wrong
    dimensions. The indefinite matrix has positive entries but eigenvalues
    -1 and 3, so checking entries alone is insufficient. Small negative and
    indefinite weights ensure that an absolute tolerance cannot hide invalid
    weights merely because their magnitude is small. Negative diagonals must
    also be rejected regardless of other entries' scale, even at the smallest
    representable magnitude. ``error_message`` names the expected failure.

    **Description of Variables Being Tested**

    ``Reset()`` must raise ``ValueError`` with a message identifying the failed
    condition. Required messages and arm geometry are valid, so each failure
    must originate from motion-weight validation.
    """
    allocation, _ = single_arm_allocation
    allocation.setWtheta(weight)

    with pytest.raises(ValueError, match=error_message):
        allocation.Reset(0)


@pytest.mark.parametrize(
    "weight, expected_matrix",
    [
        pytest.param(0.0, np.zeros((2, 2)), id="zero-scalar"),
        pytest.param(1.0e-20, 1.0e-20 * np.eye(2), id="small-positive-scalar"),
        pytest.param([0.0, 3.0], np.diag([0.0, 3.0]), id="zero-vector-entry"),
        pytest.param(np.zeros((2, 2)), np.zeros((2, 2)), id="zero-matrix"),
        pytest.param(
            np.nextafter(0.0, 1.0) * np.eye(2),
            np.nextafter(0.0, 1.0) * np.eye(2),
            id="smallest-positive-diagonal",
        ),
        pytest.param(
            np.nextafter(0.0, 1.0) * np.ones((2, 2)),
            np.nextafter(0.0, 1.0) * np.ones((2, 2)),
            id="smallest-positive-symmetric-entries",
        ),
        pytest.param(
            [[1.0, -1.0], [-1.0, 1.0]], [[1.0, -1.0], [-1.0, 1.0]],
            id="singular-matrix-negative-cross-terms",
        ),
        pytest.param(
            np.outer([0.1, 0.3], [0.1, 0.3]), np.outer([0.1, 0.3], [0.1, 0.3]),
            id="singular-matrix-roundoff",
        ),
        pytest.param(
            1.0e-20 * np.array([[1.0, -1.0], [-1.0, 1.0]]),
            1.0e-20 * np.array([[1.0, -1.0], [-1.0, 1.0]]),
            id="small-singular-matrix",
        ),
        pytest.param(
            1.0e308 * np.array([[1.0, -0.5], [-0.5, 1.0]]),
            1.0e308 * np.array([[1.0, -0.5], [-0.5, 1.0]]),
            id="large-finite-matrix",
        ),
        pytest.param(
            [[1.0, 0.5 + np.finfo(float).eps], [0.5, 1.0]],
            [[1.0, 0.5 + np.finfo(float).eps / 2.0],
             [0.5 + np.finfo(float).eps / 2.0, 1.0]],
            id="roundoff-asymmetry",
        ),
    ],
)
def test_reset_accepts_semidefinite_motion_weights(single_arm_allocation, weight, expected_matrix):
    """
    **Validation Test Description**

    Initialize the connected allocation with valid motion weights at the
    boundaries of the allowed domain. Zero and singular weights must remain
    usable, including matrices with negative off-diagonal entries. Validation
    must handle small and large finite scales without overflow or an arbitrary
    minimum weight.

    **Test Parameter Discussion**

    ``weight`` covers scalar/vector zeros, zero and singular matrices, a
    floating-point Gram matrix with a mathematically zero eigenvalue, extreme
    scales including the smallest positive float, and asymmetry of one machine
    epsilon. ``expected_matrix`` contains the resolved diagonal or full matrix.
    In the asymmetric case, the expected off-diagonal entries are the mean of
    the configured pair. Singular matrices may change within roundoff when
    negative computed eigenvalues are projected to zero.

    **Description of Variables Being Tested**

    ``Reset()`` must succeed, retain finite entries, and produce an exactly
    symmetric ``Wtheta`` matching the expected matrix. Comparison uses zero
    absolute tolerance so that small weights cannot pass after being erased.
    The caller's input array must remain unchanged.
    """
    allocation, _ = single_arm_allocation
    original_weight = np.asarray(weight).copy()
    allocation.setWtheta(weight)

    with np.errstate(over="raise", invalid="raise"):
        allocation.Reset(0)

    assert np.all(np.isfinite(allocation.Wtheta))
    np.testing.assert_array_equal(allocation.Wtheta, allocation.Wtheta.T)
    np.testing.assert_allclose(allocation.Wtheta, expected_matrix, rtol=1e-14, atol=0.0)
    np.testing.assert_array_equal(np.asarray(weight), original_weight)


@pytest.mark.parametrize(
    "weight, angle_error",
    [
        pytest.param(
            [[1.0e15, 1.0e8], [1.0e8, 9.0]], [-1.0e-7, 1.0],  # [rad]
            id="negative-mode-below-relative-tolerance",
        ),
        pytest.param(
            [[1.0, 1.0 + np.finfo(float).eps],
             [1.0 + np.finfo(float).eps, 1.0]], [1.0, -1.0],  # [rad]
            id="negative-mode-at-roundoff",
        ),
        pytest.param(
            [[1.0e16, 1.0e8], [1.0e8, 1.0]], [-1.0e-8, 1.0],  # [rad]
            id="singular-quadratic-cancellation",
        ),
    ],
)
def test_motion_penalty_is_nonnegative(single_arm_allocation, weight, angle_error):
    """
    **Validation Test Description**

    Resolve nearly positive-semidefinite weights and evaluate ``cost()`` near
    their null directions with zero requested wrench and zero thrust. There
    are no other cost contributions, so a negative result would reward motion.

    **Test Parameter Discussion**

    ``weight`` includes a matrix with a -1 eigenvalue hidden by a 1e15 scale,
    a matrix with a negative eigenvalue of roundoff magnitude, and a singular
    matrix susceptible to cancellation in the quadratic form. Negative modes
    within the documented relative tolerance must be projected out, rather
    than retained. ``angle_error`` gives the tested near-null direction [rad].

    **Description of Variables Being Tested**

    The resolved matrix must have no eigenvalue below -1e-12 in these cases.
    The actual motion cost must be finite, nonnegative, and at most 1e-12.
    The first input previously produced a cost of approximately -1 despite
    passing validation, while its projected penalty is approximately zero.
    """
    allocation, _ = single_arm_allocation
    allocation.setWtheta(weight)
    allocation.Reset(0)
    decision = np.array([*angle_error, 0.0])  # [rad, rad, N]

    cost = allocation.cost(decision, np.zeros(6), np.zeros(2))

    assert np.linalg.eigvalsh(allocation.Wtheta).min() >= -1e-12
    assert np.isfinite(cost)
    assert 0.0 <= cost <= 1e-12


def test_motion_penalty_preserves_nan(single_arm_allocation):
    """
    Verify that bounding motion cost below by zero does not hide invalid inputs.

    A NaN measured joint angle makes only the motion penalty invalid; the
    candidate angles, force-to-wrench map, requested wrench, and thrust are
    otherwise finite. ``cost()`` must remain NaN so the candidate cannot be
    mistaken for a finite, zero-cost allocation.
    """
    allocation, _ = single_arm_allocation
    allocation.setWtheta(1.0)
    allocation.Reset(0)

    cost = allocation.cost(np.zeros(3), np.zeros(6), np.array([np.nan, 0.0]))

    assert np.isnan(cost)


def test_motion_penalty_does_not_hide_overflow(single_arm_allocation):
    """
    Verify that the nonnegative cost bound cannot turn overflow into zero cost.

    This finite, nearly singular matrix and angle error have a positive motion
    penalty, but intermediate products can overflow to negative infinity.
    Resolve the weights and call the real cost with zero wrench and thrust.
    Depending on floating-point evaluation order, the result can be positive
    or NaN, but it must never be zero or negative. Either sign of infinity in
    the motion term must produce a positive-infinite cost rather than being
    clipped to zero and treated as a favorable allocation.
    """
    allocation, _ = single_arm_allocation
    allocation.setWtheta([
        [6.227709133842637e307, -7.891547020745441e307],
        [-7.891547020745441e307, 1.0e308],
    ])
    allocation.Reset(0)
    decision = np.array([-2.7992763360330044, -1.7941584448303567, 0.0])  # [rad, rad, N]

    with np.errstate(over="ignore", invalid="ignore"):
        cost = allocation.cost(decision, np.zeros(6), np.zeros(2))

    assert np.isnan(cost) or cost > 0.0


@pytest.mark.parametrize("extra_turns", [0, 3], ids=["boundary", "multiple-turns"])
@pytest.mark.parametrize(
    "weight, expected_penalty",
    [
        pytest.param(None, 0.0, id="disabled"),
        pytest.param(2.0, 0.10, id="scalar"),
        pytest.param([1.0, 3.0], 0.13, id="vector"),
        pytest.param([[1.0, 0.5], [0.5, 2.0]], 0.07, id="matrix"),
    ],
)
def test_cost_wraps_joint_motion(
    single_arm_allocation, weight, expected_penalty, extra_turns
):
    """
    **Validation Test Description**

    Call ``cost()`` directly for the axial-thruster fixture. A 0.5 N candidate
    tracks a 1 N request with a squared wrench error of 0.25. The thrust weight
    is one, giving a thrust-use term of 0.5 and a baseline cost of 0.75.

    Commanded and measured angles lie on opposite sides of the wrapping
    boundary. Their shortest signed differences are 0.1 rad and -0.2 rad.
    Expected penalties are calculated from those differences without calling
    the module's wrapping helper. This detects omitted penalties, unwrapped
    differences, and implementations that correct only one revolution.

    **Test Parameter Discussion**

    - ``weight`` selects the disabled, scalar, vector, or full-matrix case.
      ``None`` leaves the penalty disabled and passes no current angles.
    - ``expected_penalty`` is 0 for the disabled case; 0.10 for scalar weight 2;
      0.13 for diagonal weights [1, 3]; and 0.07 for matrix
      [[1, 0.5], [0.5, 2]]. The matrix case includes a cross term of -0.02.
    - ``extra_turns`` is either zero or three. Extra revolutions are added to
      the first measured angle and subtracted from the second; the expected
      wrapped errors and penalty remain unchanged.

    **Description of Variables Being Tested**

    The returned scalar cost must equal ``0.75 + expected_penalty`` within
    ``pytest.approx`` tolerances. The full-matrix case also checks that
    off-diagonal weights contribute to the cost.
    """
    allocation, _ = single_arm_allocation
    allocation.setWf(1.0)
    if weight is not None:
        allocation.setWtheta(weight)
    allocation.Reset(0)

    decision = np.array([-np.pi + 0.05, np.pi - 0.1, 0.5])  # [rad, rad, N]
    current_angles = np.array([np.pi - 0.05, -np.pi + 0.1])  # [rad]
    current_angles += extra_turns * 2.0 * np.pi * np.array([1.0, -1.0])  # [rad]
    desired_wrench = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [N, N, N, Nm, Nm, Nm]

    # Force tracking contributes (1 - 0.5)^2 and thrust use contributes 0.5.
    cost = allocation.cost(
        decision, desired_wrench, current_angles if weight is not None else None
    )
    assert cost == pytest.approx(0.75 + expected_penalty)


@pytest.mark.parametrize("lowest_cost_succeeds", [True, False])
def test_update_selects_minimum_total_cost(
    single_arm_allocation, monkeypatch, lowest_cost_succeeds
):
    """
    **Validation Test Description**

    Exercise ``UpdateState()`` with the real cost function and a deterministic
    optimizer stub. With a 1 N requested force and unit thrust weight, a
    candidate force ``f`` has numerical cost ``(1 - f)**2 + f``. The five
    prescribed forces [N] are [1, 1.5, 0.5, 0.75, 2], with costs
    [1, 1.75, 0.75, 0.8125, 3].

    The first candidate has zero wrench error, but a later candidate has lower
    total cost. The last candidate is also worse than the expected winner.
    This catches selection by wrench error, stopping at the first accurate
    result, and publishing the last result instead of the best result. The
    optimizer stub makes these checks independent of SciPy's convergence.

    **Test Parameter Discussion**

    ``lowest_cost_succeeds`` controls the success flag of the third candidate.
    If true, its 0.5 N force and 0.75 cost must win. If false, that candidate
    must be ignored and the fourth candidate must win with 0.75 N force and
    0.8125 cost. All other candidates are marked successful.

    **Description of Variables Being Tested**

    Check that all five configured seeds are visited in order and that every
    objective evaluation has the analytic cost. Verify the selected thrust
    and joint-angle messages, zero commanded joint rates and accelerations,
    ``solutionFound == 1``, and the selected ``costVal``. ``wrenchError`` and
    ``bestErrInf`` must describe the selected candidate's 0.5 N or 0.25 N
    residual, even though the first candidate had no residual.
    """
    allocation, _ = single_arm_allocation
    allocation.setWf(1.0)
    allocation.Reset(0)
    decisions = np.array([
        [0.0, 0.0, 1.0],  # [rad, rad, N]
        [0.1, -0.1, 1.5],  # [rad, rad, N]
        [0.2, -0.2, 0.5],  # [rad, rad, N]
        [0.3, -0.3, 0.75],  # [rad, rad, N]
        [0.4, -0.4, 2.0],  # [rad, rad, N]
    ])
    successes = [True, True, lowest_cost_succeeds, True, True]
    calls = _stub_optimizer(monkeypatch, decisions, successes)

    allocation.UpdateState(0)

    assert len(calls) == 5
    np.testing.assert_allclose([seed for seed, _ in calls], allocation.x0)
    np.testing.assert_allclose([cost for _, cost in calls], [1.0, 1.75, 0.75, 0.8125, 3.0])
    selected_index = 2 if lowest_cost_succeeds else 3
    expected_force = 0.5 if lowest_cost_succeeds else 0.75  # [N]
    expected_error = 1.0 - expected_force  # [N]
    assert allocation.thrForceOutMsg.read().thrForce[0] == pytest.approx(expected_force)
    joint_output = allocation.desJointAnglesOutMsg.read()
    np.testing.assert_allclose(joint_output.states, decisions[selected_index, :2])
    np.testing.assert_array_equal(joint_output.stateDots, np.zeros(2))
    np.testing.assert_array_equal(joint_output.stateDDots, np.zeros(2))
    assert allocation.solutionFound == 1
    assert allocation.costVal == pytest.approx(0.75 if lowest_cost_succeeds else 0.8125)
    assert allocation.bestErrInf == pytest.approx(expected_error)
    np.testing.assert_allclose(
        allocation.wrenchError, [expected_error, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def test_update_uses_current_joint_states(single_arm_allocation, monkeypatch):
    """
    **Validation Test Description**

    Run two updates with the same five prescribed optimizer candidates. Each
    produces the requested 1 N force and zero torque. Thrust-use weighting is
    zero and joint-motion weighting is the identity, so only motion cost can
    distinguish the candidates. Four candidates command zero joint angles;
    the third commands angles near the wrapping boundary.

    Initially, measured angles are on the opposite side of that boundary.
    The third candidate has wrapped errors [0.1, -0.2] rad and cost 0.05, so
    it must win. Rewrite both measured angles to zero before the second
    update. A zero-angle candidate must then win with zero cost. This detects
    stale or ignored joint-state inputs, missing wrapping in the objective,
    and omission of motion cost from solution selection.

    **Test Parameter Discussion**

    This test has one case containing two consecutive updates. The optimizer
    stub evaluates the actual objective for every candidate on both updates;
    it does not test SciPy convergence. The larger motion cost in each update
    is calculated independently as ``(pi - 0.05)**2 + (pi - 0.1)**2``.

    **Description of Variables Being Tested**

    Compare every recorded objective value with its analytic expectation,
    verify the selected joint angles and 1 N thrust command, and check
    ``solutionFound`` and ``costVal`` after each update. The wrench-error
    vector must remain zero to absolute tolerances of 1e-14 N for force and
    1e-14 N m for torque. Exactly ten optimizer calls are expected across the
    two updates.
    """
    allocation, input_messages = single_arm_allocation
    allocation.setWf(0.0)
    allocation.setWtheta(1.0)
    allocation.Reset(0)
    near_boundary = np.array([-np.pi + 0.05, np.pi - 0.1])  # [rad]
    decisions = np.zeros((5, 3))
    decisions[:, 2] = 1.0  # [N]
    decisions[2, :2] = near_boundary
    calls = _stub_optimizer(monkeypatch, decisions, [True] * 5)
    current_angles = np.array([np.pi - 0.05, -np.pi + 0.1])  # [rad]

    for update_index, angles in enumerate([current_angles, np.zeros(2)]):
        for message, angle in zip(input_messages["jointStatesInMsgs"], angles):
            payload = messaging.ScalarJointStateMsgPayload()
            payload.state = float(angle)
            message.write(payload)
        allocation.UpdateState(update_index)

        expected_angles = near_boundary if update_index == 0 else np.zeros(2)
        expected_cost = 0.05 if update_index == 0 else 0.0
        far_cost = (np.pi - 0.05)**2 + (np.pi - 0.1)**2
        expected_costs = [far_cost, far_cost, 0.05, far_cost, far_cost]
        if update_index == 1:
            expected_costs = [0.0, 0.0, far_cost, 0.0, 0.0]
        np.testing.assert_allclose([cost for _, cost in calls[-5:]], expected_costs)
        np.testing.assert_allclose(
            allocation.desJointAnglesOutMsg.read().states, expected_angles
        )
        assert allocation.thrForceOutMsg.read().thrForce[0] == pytest.approx(1.0)
        assert allocation.solutionFound == 1
        assert allocation.costVal == pytest.approx(expected_cost)
        np.testing.assert_allclose(allocation.wrenchError, np.zeros(6), atol=1e-14)
    assert len(calls) == 10


@pytest.mark.parametrize("use_motion_penalty", [False, True])
def test_update_optimizer_failure(
    single_arm_allocation, monkeypatch, use_motion_penalty
):
    """
    **Validation Test Description**

    First run a successful update using prescribed candidates with joint
    angles [0.4, -0.3] rad and 0.5 N thrust. Then mark all five optimizer
    results unsuccessful and run another update without resetting the module.
    Starting with a nonzero command ensures that the failure path must
    overwrite previous outputs and clear the previous success diagnostics.

    **Test Parameter Discussion**

    ``use_motion_penalty`` selects the fallback joint command. When true,
    unit motion weighting is enabled and the command must hold the measured
    angles [0.2, -0.4] rad. When false, the command must be [0, 0] rad even
    though joint-state messages are connected. Both cases use the real cost
    function with an optimizer stub that supplies the success/failure flags.

    **Description of Variables Being Tested**

    Confirm that the first update reports success and commands 0.5 N thrust,
    and that both updates together visit ten seeds. After failure, every
    thrust slot must be zero, the joint-angle array must contain exactly the
    two fallback angles, and commanded rates and accelerations must be zero.
    Check ``solutionFound == 0``, the positive-infinity failure sentinel in
    ``bestErrInf``, and ``NaN`` in ``costVal``. With zero thrust, ``wrenchError``
    must equal the original 1 N body-x request with all other components zero.
    """
    allocation, input_messages = single_arm_allocation
    current_angles = np.array([0.2, -0.4])  # [rad]
    for message, angle in zip(input_messages["jointStatesInMsgs"], current_angles):
        payload = messaging.ScalarJointStateMsgPayload()
        payload.state = float(angle)
        message.write(payload)
    if use_motion_penalty:
        allocation.setWtheta(1.0)
    allocation.Reset(0)
    decisions = np.tile([0.4, -0.3, 0.5], (5, 1))  # columns: [rad, rad, N]
    successes = [True] * 5
    calls = _stub_optimizer(monkeypatch, decisions, successes)
    allocation.UpdateState(0)
    assert allocation.solutionFound == 1
    assert allocation.thrForceOutMsg.read().thrForce[0] == pytest.approx(0.5)

    successes[:] = [False] * 5
    allocation.UpdateState(1)

    assert len(calls) == 10
    np.testing.assert_array_equal(allocation.thrForceOutMsg.read().thrForce, 0.0)
    expected_angles = current_angles if use_motion_penalty else np.zeros(2)
    joint_output = allocation.desJointAnglesOutMsg.read()
    np.testing.assert_allclose(joint_output.states, expected_angles)
    np.testing.assert_array_equal(joint_output.stateDots, np.zeros(2))
    np.testing.assert_array_equal(joint_output.stateDDots, np.zeros(2))
    assert allocation.solutionFound == 0
    assert np.isposinf(allocation.bestErrInf)
    assert np.isnan(allocation.costVal)
    np.testing.assert_allclose(allocation.wrenchError, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("previous_success", [True, False])
def test_reset_clears_solution_diagnostics(
    single_arm_allocation, monkeypatch, previous_success
):
    """
    **Validation Test Description**

    Run an allocation, reset the module, and run another allocation. The
    optimizer stub supplies a 0.5 N candidate for a 1 N request with unit
    thrust-use weight, giving cost 0.75 and wrench error 0.5 N when successful.
    Reset must discard these diagnostics together with the output commands.

    **Test Parameter Discussion**

    ``previous_success`` marks all optimizer results either successful or
    failed before reset. The failed case starts with the failure sentinel
    ``bestErrInf == inf`` and a 1 N wrench error. Both cases must return to the
    constructor's no-allocation state after reset. The subsequent update uses
    successful 0.75 N candidates, with cost 0.8125 and wrench error 0.25 N.

    **Description of Variables Being Tested**

    Before reset, verify the expected status, cost, and wrench-error diagnostics
    so the test cannot pass by resetting an unused module. After reset,
    ``solutionFound`` must be zero; ``bestErrInf``, ``costVal``, and all six
    entries of ``wrenchError`` must be NaN. Thrust outputs must be zero and all
    three joint-command arrays must be empty. The next update must populate
    the commands and diagnostics with the new successful allocation.
    """
    allocation, _ = single_arm_allocation
    allocation.setWf(1.0)
    allocation.Reset(0)
    decisions = np.tile([0.4, -0.3, 0.5], (5, 1))  # columns: [rad, rad, N]
    successes = [previous_success] * 5
    _stub_optimizer(monkeypatch, decisions, successes)

    allocation.UpdateState(0)

    assert allocation.solutionFound == int(previous_success)
    if previous_success:
        assert allocation.costVal == pytest.approx(0.75)
        assert allocation.bestErrInf == pytest.approx(0.5)  # [N]
        expected_error = 0.5  # [N]
    else:
        assert np.isnan(allocation.costVal)
        assert np.isposinf(allocation.bestErrInf)
        expected_error = 1.0  # [N]
    np.testing.assert_allclose(
        allocation.wrenchError, [expected_error, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    allocation.Reset(1)  # [ns]

    assert allocation.solutionFound == 0
    assert np.isnan(allocation.bestErrInf)
    assert np.isnan(allocation.costVal)
    assert allocation.wrenchError.shape == (6,)
    assert np.isnan(allocation.wrenchError).all()
    np.testing.assert_array_equal(allocation.thrForceOutMsg.read().thrForce, 0.0)
    joint_output = allocation.desJointAnglesOutMsg.read()
    assert len(joint_output.states) == 0
    assert len(joint_output.stateDots) == 0
    assert len(joint_output.stateDDots) == 0

    successes[:] = [True] * 5
    decisions[:, 2] = 0.75  # [N]
    allocation.UpdateState(2)  # [ns]

    assert allocation.solutionFound == 1
    assert allocation.costVal == pytest.approx(0.8125)
    assert allocation.bestErrInf == pytest.approx(0.25)  # [N]
    np.testing.assert_allclose(allocation.wrenchError, [0.25, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert allocation.thrForceOutMsg.read().thrForce[0] == pytest.approx(0.75)  # [N]
    np.testing.assert_allclose(
        allocation.desJointAnglesOutMsg.read().states, decisions[0, :2]
    )


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
