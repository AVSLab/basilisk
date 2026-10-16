# ISC License
#
# Copyright (c) 2024, AVS Laboratory, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np
import pytest
from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.fixture(params=["python", "numba"])
def prv_backend(request):
    """Exercise PRV conversions in Python and, when available, Numba."""
    if request.param == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        return pytest.importorskip("Basilisk.utilities.RigidBodyKinematicsNumba")
    return rbk


@pytest.mark.parametrize("function_name, identity", [
    ("C2PRV", np.eye(3)),
    ("EP2PRV", np.array([1.0, 0.0, 0.0, 0.0])),
    ("EP2PRV", np.array([-1.0, 0.0, 0.0, 0.0])),
    ("MRP2PRV", np.zeros(3)),
    ("gibbs2PRV", np.zeros(3)),
])
def test_prv_identity(prv_backend, function_name, identity):
    """Identity attitudes must produce zero PRVs without invalid arithmetic."""
    with np.errstate(divide="raise", invalid="raise"):
        result = getattr(prv_backend, function_name)(identity)
    np.testing.assert_array_equal(result, np.zeros(3))


@pytest.mark.parametrize("function_name, identity", [
    ("EP2PRV", [1.0, 0.0, 0.0, 0.0]),
    ("EP2PRV", [-1.0, 0.0, 0.0, 0.0]),
    ("MRP2PRV", [0.0, 0.0, 0.0]),
    ("gibbs2PRV", [0.0, 0.0, 0.0]),
])
def test_prv_identity_lists(function_name, identity):
    """Plain Python lists must also support identity-to-PRV conversion."""
    with np.errstate(divide="raise", invalid="raise"):
        result = getattr(rbk, function_name)(identity)
    np.testing.assert_array_equal(result, np.zeros(3))


@pytest.mark.parametrize("axis", [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, -2.0, 3.0],
])  # [-] rotation-axis directions
@pytest.mark.parametrize("function_name", ["C2PRV", "EP2PRV", "MRP2PRV"])
def test_prv_half_turn(prv_backend, function_name, axis):
    """Exact half-turns must retain their angle and axis, up to axis sign."""
    axis = np.array(axis) / np.linalg.norm(axis)
    if function_name == "C2PRV":
        # A symmetric DCM has an exactly zero antisymmetric part.
        attitude = 2.0 * np.outer(axis, axis) - np.eye(3)
    elif function_name == "EP2PRV":
        attitude = np.concatenate(([0.0], axis))
    else:
        attitude = axis

    result = getattr(prv_backend, function_name)(attitude)
    if np.dot(result, axis) < 0.0:
        result = -result
    np.testing.assert_allclose(result, np.pi * axis, rtol=1e-14, atol=1e-14)


@pytest.mark.parametrize("angle", [
    1e-14, -1e-8, 1e-8, 0.5, np.pi - 1e-8, -np.pi + 1e-8,
])  # [rad]
@pytest.mark.parametrize("function_name", [
    "C2PRV", "EP2PRV", "MRP2PRV", "gibbs2PRV",
])
def test_prv_near_singular_rotations(prv_backend, function_name, angle):
    """Recover small, ordinary, and near-half-turn rotations analytically."""
    axis = np.array([1.0, -2.0, 3.0])  # [-]
    axis /= np.linalg.norm(axis)
    if function_name == "C2PRV":
        cross_matrix = np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ])
        attitude = (np.cos(angle) * np.eye(3)
                    + (1.0 - np.cos(angle)) * np.outer(axis, axis)
                    - np.sin(angle) * cross_matrix)
    elif function_name == "EP2PRV":
        attitude = np.concatenate(([np.cos(angle / 2.0)],
                                   np.sin(angle / 2.0) * axis))
    elif function_name == "MRP2PRV":
        attitude = np.tan(angle / 4.0) * axis
    else:
        attitude = np.tan(angle / 2.0) * axis

    with np.errstate(divide="raise", invalid="raise"):
        result = getattr(prv_backend, function_name)(attitude)
    np.testing.assert_allclose(result, angle * axis, rtol=1e-13, atol=0.0)


@pytest.mark.parametrize("function_name", ["EP2PRV", "MRP2PRV"])
def test_prv_long_rotation(prv_backend, function_name):
    """Preserve the existing angle branch for rotations greater than pi."""
    angle = 4.0  # [rad]
    axis = np.array([1.0, -2.0, 3.0])  # [-]
    axis /= np.linalg.norm(axis)
    if function_name == "EP2PRV":
        attitude = np.concatenate(([np.cos(angle / 2.0)],
                                   np.sin(angle / 2.0) * axis))
    else:
        attitude = np.tan(angle / 4.0) * axis
    result = getattr(prv_backend, function_name)(attitude)
    np.testing.assert_allclose(result, angle * axis, rtol=1e-14, atol=0.0)


def test_ep2euler321_clips_inverse_trig_roundoff():
    """Check ``EP2Euler321`` tolerates one-ulp inverse-trig roundoff."""
    positive_pitch_quat = [
        0.7071067811865476,
        0.0,
        0.7071067811865476,
        0.0,
    ]  # [-]
    negative_pitch_quat = [
        0.7071067811865476,
        0.0,
        -0.7071067811865476,
        0.0,
    ]  # [-]

    np.testing.assert_allclose(
        rbk.EP2Euler321(positive_pitch_quat),
        [0.0, np.pi / 2.0, 0.0],
        atol=1e-14,
    )
    np.testing.assert_allclose(
        rbk.EP2Euler321(negative_pitch_quat),
        [0.0, -np.pi / 2.0, 0.0],
        atol=1e-14,
    )


def test_subMRP(show_plots):
    mrp_1 = np.array([1.5, 0.5, 0.5])
    mrp_2 = np.array([-0.5, 0.25, 0.15])
    ans = [-0.005376344086021518,0.04301075268817203,-0.4408602150537635]
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="subtraction failed")

    # should get the same answer if one MRP is switched to shadoe set
    mrp_2 = rbk.MRPswitch(mrp_2, 0.0)
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="subtraction after q2 MRP switch failed")

    mrp_1 = np.array([0.0, 0.0, 1.0])
    mrp_2 = np.array([0.0, 0.0, -1.0])
    ans = [0.0, 0.0, 0.0]
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="180 deg case failed")


if __name__ == "__main__":
    test_subMRP(True)
