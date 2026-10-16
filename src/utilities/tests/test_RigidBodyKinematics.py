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


@pytest.mark.parametrize("layout", ["contiguous", "strided", "reversed", "column"])
@pytest.mark.parametrize("angle", [0.0, 0.7])  # [rad]
def test_prv2c_array_views(prv_backend, layout, angle):
    """Convert array views without changing the rotation or the input storage."""
    axis = np.array([2.0, -3.0, 6.0]) / 7.0  # [-]
    vector = angle * axis  # [rad]
    if layout == "strided":
        storage = np.zeros(6)
        storage[::2] = vector
        value = storage[::2]
    elif layout == "reversed":
        storage = vector[::-1].copy()
        value = storage[::-1]
    elif layout == "column":
        storage = np.column_stack((vector, vector))
        value = storage[:, 0]
    else:
        storage = vector.copy()
        value = storage
    assert value.flags.c_contiguous == (layout == "contiguous")
    original = storage.copy()
    quaternion = np.r_[np.cos(angle / 2.0), axis * np.sin(angle / 2.0)]
    expected = rbk.EP2C(quaternion)

    result = prv_backend.PRV2C(value)

    np.testing.assert_allclose(result, expected, rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(storage, original)


def test_prv2c_view_in_numba_kernel():
    """Convert a matrix column created inside a compiled Numba caller."""
    numba = pytest.importorskip("numba")
    pytest.importorskip("scipy")
    from Basilisk.utilities import RigidBodyKinematicsNumba as rbk_numba

    @numba.njit
    def column_to_dcm(matrix):
        return rbk_numba.PRV2C(matrix[:, 0])

    matrix = np.array([[0.2, 0.0], [-0.3, 0.0], [0.6, 0.0]])  # [rad]
    expected = rbk.PRV2C(matrix[:, 0])
    np.testing.assert_allclose(column_to_dcm(matrix), expected, rtol=0.0, atol=5e-15)


@pytest.mark.parametrize("backend", ["python", "numba"])
@pytest.mark.parametrize("container", [list, tuple, np.array], ids=["list", "tuple", "array"])
@pytest.mark.parametrize("quaternion, expected", [
    pytest.param([-0.5, 0.5, -0.5, 0.5], [-1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0],
                 id="negative-scalar"),
    pytest.param([0.5, -0.5, 0.5, -0.5], [-1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0],
                 id="positive-scalar"),
    pytest.param([-1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], id="negative-identity"),
    pytest.param([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], id="positive-identity"),
    pytest.param([-1.0, 1e-12, -2e-12, 3e-12], [-5e-13, 1e-12, -1.5e-12],
                 id="near-negative-identity"),
    pytest.param([0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0], id="half-turn"),
])  # [-] unit quaternions and their bounded MRP sets
def test_ep2mrp_sequence_inputs(backend, container, quaternion, expected):
    """Preserve bounded MRPs and input contents for each quaternion container."""
    if backend == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        from Basilisk.utilities import RigidBodyKinematicsNumba as implementation
    else:
        implementation = rbk
    value = container(quaternion)
    original = np.array(quaternion)

    result = implementation.EP2MRP(value)

    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=0.0)
    np.testing.assert_allclose(rbk.MRP2C(result), rbk.EP2C(original), rtol=0.0, atol=5e-15)
    assert np.linalg.norm(result) <= 1.0
    np.testing.assert_array_equal(value, original)


@pytest.mark.parametrize("backend", ["python", "numba"])
@pytest.mark.parametrize("container", [list, tuple, np.array], ids=["list", "tuple", "array"])
@pytest.mark.parametrize("mrp, expected", [
    pytest.param([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], id="identity"),
    pytest.param([0.1, 0.2, 0.3], [10.0 / 43.0, 20.0 / 43.0, 30.0 / 43.0],
                 id="reported-case"),
    pytest.param([-0.25, 0.5, 0.0], [-8.0 / 11.0, 16.0 / 11.0, 0.0],
                 id="mixed-signs"),
    pytest.param([2.0, 0.0, 0.0], [-4.0 / 3.0, 0.0, 0.0], id="long-rotation"),
    pytest.param([-0.5, 0.0, 0.0], [-4.0 / 3.0, 0.0, 0.0], id="shadow-set"),
    pytest.param([1e-12, -2e-12, 3e-12], [2e-12, -4e-12, 6e-12],
                 id="small-rotation"),
])  # [-] MRPs and their Gibbs vectors
def test_mrp2gibbs_sequence_inputs(backend, container, mrp, expected):
    """Return three Gibbs components for every container without changing inputs."""
    if backend == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        from Basilisk.utilities import RigidBodyKinematicsNumba as implementation
    else:
        implementation = rbk
    value = container(mrp)
    original = np.array(mrp)

    result = implementation.MRP2Gibbs(value)

    assert result.shape == (3,)
    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=0.0)
    np.testing.assert_allclose(rbk.gibbs2C(result), rbk.MRP2C(original), rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(value, original)


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
