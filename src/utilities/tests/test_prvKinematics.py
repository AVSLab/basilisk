# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

"""Check principal rotation vector conversions, composition, and rate matrices."""

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.fixture(params=["python", "numba"])
def prv_backend(request):
    """Exercise the Python implementation and its compiled Numba counterpart."""
    if request.param == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        return pytest.importorskip("Basilisk.utilities.RigidBodyKinematicsNumba")
    return rbk


def inverse_rate_reference(vector):
    """Integrate the rotation-matrix exponential using its power series."""
    first, second, third = vector
    skew = np.array([[0.0, -third, second], [third, 0.0, -first],
                     [-second, first, 0.0]])
    result = np.eye(3)
    term = np.eye(3)
    for order in range(1, 49):
        term = term @ (-skew) / (order + 1)
        result += term
    return result


@pytest.mark.parametrize("angle", [0.0, 1e-16, 1e-12, 1e-9, 1e-5,
                                   0.009999, 0.01, 0.010001, 0.2, 2.0, 5.0])  # [rad]
def test_prv_rate_matrices(prv_backend, angle):
    """Check zero, small, and finite attitudes against an independent series."""
    vector = angle * np.array([2.0, -3.0, 6.0]) / 7.0  # [rad]
    expected_inverse = inverse_rate_reference(vector)
    expected = np.linalg.inv(expected_inverse)
    matrix = prv_backend.BmatPRV(vector)
    inverse = prv_backend.BinvPRV(vector)
    # Scale the absolute tolerance so tiny off-diagonal terms remain observable.
    tolerance = 3e-15 * min(angle, 1.0)
    np.testing.assert_allclose(matrix, expected, rtol=3e-14, atol=tolerance)
    np.testing.assert_allclose(inverse, expected_inverse, rtol=3e-14, atol=tolerance)
    omega = np.array([0.1, -0.2, 0.3])  # [rad/s]
    np.testing.assert_allclose(prv_backend.dPRV(vector, omega), expected @ omega,
                               rtol=3e-14, atol=1e-16)
    if angle == 0.0:
        np.testing.assert_array_equal(matrix, np.eye(3))
        np.testing.assert_array_equal(inverse, np.eye(3))
        np.testing.assert_array_equal(prv_backend.dPRV(vector, omega), omega)


def rotation_reference(vector):
    """Build a passive DCM with the matrix-exponential power series."""
    first, second, third = vector
    negative_skew = np.array([[0.0, third, -second], [-third, 0.0, first],
                              [second, -first, 0.0]])
    result = np.eye(3)
    term = np.eye(3)
    for order in range(1, 65):
        term = term @ negative_skew / order
        result += term
    return result


@pytest.mark.parametrize("angle", [0.0, 1e-16, 1e-13, 1e-9, 1e-8,
                                   1e-7, 1e-3, 0.4])  # [rad]
def test_prv_small_conversions(prv_backend, angle):
    """Retain tiny rotations through matrices, quaternions, Gibbs vectors, and MRPs."""
    for axis in [np.array([1.0, 0.0, 0.0]), np.array([2.0, -3.0, 6.0]) / 7.0]:
        vector = angle * axis  # [rad]
        matrix = rotation_reference(vector)
        quaternion = np.r_[np.cos(angle / 2.0), axis * np.sin(angle / 2.0)]
        gibbs = axis * np.tan(angle / 2.0)
        mrp = axis * np.tan(angle / 4.0)
        elements = np.r_[angle, axis] if angle else np.zeros(4)
        np.testing.assert_allclose(prv_backend.PRV2elem(vector), elements, rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(prv_backend.PRV2C(vector), matrix, rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(prv_backend.PRV2EP(vector), quaternion, rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(prv_backend.PRV2Gibbs(vector), gibbs, rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(prv_backend.PRV2MRP(vector), mrp, rtol=5e-15, atol=0.0)
        for name, value in [("EP2PRV", quaternion), ("C2PRV", matrix),
                            ("gibbs2PRV", gibbs), ("MRP2PRV", mrp)]:
            np.testing.assert_allclose(getattr(prv_backend, name)(value), vector,
                                       rtol=5e-15, atol=0.0)


@pytest.mark.parametrize("operation", ["add", "sub"])
def test_prv_composition(prv_backend, operation):
    """Preserve identity, cancellation, tiny increments, and noncommuting rotations."""
    cases = np.array([
        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        [[1.0, -0.5, 0.3], [0.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [1.0, -0.5, 0.3]],
        [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
        [[1.0, -0.5, 0.3], [-1.0, 0.5, -0.3]],
        [[1.0, -0.5, 0.3], [1.0, -0.5, 0.3]],
        [[1.0, 0.0, 0.0], [-1.0 + 1e-12, 0.0, 0.0]],
        [[1.0, 0.0, 0.0], [1.0 + 1e-12, 0.0, 0.0]],
        [[2.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        [[1.5, 0.5, 0.5], [-0.5, 0.25, 0.15]],
        [[1e-9, 2e-9, 0.0], [0.0, 0.7, -0.3]],
        [[1e-13, 2e-13, 0.0], [0.0, -3e-13, 4e-13]],
    ])  # [rad]
    for first, second in cases:
        matrix_first = rotation_reference(first)
        matrix_second = rotation_reference(second)
        expected = (matrix_second @ matrix_first if operation == "add"
                    else matrix_first @ matrix_second.T)
        result = getattr(prv_backend, f"{operation}PRV")(first, second)
        assert np.all(np.isfinite(result))
        np.testing.assert_allclose(rotation_reference(result), expected, rtol=0.0, atol=2e-14)
        if np.array_equal(first, -second if operation == "add" else second):
            np.testing.assert_array_equal(result, np.zeros(3))
    vector = np.array([2.0, 0.0, 0.0])  # [rad]
    np.testing.assert_allclose(prv_backend.addPRV(vector, vector), 2.0 * vector,
                               rtol=0.0, atol=1e-15)  # [rad]
