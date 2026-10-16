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

"""Check Euler conversions by reconstructing the represented rotation."""

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as rbk

SEQUENCES = ["121", "123", "131", "132", "212", "213",
             "231", "232", "312", "313", "321", "323"]
DCM_TOLERANCE = 5e-14  # [-]


@pytest.fixture(params=["python", "numba"])
def euler_backend(request):
    """Run the same rotation checks with Python and compiled Numba functions."""
    if request.param == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        return pytest.importorskip("Basilisk.utilities.RigidBodyKinematicsNumba")
    return rbk


def elementary_product(sequence, angles, exact=False):
    """Build a passive DCM from elementary rotations, optionally with exact zeros."""
    matrix = np.eye(3)
    for axis, angle in zip(sequence, angles):
        row = int(axis) % 3
        column = (int(axis) + 1) % 3
        cosine, sine = np.cos(angle), np.sin(angle)
        if exact:
            cosine = 0.0 if abs(cosine) < 1e-15 else cosine
            sine = 0.0 if abs(sine) < 1e-15 else sine
        rotation = np.eye(3)
        rotation[row, row] = rotation[column, column] = cosine
        rotation[row, column] = sine
        rotation[column, row] = -sine
        matrix = rotation @ matrix
    return matrix


def check_conversions(backend, sequence, matrix, singular=False):
    """Check matrix and both quaternion-sign inputs against an independent DCM."""
    quaternion = rbk.C2EP(matrix)
    conversions = [("C", matrix), ("EP", quaternion), ("EP", -quaternion)]
    for kind, attitude in conversions:
        result = getattr(backend, f"{kind}2Euler{sequence}")(attitude)
        assert np.all(np.isfinite(result))
        np.testing.assert_allclose(elementary_product(sequence, result), matrix,
                                   rtol=0.0, atol=DCM_TOLERANCE)
        if singular:
            assert result[2] == 0.0
        if sequence[0] == sequence[2]:
            assert 0.0 <= result[1] <= np.pi
        else:
            assert -np.pi / 2.0 <= result[1] <= np.pi / 2.0


@pytest.mark.parametrize("sequence", SEQUENCES)
@pytest.mark.parametrize("endpoint", [0, 1])
def test_euler_singularities(euler_backend, sequence, endpoint):
    """Retain the rotation at both singularities, regardless of signed zeros."""
    limits = ([0.0, np.pi] if sequence[0] == sequence[2]
              else [-np.pi / 2.0, np.pi / 2.0])  # [rad]
    angles = [0.7, limits[endpoint], -1.1]  # [rad]
    matrix = elementary_product(sequence, angles, exact=True)
    for zero in [0.0, -0.0]:
        matrix[matrix == 0.0] = zero
        check_conversions(euler_backend, sequence, matrix, singular=True)


@pytest.mark.parametrize("sequence", SEQUENCES)
@pytest.mark.parametrize("angle", [0.0, 0.5])  # [rad]
def test_euler_first_axis_rotation(euler_backend, sequence, angle):
    """Preserve identity and pure first-axis rotations for every sequence."""
    matrix = elementary_product(sequence, [angle, 0.0, 0.0])
    check_conversions(euler_backend, sequence, matrix,
                      singular=sequence[0] == sequence[2])


@pytest.mark.parametrize("sequence", SEQUENCES)
@pytest.mark.parametrize("offset", [1e-4, 1e-8, 1e-12, 1e-15])  # [rad]
def test_euler_near_singularities(euler_backend, sequence, offset):
    """Preserve rotations on either side of each singularity without angle comparisons."""
    limits = ([0.0, np.pi] if sequence[0] == sequence[2]
              else [-np.pi / 2.0, np.pi / 2.0])  # [rad]
    for middle in limits:
        for direction in [-1.0, 1.0]:
            angles = [0.7, middle + direction * offset, -1.1]  # [rad]
            matrix = elementary_product(sequence, angles)
            check_conversions(euler_backend, sequence, matrix)


@pytest.mark.parametrize("sequence", SEQUENCES)
def test_euler_ordinary_rotations(euler_backend, sequence):
    """Check nonsingular rotations with multiple outer-angle quadrants."""
    for first, last in [(0.7, -1.1), (-2.8, 2.6), (2.9, 2.7)]:  # [rad]
        matrix = elementary_product(sequence, [first, 0.8, last])  # [rad]
        check_conversions(euler_backend, sequence, matrix)


@pytest.mark.parametrize("sequence", ["123", "132", "213", "231", "312", "321"])
@pytest.mark.parametrize("representation", ["MRP", "gibbs", "PRV"])
def test_euler_indirect_conversions(euler_backend, sequence, representation):
    """Conversions through Euler parameters must also preserve locked rotations."""
    for middle in [-np.pi / 2.0, np.pi / 2.0]:  # [rad]
        matrix = elementary_product(sequence, [0.7, middle, -1.1], exact=True)  # [rad]
        converter = {"MRP": rbk.C2MRP, "gibbs": rbk.C2Gibbs,
                     "PRV": rbk.C2PRV}[representation]
        function = getattr(euler_backend, f"{representation}2Euler{sequence}")
        result = function(converter(matrix))
        np.testing.assert_allclose(elementary_product(sequence, result), matrix,
                                   rtol=0.0, atol=DCM_TOLERANCE)


@pytest.mark.parametrize("sequence", SEQUENCES)
@pytest.mark.parametrize("operation", ["add", "sub"])
def test_euler_composition(euler_backend, sequence, operation):
    """Preserve composition with identity, cancellation, and singular inputs."""
    cases = np.array([
        [[0.2, 0.5, 0.7], [0.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [0.2, 0.5, 0.7]],
        [[0.2, 0.5, 0.7], [0.2, 0.5, 0.7]],
        [[0.7, 0.0, -1.1], [-0.3, 0.8, 0.9]],
        [[0.7, np.pi, -1.1], [-0.3, 0.8, 0.9]],
        [[0.7, 1e-12, -1.1], [-0.3, np.pi - 1e-12, 0.9]],
        [[0.7, 0.0, 0.0], [-0.7, 0.0, 0.0]],
        [[-2.8, 0.8, 2.6], [2.9, -0.6, 2.7]],
    ])  # [rad]
    for first, second in cases:
        matrix_first = elementary_product(sequence, first)
        matrix_second = elementary_product(sequence, second)
        expected = (matrix_second @ matrix_first if operation == "add"
                    else matrix_first @ matrix_second.T)
        result = getattr(euler_backend, f"{operation}Euler{sequence}")(first, second)
        assert np.all(np.isfinite(result))
        np.testing.assert_allclose(elementary_product(sequence, result), expected,
                                   rtol=0.0, atol=DCM_TOLERANCE)
