# ISC License
#
# Copyright (c) 2026, AVS Laboratory, University of Colorado at Boulder
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

"""Check sequence compatibility separately from the numerical edge-case suites.

Every affected function receives lists and tuples in each sequence argument.
Mixed pairs cover operand conversion without a Cartesian product of containers;
array-only numerical coverage lives in the conversion and kinematics tests.
"""

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.fixture(params=["python", "numba"])
def backend(request):
    """Use plain Python or the compiled Numba implementation."""
    if request.param == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        from Basilisk.utilities import RigidBodyKinematicsNumba
        return RigidBodyKinematicsNumba
    return rbk


@pytest.mark.parametrize("representation", ["MRP", "Gibbs"])
@pytest.mark.parametrize("operation", ["add", "sub"])
@pytest.mark.parametrize("container1, container2", [(list, tuple), (tuple, list)],
                         ids=["list-tuple", "tuple-list"])
@pytest.mark.parametrize("case", ["ordinary", "first-identity", "second-identity", "cancelling"])
def test_composition_sequence_inputs(backend, representation, operation, container1, container2, case):
    """Compose mixed containers correctly without modifying either input."""
    first = np.array([0.1, -0.2, 0.3])  # [-]
    second = np.array([-0.3, 0.2, 0.1])  # [-]
    if case == "first-identity":
        first[:] = 0.0
    elif case == "second-identity":
        second[:] = 0.0
    elif case == "cancelling":
        second = -first if operation == "add" else first.copy()
    value1, value2 = container1(first), container2(second)
    to_dcm = rbk.MRP2C if representation == "MRP" else rbk.gibbs2C
    if operation == "add":
        expected = to_dcm(second) @ to_dcm(first)
    else:
        expected = to_dcm(first) @ to_dcm(second).T

    result = getattr(backend, operation + representation)(value1, value2)

    assert result.shape == (3,)
    np.testing.assert_allclose(to_dcm(result), expected, rtol=0.0, atol=5e-15)
    if representation == "MRP":
        assert np.linalg.norm(result) <= 1.0
    np.testing.assert_array_equal(value1, first)
    np.testing.assert_array_equal(value2, second)


@pytest.mark.parametrize("operation", ["add", "sub"])
@pytest.mark.parametrize("container", [list, tuple], ids=["list", "tuple"])
@pytest.mark.parametrize("first, second", [
    pytest.param([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], id="singular-denominator"),
    pytest.param([1.0, 0.0, 0.0], [1.0, 1e-4, 0.0], id="near-singular-denominator"),
    pytest.param([0.5, 0.0, 0.0], [0.5, 0.0, 0.0], id="output-shadow-switch"),
])  # [-] MRP components
def test_mrp_composition_shadow_branches(backend, operation, container, first, second):
    """Preserve the input and output shadow-set branches for sequence inputs."""
    first, second = np.array(first), np.array(second)
    if operation == "sub":
        second = -second
    value1, value2 = container(first), container(second)
    expected = rbk.MRP2C(second) @ rbk.MRP2C(first) if operation == "add" else (
        rbk.MRP2C(first) @ rbk.MRP2C(second).T)

    result = getattr(backend, operation + "MRP")(value1, value2)

    assert result.shape == (3,)
    assert np.linalg.norm(result) <= 1.0
    np.testing.assert_allclose(rbk.MRP2C(result), expected, rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(value1, first)
    np.testing.assert_array_equal(value2, second)


@pytest.mark.parametrize("container", [list, tuple], ids=["list", "tuple"])
@pytest.mark.parametrize("mrp, threshold, expected", [
    pytest.param([0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], id="identity"),
    pytest.param([0.2, -0.3, 0.4], 1.0, [0.2, -0.3, 0.4], id="below-threshold"),
    pytest.param([1.0, 0.0, 0.0], 1.0, [1.0, 0.0, 0.0], id="at-threshold"),
    pytest.param([2.0, -1.0, 2.0], 1.0, [-2.0 / 9.0, 1.0 / 9.0, -2.0 / 9.0],
                 id="above-threshold"),
    pytest.param([0.5, 0.0, 0.0], 0.25, [-2.0, 0.0, 0.0], id="custom-threshold"),
])  # [-] MRP components and norm thresholds
def test_mrp_switch_sequence_inputs(backend, container, mrp, threshold, expected):
    """Switch only above the requested threshold, with an array result in each branch."""
    value = container(mrp)

    result = backend.MRPswitch(value, threshold)

    assert result.shape == (3,)
    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=0.0)
    np.testing.assert_allclose(rbk.MRP2C(result), rbk.MRP2C(mrp), rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(value, mrp)


@pytest.mark.parametrize("container", [list, tuple], ids=["list", "tuple"])
@pytest.mark.parametrize("gibbs, expected", [
    pytest.param([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], id="identity"),
    pytest.param([4.0 / 9.0, -8.0 / 9.0, 8.0 / 9.0], [1.0 / 6.0, -1.0 / 3.0, 1.0 / 3.0],
                 id="mixed-signs"),
    pytest.param([1e-12, -2e-12, 3e-12], [5e-13, -1e-12, 1.5e-12], id="small-rotation"),
])  # [-] Gibbs and MRP components
def test_gibbs2mrp_sequence_inputs(backend, container, gibbs, expected):
    """Convert Gibbs sequences to the expected bounded MRP set without mutation."""
    value = container(gibbs)

    result = backend.gibbs2MRP(value)

    assert result.shape == (3,)
    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=0.0)
    np.testing.assert_allclose(rbk.MRP2C(result), rbk.gibbs2C(gibbs), rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(value, gibbs)


@pytest.mark.parametrize("source", ["MRP", "gibbs"])
@pytest.mark.parametrize("destination", ["C", "EP", "PRV"] + [
    "Euler" + sequence for sequence in
    ("121", "123", "131", "132", "212", "213", "231", "232", "312", "313", "321", "323")
])
@pytest.mark.parametrize("container", [list, tuple], ids=["list", "tuple"])
def test_mrp_gibbs_conversion_sequence_inputs(backend, source, destination, container):
    """Support sequence inputs through direct conversions and their Euler callers."""
    function_name = source + "2" + destination
    original = np.array([0.1, -0.2, 0.3])  # [-]
    value = container(original)
    expected = getattr(rbk, function_name)(original)

    result = getattr(backend, function_name)(value)

    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=5e-15)
    np.testing.assert_array_equal(value, original)


@pytest.mark.parametrize("function_name, parameters", [
    ("BinvGibbs", ("q",)), ("BinvMRP", ("q",)),
    ("BmatGibbs", ("q",)), ("BmatMRP", ("q",)),
    ("BdotmatMRP", ("q", "dq")),
    ("dGibbs", ("q", "w")), ("dMRP", ("q", "w")),
    ("dMRP2Omega", ("q", "dq")), ("ddMRP", ("q", "dq", "w", "dw")),
    ("ddMRP2dOmega", ("q", "dq", "ddq")),
])
@pytest.mark.parametrize("containers", [(list, tuple), (tuple, list)],
                         ids=["list-tuple", "tuple-list"])
def test_mrp_gibbs_rate_sequence_inputs(backend, function_name, parameters, containers):
    """Accept sequences for attitudes, derivatives, angular rates, and accelerations."""
    values = {
        "q": np.array([0.1, -0.2, 0.3]),  # [-]
        "dq": np.array([0.01, -0.02, 0.03]),  # [1/s]
        "ddq": np.array([-0.003, 0.002, 0.001]),  # [1/s^2]
        "w": np.array([0.1, 0.2, -0.3]),  # [rad/s]
        "dw": np.array([-0.03, 0.02, 0.01]),  # [rad/s^2]
    }
    originals = [values[name] for name in parameters]
    inputs = [containers[index % len(containers)](value) for index, value in enumerate(originals)]
    expected = getattr(rbk, function_name)(*originals)

    result = getattr(backend, function_name)(*inputs)

    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=5e-15)
    for value, original in zip(inputs, originals):
        np.testing.assert_array_equal(value, original)


@pytest.mark.parametrize("function_name", ["BmatPRV", "BinvPRV"])
@pytest.mark.parametrize("container, vector", [
    pytest.param(list, [0, 0, 0], id="integer-list-identity"),
    pytest.param(tuple, [0, 0, 0], id="integer-tuple-identity"),
    pytest.param(np.array, [0, 0, 0], id="integer-array-identity"),
    pytest.param(list, [0.1, -0.2, 0.3], id="ordinary-list"),
    pytest.param(tuple, [0.1, -0.2, 0.3], id="ordinary-tuple"),
])  # [rad] principal rotation vectors
def test_prv_rate_matrix_sequence_inputs(backend, function_name, container, vector):
    """Check sequences in both PRV branches and retain integer-array coverage."""
    original = np.array(vector, dtype=np.float64)
    value = container(vector)
    expected = getattr(rbk, function_name)(original)

    result = getattr(backend, function_name)(value)

    assert result.shape == (3, 3)
    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=0.0)
    inverse_name = "BinvPRV" if function_name == "BmatPRV" else "BmatPRV"
    np.testing.assert_allclose(result @ getattr(rbk, inverse_name)(original),
                               np.eye(3), rtol=0.0, atol=5e-15)
    np.testing.assert_array_equal(value, vector)


@pytest.mark.parametrize("representation", ["EP", "PRV"] + [
    "Euler" + sequence for sequence in
    ("121", "123", "131", "132", "212", "213", "231", "232", "312", "313", "321", "323")
])
@pytest.mark.parametrize("attitude_container, rate_container", [(list, tuple), (tuple, list)],
                         ids=["list-tuple", "tuple-list"])
def test_attitude_derivative_sequence_inputs(backend, representation, attitude_container,
                                             rate_container):
    """Accept mixed attitude/rate containers while preserving angular velocity."""
    attitude = np.array([0.1, 0.4, -0.3])  # [rad] nonsingular for every Euler sequence
    if representation == "EP":
        attitude = rbk.PRV2EP(attitude)  # [-] scalar-first unit quaternion
    omega = np.array([0.01, -0.02, 0.03])  # [rad/s]
    value, rate = attitude_container(attitude), rate_container(omega)
    expected = getattr(rbk, "d" + representation)(attitude, omega)

    result = getattr(backend, "d" + representation)(value, rate)

    assert result.shape == attitude.shape
    np.testing.assert_allclose(result, expected, rtol=5e-15, atol=5e-15)
    inverse = getattr(rbk, "Binv" + representation)(attitude)
    scale = 2.0 if representation == "EP" else 1.0
    np.testing.assert_allclose(scale * inverse @ result, omega, rtol=5e-15, atol=5e-15)
    np.testing.assert_array_equal(value, attitude)
    np.testing.assert_array_equal(rate, omega)


@pytest.mark.filterwarnings("error::RuntimeWarning")
def test_rate_array_precision(backend):
    """Promote ordinary float32 and unsigned attitudes before rate arithmetic."""
    omega = np.array([0.01, -0.02, 0.03])  # [rad/s]
    for attitude in (np.array([0.1, 0.2, 0.3], dtype=np.float32),
                     np.array([1, 0, 0], dtype=np.uint8)):  # [-]
        for representation in ("MRP", "Gibbs"):
            matrix = getattr(rbk, "Bmat" + representation)(attitude.astype(np.float64))
            actual = getattr(backend, "Bmat" + representation)(attitude)
            np.testing.assert_allclose(actual, matrix, rtol=5e-15, atol=0.0)
            factor = 0.25 if representation == "MRP" else 0.5
            derivative = getattr(backend, "d" + representation)(attitude, omega)
            np.testing.assert_allclose(derivative, factor * matrix @ omega, rtol=5e-15, atol=1e-17)
            inverse = getattr(backend, "Binv" + representation)(attitude)
            np.testing.assert_allclose(inverse @ derivative / factor, omega, rtol=5e-15, atol=1e-17)


def test_mrp_switch_float32_threshold(backend):
    """Keep equality unchanged and switch only above a single-precision threshold."""
    threshold = np.float32(1.1)  # [-]
    boundary = np.float64(threshold)  # [-]
    for component in (np.nextafter(boundary, 0.0), boundary, np.nextafter(boundary, np.inf)):
        value = np.array([component, 0.0, 0.0])  # [-]
        result = backend.MRPswitch(value, threshold)
        if component <= boundary:
            np.testing.assert_array_equal(result, value)
        else:
            np.testing.assert_allclose(result, [-1.0 / component, 0.0, 0.0], rtol=5e-15, atol=0.0)


def test_mrp_shadow_inputs(backend):
    """Preserve an ordinary rotation through its shadow, with a singular shadow at zero."""
    original = [2.0, -1.0, 2.0]  # [-]
    for container in (list, tuple, np.array):
        value = container(original)
        shadow = backend.MRPshadow(value)
        np.testing.assert_allclose(backend.MRP2EP(value), [-0.8, 0.4, -0.2, 0.4], rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(shadow, [-2/9, 1/9, -2/9], rtol=5e-15, atol=0.0)
        np.testing.assert_allclose(backend.MRP2C(shadow), rbk.MRP2C(original), rtol=0.0, atol=5e-15)
        np.testing.assert_array_equal(value, original)
    with np.errstate(divide="ignore", invalid="ignore"):
        assert np.isnan(backend.MRPshadow(np.zeros(3))).all()


def test_rate_sequences_in_numba_kernel():
    """Pass mixed arrays and tuples through compiled rate routines in both argument positions."""
    numba = pytest.importorskip("numba")
    pytest.importorskip("scipy")
    from Basilisk.utilities import RigidBodyKinematicsNumba as rbk_numba

    @numba.njit
    def rates(attitude, omega):
        quaternion = rbk_numba.PRV2EP(attitude)
        return (rbk_numba.dPRV(attitude, omega), rbk_numba.dEP(quaternion, omega),
                rbk_numba.dEuler321(attitude, omega), rbk_numba.BinvPRV(attitude),
                rbk_numba.dPRV(np.asarray(attitude), (omega[0], omega[1], omega[2])))

    attitude = [0.1, 0.4, -0.3]  # [rad]
    omega = [0.01, -0.02, 0.03]  # [rad/s]
    value, rate = tuple(attitude), np.array(omega)
    expected = (rbk.dPRV(attitude, omega), rbk.dEP(rbk.PRV2EP(attitude), omega),
                rbk.dEuler321(attitude, omega), rbk.BinvPRV(attitude),
                rbk.dPRV(attitude, omega))

    results = rates(value, rate)

    for result, reference in zip(results, expected):
        np.testing.assert_allclose(result, reference, rtol=5e-15, atol=5e-15)
    np.testing.assert_array_equal(value, attitude)
    np.testing.assert_array_equal(rate, omega)
