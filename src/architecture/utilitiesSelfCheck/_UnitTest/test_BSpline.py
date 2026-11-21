
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
# BSpline Unit Test
#
# Purpose:  Tests the BSpline interpolating function
# Author:   Riccardo Calaon
# Creation Date:  Oct 10 2021
#

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import BSpline, bskLogging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("P", [5, 6])
@pytest.mark.parametrize("XDot_flag", [False, True])
@pytest.mark.parametrize("XDDot_flag", [False, True])
@pytest.mark.parametrize("accuracy", [1e-6])

def test_BSpline(show_plots, P, XDot_flag, XDDot_flag, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the capability of the BSpline function to correctly interpolate
    a series of points in 3 dimensions.
    The coordinates of these 7 points are stored in 3 numpy arrays:

    X1 = np.array([0, 1, 2, 3, 4, 5, 6])

    X2 = np.array([5, 4, 3, 2, 1, 0, 1])

    X3 = np.array([3, 2, 1, 2, 3, 4, 5]).

    The input arrays are initialized through ``Input = BSpline.InputDataSet(X1, X2, X3)``.
    The time tags at which each waypoint is to be hit are provided through ``Input.setT([0, 2, 3, 5, 7, 8, 10])``.
    Alternatively, it is possible to specify the average velocity norm through ``Input.setAvgXDot()``.
    The endpoint derivatives are specified through the methods:

    - ``Input.setXDot_0()`` for starting point first-order derivative;
    - ``Input.setXDot_N()`` for last point first-order derivative;
    - ``Input.setXDDot_0()`` for starting point second-order derivative;
    - ``Input.setXDDot_N()`` for last point second-order derivative.

    Each method to specify the derivatives takes in a 3-dimensional numpy array.
    The output data structure is created with ``Output = BSpline.OutputDataSet()``.
    The interpolation happens calling the method ``BSpline.interpolate(Input, N, P, Output)`` where:

    - N is the desired number of equally spaced data points in the interpolated function;

    - P is the polynomial order of the B-Spline function. The order should be at least 3 when first-order derivatives are specified,
      and 5 when second-order derivatives are specified. The maximum oder is P = n + k - 1, with n being the number of waypoints and k
      being the number of endpoint derivatives that are being specified.

    **Test Parameters**

    As this is a parameterized unit test, note that the test case parameters values are shown automatically in the
    pytest HTML report.  This sample script has the parameters param1 and param 2.  Provide a description of what
    each parameter controls.  This is a convenient location to include the accuracy variable used in the
    validation test.

    Args:
        P (int): polynomial order of the B-Spline curve;
        XDot_flag (bool) : whether the first-order end point derivatives should be specified;
        XDDot_flag (bool) : whether the second-order end point derivatives should be specified;
        accuracy (float): absolute accuracy value used in the validation tests.

    **Description of Variables Being Tested**

    This unit test checks the correctness of the interpolated function:
    - a check is performed on whether or not each waypoint is hit at the specified time;
    - when the derivatives are specified, it checks whether the starting point derivative actually matches the input derivative.
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = BSplineTestFunction(P, XDot_flag, XDDot_flag, accuracy)
    assert testResults < 1, testMessage


def BSplineTestFunction(P, XDot_flag, XDDot_flag, accuracy):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    X1 = np.array([0, 1, 2, 3, 4, 5, 6])
    X2 = np.array([5, 4, 3, 2, 1, 0, 1])
    X3 = np.array([3, 2, 1, 2, 3, 4, 5])

    Input = BSpline.InputDataSet(X1, X2, X3)
    Input.setT([0, 2, 3, 5, 7, 8, 10])
    if XDot_flag:
        Input.setXDot_0([0, 0, 0])
        Input.setXDot_N([0, 0, 0])
    if XDDot_flag:
        Input.setXDDot_0([0, 0, 0])
        Input.setXDDot_N([0.2, 0, 0])

    Output = BSpline.OutputDataSet()
    BSpline.interpolate(Input, 101, P, Output)

    for i in range(len(Output.T)):
        for j in range(len(Input.T)):
            if abs(Output.T[i][0] - Input.T[j][0]) < accuracy:
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[j][0]))
    if XDot_flag:
        if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and
                (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and
                (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
            testFailCount += 1
            testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
    if XDDot_flag:
        if not ((abs(Output.XDD1[0][0]-Input.XDDot_0[0][0]) < accuracy) and
                (abs(Output.XDD2[0][0]-Input.XDDot_0[1][0]) < accuracy) and
                (abs(Output.XDD3[0][0]-Input.XDDot_0[2][0]) < accuracy)):
            testFailCount += 1
            testMessages.append("FAILED: BSpline." + " Function of order {} failed second derivative at starting point".format(P))


    return [testFailCount, ''.join(testMessages)]


@pytest.mark.parametrize("start_derivative_order", [0, 1, 2])
@pytest.mark.parametrize("end_derivative_order", [0, 1, 2])
def test_approximate_endpoint_constraints(start_derivative_order, end_derivative_order):
    """Check positions and requested derivatives at both approximation endpoints.

    Each endpoint independently constrains position only, position and velocity,
    or position, velocity, and acceleration. Distinct nonzero vector components
    expose incorrect constraint-vector indexing, including GitHub issue 1563.
    Nonuniform time tags also exercise derivative scaling by the total duration.

    :param start_derivative_order: Highest derivative constrained at the start.
    :param end_derivative_order: Highest derivative constrained at the end.
    """
    waypoints = np.array([
        [0.0, 5.0, 3.0],
        [1.0, 4.0, 2.0],
        [2.0, 3.0, 1.0],
        [3.0, 2.0, 2.0],
        [4.0, 1.0, 3.0],
        [5.0, 0.0, 4.0],
        [6.0, 1.0, 5.0],
    ])  # [m]
    times = np.array([0.0, 2.0, 3.0, 5.0, 7.0, 8.0, 10.0])  # [s]
    velocities = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]])  # [m/s]
    accelerations = np.array([[0.01, 0.02, 0.03], [0.04, 0.05, 0.06]])  # [m/s^2]
    position_accuracy = 1e-10  # [m]
    velocity_accuracy = 1e-10  # [m/s]
    acceleration_accuracy = 1e-10  # [m/s^2]

    spline_input = BSpline.InputDataSet(*waypoints.T)
    spline_input.setT(times)
    if start_derivative_order >= 1:
        spline_input.setXDot_0(velocities[0])
    if start_derivative_order == 2:
        spline_input.setXDDot_0(accelerations[0])
    if end_derivative_order >= 1:
        spline_input.setXDot_N(velocities[1])
    if end_derivative_order == 2:
        spline_input.setXDDot_N(accelerations[1])

    output = BSpline.OutputDataSet()
    BSpline.approximate(spline_input, 101, 6, 4, output)
    positions = np.column_stack((output.X1, output.X2, output.X3))
    first_derivatives = np.column_stack((output.XD1, output.XD2, output.XD3))
    second_derivatives = np.column_stack((output.XDD1, output.XDD2, output.XDD3))

    np.testing.assert_allclose(
        positions[[0, -1]], waypoints[[0, -1]], rtol=0, atol=position_accuracy
    )
    for endpoint, derivative_order in enumerate((start_derivative_order, end_derivative_order)):
        sample = 0 if endpoint == 0 else -1
        if derivative_order >= 1:
            np.testing.assert_allclose(
                first_derivatives[sample], velocities[endpoint], rtol=0, atol=velocity_accuracy
            )
        if derivative_order == 2:
            np.testing.assert_allclose(
                second_derivatives[sample], accelerations[endpoint], rtol=0, atol=acceleration_accuracy
            )


@pytest.mark.parametrize("fit, args", [
    (BSpline.interpolate, (11, 3)),
    (BSpline.approximate, (11, 3, 3)),
], ids=["interpolate", "approximate"])
def test_public_fit_waypoint_lengths(fit, args):
    """Reject invalid lengths before changing the output and fit again after correction."""
    data = BSpline.InputDataSet()
    output = BSpline.OutputDataSet()
    output.P = 1
    for count in (0, 1):
        data.X1 = data.X2 = data.X3 = np.zeros(count)  # [m]
        with pytest.raises(bskLogging.BasiliskError, match="at least two waypoints"):
            fit(data, *args, output)
    data.X1 = np.arange(6.0)  # [m]
    data.X2 = data.X3 = np.zeros(6)  # [m]
    data.setT(np.arange(6.0))  # [s]
    for field in ("X1", "X2", "X3", "T"):
        original = np.ravel(getattr(data, field))
        for values in ([], original[:-1], np.append(original, original[-1])):
            setattr(data, field, values)
            with pytest.raises(bskLogging.BasiliskError, match="waypoint"):
                fit(data, *args, output)
            assert output.P == 1
        setattr(data, field, original)
    fit(data, *args, output)
    expected = np.linspace(0.0, 5.0, 11)  # [m]
    np.testing.assert_allclose(np.ravel(output.X1), expected, atol=1e-12)


@pytest.mark.parametrize("fit, args", [
    (BSpline.interpolate, (11, 3)),
    (BSpline.approximate, (11, 3, 3)),
], ids=["interpolate", "approximate"])
def test_public_fit_waypoint_times(fit, args):
    """Reject invalid explicit and distance-based times before modifying the output."""
    x = np.arange(4.0)  # [m]
    data = BSpline.InputDataSet(x, np.zeros(4), np.zeros(4))
    output = BSpline.OutputDataSet()
    output.P = 1
    for times in ([0, 0, 0, 0], [0, 1, 1, 3], [0, 2, 1, 3], [-1, 0, 1, 2], [1, 2, 3, 4],
                  [0, 1, np.nan, 3], [0, 1, 2, np.inf]):  # [s]
        data.setT(times)
        with pytest.raises(bskLogging.BasiliskError, match="strictly increasing"):
            fit(data, *args, output)
        assert output.P == 1

    for speed in (0.0, -1.0, np.nan, np.inf):  # [m/s]
        data.setAvgXDot(speed)
        with pytest.raises(bskLogging.BasiliskError, match="speed must be finite and positive"):
            fit(data, *args, output)
        assert output.P == 1

    data.setAvgXDot(2.0)  # [m/s]
    data.X1 = [0.0, 1.0, 1.0, 3.0]  # [m]
    with pytest.raises(bskLogging.BasiliskError, match="strictly increasing"):
        fit(data, *args, output)
    assert output.P == 1
    data.X1 = x
    fit(data, *args, output)
    np.testing.assert_allclose(np.ravel(output.T), np.linspace(0, 1.5, 11))  # [s]
    np.testing.assert_allclose(np.ravel(output.X1), np.linspace(0, 3, 11), atol=1e-12)  # [m]


def test_approximate_free_control_point_limit():
    """One interior waypoint can determine one free control point."""
    data = BSpline.InputDataSet([0, 1, 2], [0, 0, 0], [0, 0, 0])  # [m]
    data.setT([0, 1, 2])  # [s]
    output = BSpline.OutputDataSet()
    output.P = 1
    with pytest.raises(bskLogging.BasiliskError, match="more free control points than interior waypoints"):
        BSpline.approximate(data, 3, 3, 3, output)
    assert output.P == 1
    data.setXDot_0([0, 0, 0])  # [m/s]
    BSpline.approximate(data, 3, 3, 3, output)
    np.testing.assert_allclose(np.ravel(output.X1), [0, 1, 2])  # [m]
    assert output.getStates(0, 1, 0) == pytest.approx(0.0)  # [m/s]


@pytest.mark.parametrize("offset", [[8.0, -4.0, 2.0], [np.nan, np.inf, -np.inf]], ids=["finite", "nonfinite"])  # [m]
def test_approximate_zero_weight_waypoints(offset):
    """Ignore a zero-weight outlier and reject weights that leave a control point undetermined."""
    times = np.arange(10.0)  # [s]
    velocity = np.array([1.0, -2.0, 0.5])  # [m/s]
    expected = times[:, None] * velocity  # [m]
    waypoints = expected.copy()
    waypoints[4] += offset
    data = BSpline.InputDataSet(*waypoints.T)
    data.setT(times)
    weights = np.ones(10)
    weights[4] = 0
    data.setW(weights)
    output = BSpline.OutputDataSet()
    BSpline.approximate(data, 10, 3, 1, output)
    np.testing.assert_allclose(np.column_stack((output.X1, output.X2, output.X3)), expected, atol=1e-12)

    # Both retained interior waypoints precede the first interior knot and constrain the same free control point.
    weights[3:-1] = 0
    data.setW(weights)
    with pytest.raises(bskLogging.BasiliskError, match="rank deficient"):
        BSpline.approximate(data, 10, 3, 1, output)
    np.testing.assert_allclose(np.column_stack((output.X1, output.X2, output.X3)), expected, atol=1e-12)


def test_approximate_interior_weights():
    """A symmetric quadratic gives both interior samples their weighted mean."""
    data = BSpline.InputDataSet([0, 1, 3, 0], np.zeros(4), np.zeros(4))  # [m]
    data.setT([0, 1, 2, 3])  # [s]
    output = BSpline.OutputDataSet()
    for weight in (-1.0, np.nan, np.inf):
        data.setW([1, 1, weight, 1])
        with pytest.raises(bskLogging.BasiliskError, match="weights must be finite and nonnegative"):
            BSpline.approximate(data, 4, 2, 2, output)
    data.setW([1, 1, 4, 1])
    BSpline.approximate(data, 4, 2, 2, output)
    np.testing.assert_allclose(np.ravel(output.X1), [0, 2.6, 2.6, 0], atol=1e-12)  # [m]


def test_approximate_interior_knots():
    """Reject a knot at the clamped start or without bracketing times, then fit a valid layout."""
    data = BSpline.InputDataSet([0, 1, 2], [0, 0, 0], [0, 0, 0])  # [m]
    data.setT([0, 1, 2])  # [s]
    data.setXDot_0([0, 0, 0])  # [m/s]
    data.setXDot_N([0, 0, 0])  # [m/s]
    data.setXDDot_0([0.25, 0, 0])  # [m/s^2]
    data.setXDDot_N([-0.5, 0, 0])  # [m/s^2]
    output = BSpline.OutputDataSet()
    output.P = 1
    for degree, reason in ((3, "increase strictly"), (2, "bracketed by waypoint times")):
        with pytest.raises(bskLogging.BasiliskError, match=reason):
            BSpline.approximate(data, 3, 5, degree, output)
        assert output.P == 1
    data.setW([0, 0, 0])  # The six endpoint conditions determine all six control points.
    BSpline.approximate(data, 3, 5, 4, output)
    np.testing.assert_allclose(np.ravel(output.U), [0] * 5 + [0.25] + [1] * 5)
    np.testing.assert_allclose(np.ravel(output.X1)[[0, -1]], [0, 2], atol=1e-12)  # [m]
    np.testing.assert_allclose(np.ravel(output.XDD1)[[0, -1]], [0.25, -0.5])  # [m/s^2]


@pytest.mark.parametrize("endpoint", ["0", "N"], ids=["start", "end"])
def test_approximate_acceleration_requires_velocity_at_same_endpoint(endpoint):
    """Require velocity at the accelerated endpoint even if the other velocity is set."""
    data = BSpline.InputDataSet([0, 2, 1, 3, 6, 4], np.zeros(6), np.zeros(6))  # [m]
    data.setT(np.arange(6.0))  # [s]
    other = "N" if endpoint == "0" else "0"
    getattr(data, f"setXDot_{other}")([0, 0, 0])  # [m/s]
    getattr(data, f"setXDDot_{endpoint}")([0.25, 0, 0])  # [m/s^2]
    output = BSpline.OutputDataSet()
    output.P = 1
    with pytest.raises(bskLogging.BasiliskError, match="requires velocity at each endpoint"):
        BSpline.approximate(data, 21, 4, 3, output)
    assert output.P == 1
    getattr(data, f"setXDot_{endpoint}")([0.5, 0, 0])  # [m/s]
    with pytest.raises(bskLogging.BasiliskError, match="support the endpoint derivatives"):
        BSpline.approximate(data, 21, 4, 1, output)
    BSpline.approximate(data, 21, 4, 3, output)
    sample = 0 if endpoint == "0" else -1
    assert np.ravel(output.XD1)[sample] == pytest.approx(0.5)  # [m/s]
    assert np.ravel(output.XDD1)[sample] == pytest.approx(0.25)  # [m/s^2]


def test_default_input_flags_and_weight_lengths():
    """Fit manually supplied coordinates with optional inputs disabled and validate active weights."""
    data = BSpline.InputDataSet()
    flags = ("T_flag", "AvgXDot_flag", "W_flag", "XDot_0_flag", "XDot_N_flag", "XDDot_0_flag", "XDDot_N_flag")
    assert not any(getattr(data, flag) for flag in flags)
    data.X1 = np.arange(6.0)  # [m]
    data.X2 = data.X3 = np.zeros(6)  # [m]
    output = BSpline.OutputDataSet()
    with pytest.raises(bskLogging.BasiliskError, match="degree must be less than the number of control points"):
        BSpline.approximate(data, 11, 2, 3, output)
    BSpline.approximate(data, 11, 3, 3, output)
    expected = np.linspace(0.0, 5.0, 11)  # [m]
    np.testing.assert_allclose(np.ravel(output.X1), expected, atol=1e-12)
    for count in (0, 5, 7):
        data.setW(np.ones(count))
        with pytest.raises(bskLogging.BasiliskError, match="weights must have one entry per waypoint"):
            BSpline.approximate(data, 11, 3, 3, output)
    BSpline.interpolate(data, 11, 3, output)
    np.testing.assert_allclose(np.ravel(output.X1), expected, atol=1e-12)
    data.setW(np.ones(6))
    BSpline.approximate(data, 11, 3, 3, output)
    np.testing.assert_allclose(np.ravel(output.X1), expected, atol=1e-12)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    BSplineTestFunction(
        5,        # polynomial order
        True,    # XDot_flag
        False,    # XDDot_flag
        1e-6)


def test_short_final_knot_span_preserves_constant():
    """The last basis function must only contribute inside its knot support."""
    fit = BSpline.OutputDataSet()
    fit.P = 3
    fit.T = [0, 1]  # [s]
    fit.U = [0, 0, 0, 0, 0.999999, 1, 1, 1, 1]
    fit.C1 = np.ones(5)
    fit.C2 = fit.C3 = np.zeros(5)
    for t in (0.999995, 0.999999, 0.9999999, 1.0):  # [s]
        assert fit.getStates(t, 0, 0) == pytest.approx(1.0, rel=0, abs=1e-12)
