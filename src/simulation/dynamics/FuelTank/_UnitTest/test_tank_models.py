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


import inspect
import os

import numpy as np
import pytest
from Basilisk.simulation import fuelTank

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def test_tankModelConstantVolume(show_plots=False):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = 10
    model.r_TcT_TInit = [[1], [1], [1]]
    model.radiusTankInit = 5

    trials = [(0, 0), (10, -1), (5, -1)]  # mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [100, 0, 0, 0, 100, 0, 0, 0, 100],
        [50, 0, 0, 0, 50, 0, 0, 0, 50]
    ]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-10, 0, 0, 0, -10, 0, 0, 0, -10],
        [-10, 0, 0, 0, -10, 0, 0, 0, -10]
    ]
    true_r_TcT_T = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataITank,
                                   true_ITankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant volume tank inertia not equal")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataIPrimeTank,
                                   true_IPrimeTankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant volume tank inertia derivatives not equal")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        np.testing.assert_allclose(dataR,
                                   true_r_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant volume tank tank center mass position not equal")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPrime,
                                   true_rPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant volume tank tank center mass position derivative not equal")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPPrime,
                                   true_rPPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant volume tank tank center mass position second derivative not equal")


def test_tankModelConstantDensity(show_plots=False):
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelConstantDensity()
    model.propMassInit = 10
    model.r_TcT_TInit = [[1], [1], [1]]
    model.radiusTankInit = 5

    trials = [(0, 0), (10, -1), (5, -1)]  # mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [100, 0, 0, 0, 100, 0, 0, 0, 100],
        [31.498026247371826, 0, 0, 0, 31.498026247371826, 0, 0, 0, 31.498026247371826]
    ]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-16.666666666666668, 0, 0, 0, -16.666666666666668, 0, 0, 0, -16.666666666666668],
        [-10.499342082457275, 0, 0, 0, -10.499342082457275, 0, 0, 0, -10.499342082457275]
    ]
    true_r_TcT_T = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataITank,
                                   true_ITankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant density tank inertia not equal")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataIPrimeTank,
                                   true_IPrimeTankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant density tank inertia derivatives not equal")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        np.testing.assert_allclose(dataR,
                                   true_r_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant density tank tank center mass position not equal")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPrime,
                                   true_rPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant density tank tank center mass position derivative not equal")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPPrime,
                                   true_rPPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Constant density tank tank center mass position second derivative not equal")


def test_tankModelEmptying(show_plots=False):
    """Verify the emptying-tank properties at empty, full, and half-full conditions."""
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelEmptying()
    model.propMassInit = 10  # [kg]
    model.r_TcT_TInit = [[1], [1], [1]]  # [m]
    model.radiusTankInit = 5  # [m]

    trials = [(0, 0), (10, -1), (5, -1)]  # [kg], [kg/s] mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [100, 0, 0, 0, 100, 0, 0, 0, 100],
        [50.0, 0, 0, 0, 50.0, 0, 0, 0, 50]
    ]  # [kg*m^2]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-8.75, 0, 0, 0, -8.75, 0, 0, 0, -12.5]
    ]  # [kg*m^2/s]
    true_r_TcT_T = [
        [1, 1, 1 - 5.0],
        [1, 1, 1],
        [1, 1, 1.0 - 15.0 / 8.0]
    ]  # [m]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, -3.0 / 8.0]
    ]  # [m/s]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, -1.0 / 60.0]
    ]  # [m/s^2]

    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataITank,
                                   true_ITankPntT_T[idx],
                                   atol=accuracy,
                                   err_msg="Emptying tank inertia not equal")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataIPrimeTank,
                                   true_IPrimeTankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Emptying tank inertia derivative not equal")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        np.testing.assert_allclose(dataR,
                                   true_r_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Emptying tank center of mass position not equal")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPrime,
                                   true_rPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Emptying tank center of mass position derivative not equal")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPPrime,
                                   true_rPPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Emptying tank center of mass position second derivative not equal")


@pytest.mark.parametrize("fuel_mass_fraction", [0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95])
def test_emptying_tank_derivatives_match_finite_differences(fuel_mass_fraction):
    """Compare emptying-tank COM and inertia derivatives with centered finite differences."""
    model = fuelTank.FuelTankModelEmptying()
    model.propMassInit = 200.0  # [kg]
    model.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m]
    model.radiusTankInit = 0.5  # [m]

    fuel_mass = fuel_mass_fraction * model.propMassInit  # [kg]
    fuel_mass_rate = -1.7  # [kg/s]
    difference_time_step = 1.0e-2  # [s]

    model.computeTankProps(fuel_mass)
    center_position = np.array(model.r_TcT_T).reshape(3).copy()  # [m]
    model.computeTankPropDerivs(fuel_mass, fuel_mass_rate)
    center_position_derivative = np.array(model.rPrime_TcT_T).reshape(3).copy()  # [m/s]
    center_position_second_derivative = np.array(model.rPPrime_TcT_T).reshape(3).copy()  # [m/s^2]
    inertia_derivative = np.array(model.IPrimeTankPntT_T).copy()  # [kg*m^2/s]

    model.computeTankProps(fuel_mass + fuel_mass_rate * difference_time_step)
    center_position_plus = np.array(model.r_TcT_T).reshape(3).copy()  # [m]
    inertia_plus = np.array(model.ITankPntT_T).copy()  # [kg*m^2]

    model.computeTankProps(fuel_mass - fuel_mass_rate * difference_time_step)
    center_position_minus = np.array(model.r_TcT_T).reshape(3).copy()  # [m]
    inertia_minus = np.array(model.ITankPntT_T).copy()  # [kg*m^2]

    finite_difference_center_derivative = (
        center_position_plus - center_position_minus) / (2.0 * difference_time_step)  # [m/s]
    finite_difference_center_second_derivative = (
        center_position_plus - 2.0 * center_position + center_position_minus
    ) / (difference_time_step * difference_time_step)  # [m/s^2]
    finite_difference_inertia_derivative = (
        inertia_plus - inertia_minus) / (2.0 * difference_time_step)  # [kg*m^2/s]

    np.testing.assert_allclose(center_position_derivative, finite_difference_center_derivative, rtol=1.0e-5)
    np.testing.assert_allclose(
        center_position_second_derivative, finite_difference_center_second_derivative, rtol=1.0e-5)
    np.testing.assert_allclose(inertia_derivative, finite_difference_inertia_derivative, rtol=1.0e-5)


def test_tankModelUniformBurn(show_plots=False):
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelUniformBurn()
    model.propMassInit = 10
    model.r_TcT_TInit = [[1], [1], [1]]
    model.radiusTankInit = 5
    model.lengthTank = 5;

    trials = [(0, 0), (10, -1), (5, -1)]  # mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [83.33333333333334, 0, 0, 0, 83.33333333333334, 0, 0, 0, 125],
        [41.66666666666667, 0, 0, 0, 41.66666666666667, 0, 0, 0, 62.5]
    ]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-8.3333333333334, 0, 0, 0, -8.3333333333334, 0, 0, 0, -12.5],
        [-8.3333333333334, 0, 0, 0, -8.3333333333334, 0, 0, 0, -12.5]
    ]
    true_r_TcT_T = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataITank,
                                   true_ITankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank uniform burn inertia not equal")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataIPrimeTank,
                                   true_IPrimeTankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank uniform burn inertia derivative not equal")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        np.testing.assert_allclose(dataR,
                                   true_r_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank uniform burn center of mass position not equal")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPrime,
                                   true_rPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank uniform burn center of mass position derivative not equal")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPPrime,
                                   true_rPPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank uniform burn center of mass position second derivative not equal")


def test_tankModelCentrifugalBurn(show_plots=False):
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelCentrifugalBurn()
    model.propMassInit = 10
    model.r_TcT_TInit = [[1], [1], [1]]
    model.radiusTankInit = 5
    model.lengthTank = 5

    trials = [(0, 0), (10, -1), (5, -1)]  # mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [83.33333333333334, 0, 0, 0, 83.33333333333334, 0, 0, 0, 125],
        [57.291666666666671, 0, 0, 0, 57.291666666666671, 0, 0, 0, 93.75]
    ]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-2.0833333333333335, 0, 0, 0, -2.0833333333333335, 0, 0, 0, 0.0],
        [-8.3333333333333339, 0, 0, 0, -8.3333333333333339, 0, 0, 0, -12.500000000000002]
    ]
    true_r_TcT_T = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataITank,
                                   true_ITankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank centrifugal burn inertia not equal")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        np.testing.assert_allclose(dataIPrimeTank,
                                   true_IPrimeTankPntT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank centrifugal burn inertia derivative not equal")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        np.testing.assert_allclose(dataR,
                                   true_r_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank centrifugal burn center of mass position not equal")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        np.testing.assert_allclose(dataRPrime,
                                   true_rPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank centrifugal burn center of mass position derivative not equal")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]

        np.testing.assert_allclose(dataRPPrime,
                                   true_rPPrime_TcT_T[idx],
                                   rtol=accuracy,
                                   err_msg="Tank centrifugal burn center of mass position second derivative not equal")


if __name__ == "__main__":
    # test_tankModelConstantVolume(True)
    test_tankModelConstantDensity(True)
    # test_tankModelEmptying(False)
    # test_tankModelUniformBurn(False)
    # test_tankModelCentrifugalBurn(False)
