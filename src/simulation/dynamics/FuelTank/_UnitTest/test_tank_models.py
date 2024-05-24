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
    __tracebackhide__ = True

    model = fuelTank.FuelTankModelEmptying()
    model.propMassInit = 10
    model.r_TcT_TInit = [[1], [1], [1]]
    model.radiusTankInit = 5

    trials = [(0, 0), (10, -1), (5, -1)]  # mFuel, mDotFuel
    true_ITankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [100, 0, 0, 0, 100, 0, 0, 0, 100],
        [50.0, 0, 0, 0, 50.0, 0, 0, 0, 50]
    ]
    true_IPrimeTankPntT_T = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-8.75, 0, 0, 0, -8.75, 0, 0, 0, -12.5]
    ]
    true_r_TcT_T = [
        [1, 1, 1 - 5.0],
        [1, 1, 1],
        [1, 1, 1.0 - 15.0 / 8.0]
    ]
    true_rPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, -3.0 / 8.0]
    ]
    true_rPPrime_TcT_T = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, -17.0 / 30.0]
    ]

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
