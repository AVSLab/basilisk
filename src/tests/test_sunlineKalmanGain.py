#
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
#

"""Test scalar innovation covariance inversion in sunline filters."""

import numpy as np
import pytest

from Basilisk.fswAlgorithms import okeefeEKF, sunlineEKF, sunlineSEKF


@pytest.mark.parametrize(
    "filter_module,state_count,gain_function",
    [
        (sunlineEKF, sunlineEKF.SKF_N_STATES, sunlineEKF.sunlineKalmanGain),
        (
            sunlineSEKF, sunlineSEKF.EKF_N_STATES_SWITCH,
            sunlineSEKF.sunlineKalmanGain,
        ),
        (
            okeefeEKF, okeefeEKF.SKF_N_STATES_HALF,
            okeefeEKF.sunlineKalmanGainOkeefe,
        ),
    ],
    ids=["sunlineEKF", "sunlineSEKF", "okeefeEKF"],
)
def test_single_observation_kalman_gain(
    filter_module, state_count, gain_function,
):
    """Check each filter's gain against an analytical scalar solution.

    The three dimensionless Sun-direction components have correlated
    covariance; the other states have zero covariance. With sensor normal
    ``[0.6, -0.8, 0]``, the projected covariance is ``[0.8, -2.1, -0.2]`` and
    the innovation variance is three. All three filters must produce this
    nonzero gain when only one observation is available.
    """
    covariance = np.zeros((state_count, state_count))
    covariance[:3, :3] = [
        [2.0, 0.5, 0.0],
        [0.5, 3.0, 0.25],
        [0.0, 0.25, 4.0],
    ]  # [-] Covariance of dimensionless Sun-direction components.
    observations = np.zeros(filter_module.MAX_N_CSS_MEAS*state_count)
    observations[:3] = [0.6, -0.8, 0.0]  # [-] Unit sensor normal.
    measurement_variance = 0.84  # [-] Normalized sensor measurement variance.
    expected_gain = np.zeros(state_count)
    expected_gain[:3] = [4.0/15.0, -0.7, -1.0/15.0]  # [-]

    gain = filter_module.new_doubleArray(state_count)
    try:
        for index in range(state_count):
            filter_module.doubleArray_setitem(gain, index, float("nan"))
        gain_function(
            covariance.ravel().tolist(), observations.tolist(),
            measurement_variance, 1, gain,
        )
        actual_gain = [
            filter_module.doubleArray_getitem(gain, index)
            for index in range(state_count)
        ]
        np.testing.assert_allclose(
            actual_gain, expected_gain, rtol=1e-14, atol=1e-15,
        )
    finally:
        filter_module.delete_doubleArray(gain)
