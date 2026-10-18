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

"""Public type and convention compatibility regressions for issue 1629."""

import math
import warnings

import numpy as np
import pytest

from Basilisk.utilities import deprecated, orbitalMotion as om


@pytest.fixture(autouse=True)
def reject_deprecation_warnings():
    """Existing orbital APIs remain supported without requiring caller migration."""
    with warnings.catch_warnings():
        warnings.simplefilter("error", deprecated.BSKDeprecationWarning)
        warnings.simplefilter("error", deprecated.BSKUrgentDeprecationWarning)
        yield


@pytest.mark.parametrize("solver,eccentricity", [(om.M2E, 0.4), (om.N2H, 1.4)])
@pytest.mark.parametrize("mean", [0.0, 0.3])  # [rad]
@pytest.mark.parametrize("scalar", [np.float32, np.float64])
@pytest.mark.parametrize("argument", [0, 1])
@pytest.mark.parametrize("zero_dimensional", [False, True])
def test_kepler_numpy_scalar_contract(solver, eccentricity, mean, scalar, argument, zero_dimensional):
    """Either NumPy input retains NumPy arithmetic, including the zero-anomaly shortcut."""
    arguments = [mean, eccentricity]
    arguments[argument] = scalar(arguments[argument])
    if zero_dimensional:
        arguments[argument] = np.asarray(arguments[argument])
    original = float(arguments[argument])
    result = solver(*arguments)
    assert float(arguments[argument]) == original
    if zero_dimensional and argument == 0:
        assert isinstance(result, np.ndarray)
        assert result.shape == ()
        assert result.dtype == np.dtype(scalar)
    else:
        assert type(result) is scalar
    reference = solver(*(float(value) for value in arguments))
    assert result == scalar(reference)
