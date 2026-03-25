#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""Regression tests for :mod:`Basilisk.utilities.pythonVariableLogger`."""

import numpy as np
import pytest

from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger


def test_python_variable_logger_getattr_returns_logged_values() -> None:
    """Ensure ``__getattr__`` serves logged names without generated properties."""
    logger = PythonVariableLogger({
        "theta-dot": lambda current_sim_nanos: current_sim_nanos,
    })

    logger.UpdateState(1)
    logger.UpdateState(2)

    assert "theta-dot" not in type(logger).__dict__
    np.testing.assert_array_equal(getattr(logger, "theta-dot"), np.array([1, 2]))


def test_python_variable_logger_getattr_keeps_times_method_available() -> None:
    """Ensure a logged ``times`` field does not shadow the logger time accessor."""
    logger = PythonVariableLogger({
        "times": lambda current_sim_nanos: current_sim_nanos,
    })

    logger.UpdateState(3)

    np.testing.assert_array_equal(logger.times(), np.array([3]))
    np.testing.assert_array_equal(logger.__getattr__("times"), np.array([3]))


def test_python_variable_logger_getattr_handles_missing_storage() -> None:
    """Ensure missing internal state still produces a clean ``AttributeError``."""
    logger = object.__new__(PythonVariableLogger)

    with pytest.raises(AttributeError, match="missing"):
        getattr(logger, "missing")
