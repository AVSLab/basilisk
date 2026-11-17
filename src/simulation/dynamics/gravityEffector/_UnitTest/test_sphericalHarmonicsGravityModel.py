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

"""Validate spherical-harmonic table bounds and initialization (issue 1682).

Malformed coefficient tables must produce recoverable errors through direct
calls and ``GravBodyData``. Evaluation checks also cover public configuration
changes after initialization. Degree-zero results are compared with point-mass
gravity, degree-two results with a stored baseline, and reinitialized models
with freshly initialized models using the same coefficients.
"""

from functools import partial
import re

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import gravityEffector
from Basilisk.simulation.sphericalHarmonicsGravityModel import (
    SphericalHarmonicsGravityModel,
)


MU = 3.986004415e14  # [m^3/s^2]
REFERENCE_RADIUS = 6378136.3  # [m]
POSITION = [7.0e6, 1.0e6, 2.0e6]  # [m]
C_BAR = [[1.0], [0.0, 0.0], [-4.84e-4, 0.0, 2.44e-6]]  # [-]
S_BAR = [[0.0], [0.0, 0.0], [0.0, 0.0, -1.40e-6]]  # [-]
# Reference acceleration from develop e5eddcb99d before the validation changes.
DEGREE_TWO_FIELD = [-7.0369155911816765, -1.005316093907567, -2.015474299531327]  # [m/s^2]
MAX_SIZE_T = int(np.iinfo(np.uintp).max)


def _make_model(degree=2):
    """Configure a degree-two coefficient table without initializing caches.

    :param degree: Requested maximum degree; may intentionally exceed the table.
    :return: Configured spherical-harmonic model.
    """
    model = SphericalHarmonicsGravityModel()
    model.cBar = C_BAR
    model.sBar = S_BAR
    model.muBody = MU
    model.radEquator = REFERENCE_RADIUS
    model.maxDeg = degree
    return model


def _initializer(model, through_body):
    """Return an initializer retaining the same model and optional body.

    :param model: Model to initialize.
    :param through_body: Whether to use ``GravBodyData.initBody``.
    :return: Callable returning ``None`` on success.
    """
    if not through_body:
        return model.initializeParameters
    body = gravityEffector.GravBodyData()
    body.planetName = "spherical_harmonics_test"
    body.mu = MU
    body.radEquator = REFERENCE_RADIUS
    body.gravityModel = model
    return partial(body.initBody, 0)


def _damage_table(model, table_name, defect):
    """Remove required coefficient storage and return the expected error text.

    :param model: Model whose public table is changed.
    :param table_name: Either ``cBar`` or ``sBar``.
    :param defect: Missing storage configuration to create.
    :return: Regular expression identifying the malformed table and row.
    """
    table = [list(row) for row in getattr(model, table_name)]
    if defect == "empty":
        table = []
    elif defect == "missing_row":
        table.pop()
    elif defect == "short_row":
        table[2].pop()
    else:
        table[0] = []
    setattr(model, table_name, table)
    if defect == "short_row":
        return table_name + r" row 2.*3 entries"
    if defect == "empty_zero_row":
        return table_name + r" row 0.*1 entries"
    return table_name + r".*row for degree 2"


def _assert_initialization_error(initialize, through_body, message):
    """Check the documented error contract for the selected entry point.

    :param initialize: Initialization callable.
    :param through_body: Whether the callable raises ``BasiliskError``.
    :param message: Expected regular expression in the diagnostic.
    """
    if through_body:
        with pytest.raises(BasiliskError, match=message):
            initialize()
    else:
        error = initialize()
        assert isinstance(error, str)
        assert re.search(message, error), error


@pytest.mark.parametrize("table_name", ["cBar", "sBar"])
@pytest.mark.parametrize(
    "defect", ["empty", "missing_row", "short_row", "empty_zero_row"]
)
@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
def test_rejects_invalid_tables(table_name, defect, through_body):
    """Reject missing rows and entries in either coefficient table.

    :param table_name: Coefficient table to damage.
    :param defect: Missing storage configuration.
    :param through_body: Whether initialization uses ``GravBodyData``.
    """
    model = _make_model()
    message = _damage_table(model, table_name, defect)
    _assert_initialization_error(
        _initializer(model, through_body), through_body, message
    )


@pytest.mark.parametrize("degree", [3, 40, MAX_SIZE_T], ids=["boundary", "40", "maximum_size"])
@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
def test_rejects_degree_beyond_tables(degree, through_body):
    """Reject excessive degrees before any degree-dependent allocation or addition.

    :param degree: Invalid degree for a three-row table.
    :param through_body: Whether initialization uses ``GravBodyData``.
    """
    model = _make_model(degree)
    _assert_initialization_error(
        _initializer(model, through_body), through_body,
        "cBar.*row for degree " + str(degree),
    )


@pytest.mark.parametrize("explicit_degree", [False, True])
def test_evaluation_requires_initialization(explicit_degree):
    """Reject evaluation before a successful initialization.

    :param explicit_degree: Whether to use the explicit-degree overload.
    """
    model = _make_model()
    args = (POSITION, 2, True) if explicit_degree else (POSITION,)
    with pytest.raises(BasiliskError, match="not initialized"):
        model.computeField(*args)


@pytest.mark.parametrize("table_name", ["cBar", "sBar"])
@pytest.mark.parametrize(
    "defect", ["empty", "missing_row", "short_row", "empty_zero_row"]
)
@pytest.mark.parametrize("explicit_degree", [False, True])
def test_evaluation_rechecks_table_dimensions(table_name, defect, explicit_degree):
    """Reject public table mutations that would cause out-of-bounds reads.

    :param table_name: Coefficient table to damage after initialization.
    :param defect: Missing storage configuration.
    :param explicit_degree: Whether to use the explicit-degree overload.
    """
    model = _make_model()
    assert model.initializeParameters() is None
    message = _damage_table(model, table_name, defect)
    args = (POSITION, 2, True) if explicit_degree else (POSITION,)
    with pytest.raises(BasiliskError, match=message):
        model.computeField(*args)


@pytest.mark.parametrize("degree", [3, 40, MAX_SIZE_T])
@pytest.mark.parametrize("explicit_degree", [False, True])
def test_evaluation_rejects_unprepared_degree(degree, explicit_degree):
    """Reject higher degrees even after public configuration allows them.

    :param degree: Degree exceeding the initialized recurrence storage.
    :param explicit_degree: Whether to use the explicit-degree overload.
    """
    model = _make_model()
    assert model.initializeParameters() is None
    model.maxDeg = degree
    # Supply degree-three coefficients to isolate the recurrence bound in that case.
    if degree == 3:
        model.cBar = C_BAR + [[0.0] * 4]
        model.sBar = S_BAR + [[0.0] * 4]
    args = (POSITION, degree, True) if explicit_degree else (POSITION,)
    with pytest.raises(BasiliskError, match="exceeds initialized degree 2"):
        model.computeField(*args)


def test_evaluation_preserves_configured_degree_limit():
    """Retain the existing limit even when larger recurrence tables are available."""
    model = _make_model()
    assert model.initializeParameters() is None
    model.maxDeg = 1
    with pytest.raises(BasiliskError, match="greater than maximum degree"):
        model.computeField(POSITION, 2, True)


@pytest.mark.parametrize("degree", [0, 1, 2])
@pytest.mark.parametrize("layout", ["triangular", "padded", "unused_rows"])
@pytest.mark.parametrize("include_zero_degree", [False, True])
def test_valid_fields_are_unchanged(degree, layout, include_zero_degree):
    """Preserve point-mass and degree-two results with supported table layouts.

    :param degree: Highest degree retained from the degree-two input tables.
    :param layout: Triangular, padded, or extra unused rows.
    :param include_zero_degree: Whether to include the point-mass contribution.
    """
    model = _make_model(degree)
    for table_name in ("cBar", "sBar"):
        table = [list(row) for row in getattr(model, table_name)]
        if layout == "padded":
            table = [row + [1.0] * (4 - len(row)) for row in table]  # [-] Unused entries.
        elif layout == "unused_rows":
            table.append([])
        setattr(model, table_name, table)
    assert model.initializeParameters() is None
    point_mass = -MU * np.array(POSITION) / np.linalg.norm(POSITION)**3
    expected = np.array(DEGREE_TWO_FIELD) if degree == 2 else point_mass
    if not include_zero_degree:
        expected = expected - point_mass
    absolute_tolerance = 1e-13  # [m/s^2]
    np.testing.assert_allclose(
        np.asarray(model.computeField(POSITION, degree, include_zero_degree)).ravel(),
        expected, rtol=1e-12, atol=absolute_tolerance,
    )


@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
def test_reinitialization_rebuilds_recurrence_tables(through_body):
    """Match fresh models after growing, shrinking, and repeating the degree.

    :param through_body: Whether all reinitializations use ``GravBodyData``.
    """
    model = _make_model()
    initialize = _initializer(model, through_body)
    for degree in (0, 2, 1, 2, 2, 0, 2):
        model.maxDeg = degree
        assert initialize() is None
        fresh = _make_model(degree)
        assert fresh.initializeParameters() is None
        np.testing.assert_allclose(
            model.computeField(POSITION), fresh.computeField(POSITION),
            rtol=1e-13, atol=0.0,
        )


@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
@pytest.mark.parametrize("previous_degree", [None, 0, 2])
@pytest.mark.parametrize("failure", ["table", "degree"])
def test_retry_after_failed_initialization(through_body, previous_degree, failure):
    """Invalidate failed attempts and recover using only the rebuilt caches.

    :param through_body: Whether initialization uses ``GravBodyData``.
    :param previous_degree: Optional degree of an earlier successful initialization.
    :param failure: Invalid table or degree configuration.
    """
    model = _make_model()
    initialize = _initializer(model, through_body)
    if previous_degree is not None:
        model.maxDeg = previous_degree
        assert initialize() is None
    model.maxDeg = 2
    if failure == "table":
        message = _damage_table(model, "sBar", "short_row")
    else:
        model.maxDeg = 40
        message = "cBar.*row for degree 40"
    _assert_initialization_error(initialize, through_body, message)
    with pytest.raises(BasiliskError, match="not initialized"):
        model.computeField(POSITION)
    model.sBar = S_BAR
    model.maxDeg = 2
    with pytest.raises(BasiliskError, match="not initialized"):
        model.computeField(POSITION)
    assert initialize() is None
    fresh = _make_model()
    assert fresh.initializeParameters() is None
    np.testing.assert_allclose(
        model.computeField(POSITION), fresh.computeField(POSITION),
        rtol=1e-13, atol=0.0,
    )


@pytest.mark.parametrize("invalid_mu", [0.0, float("nan")])  # [m^3/s^2]
def test_invalid_body_parameter_invalidates_initialization(invalid_mu):
    """Invalidate caches even when the body overload rejects its GM first.

    :param invalid_mu: Invalid gravitational parameter in m^3/s^2.
    """
    model = _make_model()
    body = gravityEffector.GravBodyData()
    body.planetName = "invalid_mu_test"
    body.mu = MU
    body.gravityModel = model
    body.initBody(0)
    body.mu = invalid_mu
    with pytest.raises(BasiliskError, match=r"GravBodyData\.mu.*finite and positive"):
        body.initBody(0)
    with pytest.raises(BasiliskError, match="not initialized"):
        model.computeField(POSITION)
    body.mu = MU
    body.initBody(0)
    np.testing.assert_allclose(
        np.asarray(model.computeField(POSITION)).ravel(), DEGREE_TWO_FIELD,
        rtol=1e-13, atol=0.0,
    )


def test_coefficient_values_can_change_without_reinitialization():
    """Read updated coefficients when their table dimensions remain valid."""
    model = _make_model()
    assert model.initializeParameters() is None
    updated_c = [list(row) for row in C_BAR]
    updated_c[2][0] *= 2.0
    model.cBar = updated_c
    fresh = _make_model()
    fresh.cBar = updated_c
    assert fresh.initializeParameters() is None
    np.testing.assert_allclose(
        model.computeField(POSITION), fresh.computeField(POSITION),
        rtol=1e-13, atol=0.0,
    )


def test_truncated_evaluation_ignores_unused_rows():
    """Check only the coefficient prefix required by an explicit lower degree."""
    model = _make_model()
    assert model.initializeParameters() is None
    model.cBar = [[1.0], [0.0, 0.0], []]  # [-]
    fresh = _make_model(1)
    assert fresh.initializeParameters() is None
    np.testing.assert_allclose(
        model.computeField(POSITION, 1, True), fresh.computeField(POSITION),
        rtol=1e-13, atol=0.0,
    )
    with pytest.raises(BasiliskError, match="cBar row 2"):
        model.computeField(POSITION)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
