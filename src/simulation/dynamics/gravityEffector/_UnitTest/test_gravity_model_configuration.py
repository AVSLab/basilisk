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

"""Tests for the Python ``GravBodyData`` gravity-model configuration API."""

import pytest

from Basilisk.simulation import gravityEffector
from Basilisk.simulation.polyhedralGravityModel import PolyhedralGravityModel
from Basilisk.simulation.sphericalHarmonicsGravityModel import (
    SphericalHarmonicsGravityModel,
)


def test_legacy_gravity_model_flags_are_read_only():
    """Verify the expired model-selection flag setters are no longer available."""
    body = gravityEffector.GravBodyData()

    assert body.useSphericalHarmParams is False
    assert body.usePolyhedral is False

    with pytest.raises(AttributeError):
        body.useSphericalHarmParams = True
    with pytest.raises(AttributeError):
        body.usePolyhedral = True

    body.gravityModel = SphericalHarmonicsGravityModel()
    assert body.useSphericalHarmParams is True
    assert body.usePolyhedral is False

    body.gravityModel = PolyhedralGravityModel()
    assert body.useSphericalHarmParams is False
    assert body.usePolyhedral is True


def test_gravity_model_access_errors_name_current_configuration_methods():
    """Verify model access errors direct users only to the supported configuration API."""
    body = gravityEffector.GravBodyData()

    with pytest.raises(ValueError) as spherical_error:
        _ = body.spherHarm
    assert "useSphericalHarmonicsGravityModel" in str(spherical_error.value)
    assert "useSphericalHarmParams" not in str(spherical_error.value)

    with pytest.raises(ValueError) as polyhedral_error:
        _ = body.poly
    assert "usePolyhedralGravityModel" in str(polyhedral_error.value)
    assert "set 'usePolyhedral'" not in str(polyhedral_error.value)
