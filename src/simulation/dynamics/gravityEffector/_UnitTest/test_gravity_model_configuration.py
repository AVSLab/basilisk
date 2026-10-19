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
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile


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


def test_spherical_harmonics_keep_reference_radius_of_file():
    """Verify initializing the body keeps the reference radius of the coefficient file.

    The factory Mars radius differs from the reference radius of
    ``GGM2BData.txt``. The field must keep the radius of the file, the
    body radius must be left unchanged, and the field must use the body ``mu``.
    """
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createMarsBarycenter()
    bodyRadius = planet.radEquator  # [m]
    ggm2bPath = str(get_path(DataFile.LocalGravData.GGM2BData))
    planet.useSphericalHarmonicsGravityModel(ggm2bPath, 4)

    planet.initBody(0)

    assert planet.spherHarm.radEquator == 3397000.0  # [m]
    assert planet.radEquator == bodyRadius
    assert planet.spherHarm.muBody == planet.mu


def test_spherical_harmonics_without_radius_use_body_radius():
    """Verify a model without a reference radius takes the radius of the body."""
    body = gravityEffector.GravBodyData()
    body.mu = 3.986004415e14  # [m^3/s^2]
    body.radEquator = 6378136.3  # [m]
    model = SphericalHarmonicsGravityModel()
    model.cBar = [[1.0], [0.0, 0.0], [-4.84165371736e-4, 0.0, 0.0]]
    model.sBar = [[0.0], [0.0, 0.0], [0.0, 0.0, 0.0]]
    model.maxDeg = 2
    body.gravityModel = model

    body.initBody(0)

    assert body.spherHarm.radEquator == body.radEquator
    assert body.spherHarm.muBody == body.mu


def test_spherical_harmonics_keep_file_mu_without_body_mu():
    """Verify the gravitational parameter of the file is kept when the body has none."""
    body = gravityEffector.GravBodyData()
    ggm03sPath = str(get_path(DataFile.LocalGravData.GGM03S))
    body.useSphericalHarmonicsGravityModel(ggm03sPath, 2)

    body.initBody(0)

    assert body.spherHarm.muBody == 3.986004415e14  # [m^3/s^2]
    assert body.spherHarm.radEquator == 6378136.3  # [m]
