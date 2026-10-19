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

import numpy as np
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


def test_spherical_harmonics_field_uses_reference_radius_of_file(tmp_path):
    """Verify the computed field is scaled by the reference radius of the coefficient file.

    A coefficient file with only the normalized :math:`\\bar{C}_{20}` term is
    loaded into a body whose equatorial radius and gravitational parameter
    differ from those of the file. The field computed after initializing the
    body is compared against the closed-form :math:`J_2` acceleration, which
    uses the reference radius of the file and the ``mu`` of the body.
    """
    radFile = 6378136.3  # [m]
    muFile = 3.986004415e14  # [m^3/s^2]
    c20Bar = -4.84165371736e-4  # [-]
    coeffPath = tmp_path / "zonalDegreeTwo.txt"
    coeffPath.write_text(
        f"{radFile}, {muFile}, 0.0, 2, 2, 1, 0.0, 0.0\n"
        f"2, 0, {c20Bar}, 0.0, 0.0, 0.0\n"
        "2, 1, 0.0, 0.0, 0.0, 0.0\n"
        "2, 2, 0.0, 0.0, 0.0, 0.0\n"
    )

    body = gravityEffector.GravBodyData()
    body.mu = 3.9860044e14  # [m^3/s^2]
    body.radEquator = 6.0e6  # [m]
    body.useSphericalHarmonicsGravityModel(str(coeffPath), 2)
    body.initBody(0)

    position = np.array([4.0e6, -3.0e6, 5.5e6])  # [m] planet-fixed
    fieldJ2 = np.array(body.spherHarm.computeField(position, 2, False)).flatten()  # [m/s^2]

    def j2Acceleration(radius):
        """Closed-form :math:`J_2` acceleration for the given reference radius."""
        j2 = -np.sqrt(5.0) * c20Bar  # [-]
        x, y, z = position
        r = np.linalg.norm(position)  # [m]
        factor = -1.5 * j2 * body.mu * radius**2 / r**5  # [1/s^2]
        zRatio = 5.0 * z**2 / r**2  # [-]
        return factor * np.array([x * (1.0 - zRatio), y * (1.0 - zRatio), z * (3.0 - zRatio)])

    expected = j2Acceleration(radFile)  # [m/s^2]
    np.testing.assert_allclose(fieldJ2, expected, rtol=1e-12, atol=0.0)
    # The body radius would scale the field by (radFile / body.radEquator)^-2.
    assert not np.allclose(fieldJ2, j2Acceleration(body.radEquator), rtol=1e-3, atol=0.0)
