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

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import GravityGradientEffector, gravityEffector, spacecraft
from Basilisk.simulation.polyhedralGravityModel import PolyhedralGravityModel
from Basilisk.simulation.sphericalHarmonicsGravityModel import (
    SphericalHarmonicsGravityModel,
)
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody
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


@pytest.mark.parametrize(
    "mu_case", ["unset", "negative", "nan", "infinite", "negative_infinite", "body", "file"]
)
def test_spherical_harmonics_body_mu_is_consistent(mu_case):
    """Require a valid body GM and use it for both acceleration and gravity-gradient torque.

    A degree-zero model isolates the central gravity field. Invalid body GMs must
    stop simulation initialization even when the coefficient file supplies a valid
    GM. Valid cases use either an explicit body GM different from the file or an
    explicit copy of the file GM. The published planet GM, spacecraft acceleration,
    and gravity-gradient torque must all agree with the chosen body GM.

    :param mu_case: Body GM configuration to exercise.
    """
    body = gravityEffector.GravBodyData()
    body.planetName = "earth"
    body.isCentralBody = True
    ggm03s_path = str(get_path(DataFile.LocalGravData.GGM03S))
    body.useSphericalHarmonicsGravityModel(ggm03s_path, 0)
    invalid_mu = {
        "negative": -1.0,
        "nan": float("nan"),
        "infinite": float("inf"),
        "negative_infinite": float("-inf"),
    }  # [m^3/s^2]
    if mu_case == "body":
        body.mu = 0.5 * body.spherHarm.muBody
    elif mu_case == "file":
        body.mu = body.spherHarm.muBody
    elif mu_case in invalid_mu:
        body.mu = invalid_mu[mu_case]
    expected_mu = body.mu  # [m^3/s^2] Preserve the requested GM before initialization.

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    time_step = macros.sec2nano(1.0)  # [ns]
    process.addTask(simulation.CreateNewTask("task", time_step))
    spacecraft_object = spacecraft.Spacecraft()
    spacecraft_object.hub.mHub = 100.0  # [kg]
    inertia = np.diag([500.0, 800.0, 350.0])  # [kg*m^2]
    spacecraft_object.hub.IHubPntBc_B = inertia.tolist()
    position = np.array([4.0e6, -3.0e6, 5.5e6])  # [m]
    spacecraft_object.hub.r_CN_NInit = position.tolist()
    spacecraft_object.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    spacecraft_object.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    spacecraft_object.gravField.gravBodies = spacecraft.GravBodyVector([body])
    gravity_gradient = GravityGradientEffector.GravityGradientEffector()
    gravity_gradient.addPlanetName(body.planetName)
    spacecraft_object.addDynamicEffector(gravity_gradient)
    simulation.AddModelToTask("task", spacecraft_object)
    simulation.AddModelToTask("task", gravity_gradient)

    if mu_case not in ("body", "file"):
        with pytest.raises(BasiliskError, match=r"GravBodyData\.mu.*finite and positive"):
            simulation.InitializeSimulation()
        return

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(0)
    simulation.ExecuteSimulation()

    assert body.mu == expected_mu
    assert body.spherHarm.muBody == expected_mu
    published_mu = spacecraft_object.dynManager.getPropertyReference("earth.mu")
    np.testing.assert_array_equal(published_mu, [[expected_mu]])
    radius = np.linalg.norm(position)
    expected_acceleration = -expected_mu * position / radius**3
    acceleration = spacecraft_object.dynManager.getPropertyReference("g_N")
    np.testing.assert_allclose(np.asarray(acceleration).flatten(), expected_acceleration, rtol=1e-14, atol=0.0)
    # At identity attitude, the position components are also in the body frame.
    expected_torque = 3.0 * expected_mu / radius**5 * np.cross(position, inertia @ position)
    np.testing.assert_allclose(
        np.asarray(gravity_gradient.torqueExternalPntB_B).flatten(), expected_torque, rtol=1e-14, atol=0.0
    )


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
