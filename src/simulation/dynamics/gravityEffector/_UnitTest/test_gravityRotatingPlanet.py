# ISC License
#
# Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""Tests that the gravity effector keeps the extrapolated planet orientation a rotation."""

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody

OMEGA_PLANET = 7.2921150e-5  # [rad/s] planet spin rate about the inertial z axis
MU_EARTH = 3.986004415e14  # [m^3/s^2]
R_INIT = [7.0e6, 0.0, 0.0]  # [m]
V_INIT = [0.0, 5.0e3, 5.5e3]  # [m/s]
SIM_TIME = 3600.0  # [s]


def _planetMessage(spinning):
    """Return a planet state message with the planet spinning about the inertial z axis.

    The message is written once at t = 0, so the gravity effector must extrapolate the
    orientation by the full elapsed time.

    Args:
        spinning (bool): if False the orientation is the identity with a zero rate.
    """
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.J20002Pfix = np.eye(3).tolist()
    if spinning:
        # J20002Pfix maps inertial components into the planet-fixed frame
        payload.J20002Pfix_dot = (OMEGA_PLANET * np.array([[0.0, 1.0, 0.0],
                                                           [-1.0, 0.0, 0.0],
                                                           [0.0, 0.0, 0.0]])).tolist()
    return messaging.SpicePlanetStateMsg().write(payload)


def _propagate(stepSeconds, spinning):
    """Propagate a point-mass orbit about a planet with the given orientation message.

    Args:
        stepSeconds (float): integration time step in seconds.
        spinning (bool): whether the planet orientation message carries a spin rate.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(stepSeconds)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "sc"
    scObject.gravField.bskLogger.setLogLevel(bskLogging.BSK_ERROR)
    scSim.AddModelToTask("t", scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createCustomGravObject("planet", MU_EARTH)
    planet.isCentralBody = True
    planetMsg = _planetMessage(spinning)
    planet.planetBodyInMsg.subscribeTo(planetMsg)
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject.hub.r_CN_NInit = R_INIT
    scObject.hub.v_CN_NInit = V_INIT

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(SIM_TIME))
    scSim.ExecuteSimulation()

    stateManager = scObject.dynManager
    dcm = np.array(stateManager.getPropertyReference("planet.J20002Pfix"))
    r_N = np.array(scObject.scStateOutMsg.read().r_BN_N)
    return r_N, dcm


@pytest.mark.parametrize("stepSeconds", [10.0, 60.0])
def test_extrapolated_orientation_is_orthonormal(stepSeconds):
    """Verify the extrapolated planet orientation remains a proper rotation matrix.

    A first-order update of the matrix elements loses orthonormality by about
    ``(omega * dt)^2 / 2``, which is a few percent after an hour of extrapolation."""
    _, dcm = _propagate(stepSeconds, spinning=True)

    np.testing.assert_allclose(dcm @ dcm.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(dcm) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("stepSeconds", [10.0, 60.0])
def test_point_mass_orbit_is_independent_of_planet_spin(stepSeconds):
    """Verify a point-mass orbit does not depend on the planet orientation.

    The point-mass field is spherically symmetric, so a spinning planet must give the same
    trajectory as a non-spinning one. A non-orthonormal orientation scales the evaluated
    field and breaks this."""
    rSpinning, _ = _propagate(stepSeconds, spinning=True)
    rFixed, _ = _propagate(stepSeconds, spinning=False)

    np.testing.assert_allclose(rSpinning, rFixed, atol=1e-6)


if __name__ == "__main__":
    test_extrapolated_orientation_is_orthonormal(10.0)
    test_point_mass_orbit_is_independent_of_planet_spin(10.0)
