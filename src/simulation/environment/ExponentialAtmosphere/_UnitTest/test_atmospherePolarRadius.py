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

"""Test the optional polar radius of the atmosphere base class, which selects a geodetic altitude."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import exponentialAtmosphere, msisAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simSetPlanetEnvironment

RP_EARTH = orbitalMotion.RP_EARTH * 1000.0  # [m] polar radius
REQ_EARTH = orbitalMotion.REQ_EARTH * 1000.0  # [m] equatorial radius set by simSetPlanetEnvironment


def _density(polarRadius):
    """Return the density 400 km above the geodetic pole.

    Args:
        polarRadius (float): [m] polar radius given to the module, negative for a sphere.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(1.0)))

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.setPlanetPolarRadius(polarRadius)

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.J20002Pfix = np.eye(3).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)
    atmo.planetPosInMsg.subscribeTo(planetMsg)

    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [1000.0, 0.0, RP_EARTH + 400.0e3]  # [m] close to the pole
    scMsg = messaging.SCStatesMsg().write(payload)
    atmo.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1], atmo


def test_polar_radius_selects_geodetic_altitude():
    """Verify the density uses the geodetic altitude above the ellipsoid once the polar radius is set.

    Over the pole the ellipsoid is about 21 km closer to the planet center than the sphere, so a spacecraft
    400 km above the ellipsoid is 379 km above the sphere. The default planet is a sphere."""
    scaleHeight = 8500.0  # [m]
    baseDensity = 1.217  # [kg/m^3]
    sphericalAltitude = np.hypot(1000.0, RP_EARTH + 400.0e3) - REQ_EARTH  # [m]
    geodeticAltitude = 400.0e3  # [m]

    densitySphere, atmoSphere = _density(-1.0)
    densityEllipsoid, atmoEllipsoid = _density(RP_EARTH)

    assert atmoSphere.getPlanetPolarRadius() < 0.0
    assert atmoEllipsoid.getPlanetPolarRadius() == pytest.approx(RP_EARTH)
    assert densitySphere == pytest.approx(baseDensity * np.exp(-sphericalAltitude / scaleHeight), rel=1e-9, abs=0.0)
    assert densityEllipsoid == pytest.approx(baseDensity * np.exp(-geodeticAltitude / scaleHeight), rel=1e-3, abs=0.0)
    assert densityEllipsoid < densitySphere


def test_polar_radius_setter_and_getter():
    """Verify the polar radius defaults to a sphere, round-trips, and rejects zero."""
    for atmo in (exponentialAtmosphere.ExponentialAtmosphere(), msisAtmosphere.MsisAtmosphere()):
        assert atmo.getPlanetPolarRadius() < 0.0
        atmo.setPlanetPolarRadius(RP_EARTH)
        assert atmo.getPlanetPolarRadius() == RP_EARTH
        with pytest.raises(Exception):
            atmo.setPlanetPolarRadius(0.0)


if __name__ == "__main__":
    test_polar_radius_selects_geodetic_altitude()
    test_polar_radius_setter_and_getter()
