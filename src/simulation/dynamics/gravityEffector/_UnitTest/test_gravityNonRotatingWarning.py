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

#
#   Unit Test Script
#   Module Name:        gravityEffector
#   Purpose:            Verify the orientation-dependence predicate that gates the
#                       non-rotating-planet warning, that GravityEffector::Reset()
#                       actually emits (and suppresses) the BSK_WARNING
#                       accordingly, and that spherical-harmonic gravity without a
#                       planet-orientation message still runs (issue #1352).
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      Jun. 25, 2026
#

import os

import pytest
from Basilisk import __path__
from Basilisk.simulation import spacecraft
from Basilisk.simulation.pointMassGravityModel import PointMassGravityModel
from Basilisk.simulation.polyhedralGravityModel import PolyhedralGravityModel
from Basilisk.simulation.sphericalHarmonicsGravityModel import SphericalHarmonicsGravityModel
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody

bskPath = __path__[0]
GGM03S = bskPath + "/supportData/LocalGravData/GGM03S.txt"
GGM03S_J2 = bskPath + "/supportData/LocalGravData/GGM03S-J2-only.txt"
# Polyhedral shape model (Eros) shipped alongside this test.
EROS_POLY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "EROS856Vert1708Fac.txt")
EROS_MU = 4.46275472004e5   # [m^3/s^2] Eros gravitational parameter

# A distinctive, stable fragment of the GravityEffector::Reset() warning text.
WARNING_SUBSTR = "no planet-orientation message"


def _buildShSim(gravFile, gravDeg):
    """Build a single-body spacecraft sim under a spherical-harmonic Earth field
    with NO planet-orientation message connected.

    The (unstarted) simulation is returned so the caller can run
    ``InitializeSimulation`` itself and capture the reset-time warning.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(10.0)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "sc"
    scSim.AddModelToTask("t", scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    earth.useSphericalHarmonicsGravityModel(gravFile, gravDeg)
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(
        list(gravFactory.gravBodies.values())
    )
    scObject.hub.r_CN_NInit = [-138514.048884, -888553.853077, 6818674.278964]   # [m]
    scObject.hub.v_CN_NInit = [-7306.6601, 2126.208457, 128.787296]              # [m/s]
    return scSim


def _buildPolySim():
    """Build a single-body spacecraft sim under a polyhedral (Eros) field with NO
    planet-orientation message connected.

    The (unstarted) simulation is returned so the caller can run
    ``InitializeSimulation`` itself and capture the reset-time warning.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(10.0)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "sc"
    scSim.AddModelToTask("t", scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    eros = gravFactory.createCustomGravObject("eros", mu=EROS_MU)
    eros.isCentralBody = True
    eros.usePolyhedralGravityModel(EROS_POLY)
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(
        list(gravFactory.gravBodies.values())
    )
    scObject.hub.r_CN_NInit = [40.0e3, 0.0, 0.0]   # [m] just outside the Eros body
    scObject.hub.v_CN_NInit = [0.0, 3.0, 0.0]      # [m/s]
    return scSim


def test_tesseralFieldDependsOnOrientation():
    """A spherical-harmonic field that retains order >= 1 terms varies with the
    body's longitude, so it must report that it depends on the body orientation.
    GravityEffector::Reset() uses this to decide whether to warn about a missing
    planet-orientation message (issue #1352)."""
    model = SphericalHarmonicsGravityModel()
    model.loadFromFile(GGM03S, 4)
    assert model.dependsOnOrientation() is True


def test_zonalFieldDoesNotDependOnOrientation():
    """A purely zonal field (J2 only, no order >= 1 terms) depends on latitude
    alone, so it must report no orientation dependence (no false warning)."""
    model = SphericalHarmonicsGravityModel()
    model.loadFromFile(GGM03S_J2, 2)
    assert model.dependsOnOrientation() is False


def test_pointMassDoesNotDependOnOrientation():
    """Point-mass gravity is spherically symmetric and never depends on the
    body orientation (the base-class default)."""
    assert PointMassGravityModel().dependsOnOrientation() is False


def test_polyhedralDependsOnOrientation():
    """A polyhedral shape model is generally non-axisymmetric, so its field is
    always orientation-dependent (issue #1352). This holds independently of the
    loaded vertex/facet data, so the bare model must report True."""
    assert PolyhedralGravityModel().dependsOnOrientation() is True


def test_resetEmitsWarningForTesseralFieldWithoutRotation(capfd):
    """GravityEffector::Reset() must actually emit the BSK_WARNING when a body
    whose field depends on orientation (GGM03S order 4, which retains tesseral
    terms) has no planet-orientation message connected. This is the user-facing
    alert for issue #1352, so the test fails if the warning block is removed.

    The warning is read from stdout (where ``bskLog`` writes it). It is observable
    here only because ``bskLog`` flushes warning-level output; without the flush
    the C runtime buffers it past pytest's capture point.

    Capturing the warning also exercises the full reset path, and propagating a
    few steps afterwards confirms the non-rotating field still initializes and
    runs (it must warn, not fail)."""
    scSim = _buildShSim(GGM03S, 4)
    scSim.InitializeSimulation()
    out, _ = capfd.readouterr()
    assert WARNING_SUBSTR in out, (
        "Reset() did not emit the missing-planet-rotation warning; captured "
        f"stdout was:\n{out}"
    )

    scSim.ConfigureStopTime(macros.sec2nano(60.0))
    scSim.ExecuteSimulation()


def test_resetEmitsWarningForPolyhedralWithoutRotation(capfd):
    """GravityEffector::Reset() must also emit the warning for a polyhedral body
    without a planet-orientation message. This is the case schaubh flagged: a
    polyhedral field is evaluated in the body-fixed frame and was previously
    falling back to non-rotating silently (issue #1352). Init-only to stay fast."""
    scSim = _buildPolySim()
    scSim.InitializeSimulation()
    out, _ = capfd.readouterr()
    assert WARNING_SUBSTR in out, (
        "Reset() did not emit the missing-planet-rotation warning for a polyhedral "
        f"body; captured stdout was:\n{out}"
    )


def test_resetSilentForZonalFieldWithoutRotation(capfd):
    """A purely zonal field (GGM03S J2-only) does not depend on orientation, so
    Reset() must NOT warn even without a planet-orientation message. This guards
    against a false alarm on the common J2-only configuration (issue #1352)."""
    scSim = _buildShSim(GGM03S_J2, 2)
    scSim.InitializeSimulation()
    out, _ = capfd.readouterr()
    assert WARNING_SUBSTR not in out, (
        "Reset() warned about a purely zonal (orientation-independent) field; "
        f"captured stdout was:\n{out}"
    )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
