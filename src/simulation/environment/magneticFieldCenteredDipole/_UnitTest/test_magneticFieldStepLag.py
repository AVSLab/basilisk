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

"""Test that the magnetic field base class evaluates the spacecraft state extrapolated to the step midpoint."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simSetPlanetEnvironment

STEP = 10.0  # [s] module update period


def _field(r, v, timeNanos):
    """Return the dipole field at the second module update.

    Args:
        r (list): [m] spacecraft position written to the state message.
        v (list): [m/s] spacecraft velocity written to the state message.
        timeNanos (int): [ns] time at which the state message is written.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    dipole = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
    dipole.setExtrapolateScStateToStepMidpoint(True)
    simSetPlanetEnvironment.centeredDipoleMagField(dipole, "earth")
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PlanetName = "earth"
    planetPayload.J20002Pfix = np.eye(3).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)
    dipole.planetPosInMsg.subscribeTo(planetMsg)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(r)
    payload.v_BN_N = list(v)
    scMsg = messaging.SCStatesMsg().write(payload, timeNanos)
    dipole.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", dipole)
    recorder = dipole.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    return np.array(recorder.magField_N[-1])


def test_field_is_evaluated_at_extrapolated_position():
    """Verify the field uses the state advanced by half of the message age.

    A state written at the start of the simulation with a radial velocity must give the field of
    the position advanced by v * dt / 2; the field decays with the cube of the radius so the
    advanced and static fields differ measurably."""
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    v0 = [1.0e3, 0.0, 0.0]  # [m/s] radial
    shift = v0[0] * STEP / 2.0  # [m]

    fieldFromVelocity = _field(r0, v0, 0)
    fieldAdvanced = _field([r0[0] + shift, 0.0, 0.0], [0.0, 0.0, 0.0], macros.sec2nano(STEP))
    fieldStatic = _field(r0, [0.0, 0.0, 0.0], 0)
    fieldWrittenNow = _field(r0, v0, macros.sec2nano(STEP))

    np.testing.assert_allclose(fieldFromVelocity, fieldAdvanced, rtol=1e-12)
    assert np.linalg.norm(fieldFromVelocity) < np.linalg.norm(fieldStatic)
    np.testing.assert_allclose(fieldWrittenNow, fieldStatic, rtol=1e-12)


if __name__ == "__main__":
    test_field_is_evaluated_at_extrapolated_position()
