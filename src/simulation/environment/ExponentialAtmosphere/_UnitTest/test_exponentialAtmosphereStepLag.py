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

"""Test that the atmosphere base class evaluates the spacecraft state extrapolated to the step midpoint."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simSetPlanetEnvironment

STEP = 10.0  # [s] module update period


def _density(r, v, timeNanos):
    """Return the exponential atmosphere density at the second module update.

    Args:
        r (list): [m] spacecraft position written to the state message.
        v (list): [m/s] spacecraft velocity written to the state message.
        timeNanos (int): [ns] time at which the state message is written.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(r)
    payload.v_BN_N = list(v)
    scMsg = messaging.SCStatesMsg().write(payload, timeNanos)
    atmo.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1]


def test_density_is_evaluated_at_extrapolated_position():
    """Verify the density uses the state advanced by half of the message age.

    A state written at the start of the simulation with a radial velocity must give the density of
    the position advanced by v * dt / 2. The same state written at the current time, or a
    zero velocity, is not advanced."""
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    v0 = [1.0e3, 0.0, 0.0]  # [m/s] radial
    shift = v0[0] * STEP / 2.0  # [m]

    densityFromVelocity = _density(r0, v0, 0)
    densityAdvanced = _density([r0[0] + shift, 0.0, 0.0], [0.0, 0.0, 0.0], macros.sec2nano(STEP))
    densityStatic = _density(r0, [0.0, 0.0, 0.0], 0)
    densityWrittenNow = _density(r0, v0, macros.sec2nano(STEP))

    assert densityFromVelocity == pytest.approx(densityAdvanced, rel=1e-12, abs=0.0)
    assert densityFromVelocity < densityStatic
    assert densityWrittenNow == pytest.approx(densityStatic, rel=1e-12, abs=0.0)


if __name__ == "__main__":
    test_density_is_evaluated_at_extrapolated_position()
