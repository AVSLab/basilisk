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

"""Test that the wind base class evaluates the spacecraft state extrapolated to the step midpoint."""

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import zeroWindModel
from Basilisk.utilities import SimulationBaseClass, macros

STEP = 10.0  # [s] module update period
OMEGA_PLANET = np.array([0.0, 0.0, 7.2921150e-5])  # [rad/s]


def _airVelocity(r, v, timeNanos):
    """Return the co-rotating air velocity at the third module update.

    Args:
        r (list): [m] spacecraft position written to the state message.
        v (list): [m/s] spacecraft velocity written to the state message.
        timeNanos (int): [ns] time at which the state message is written.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    wind = zeroWindModel.ZeroWindModel()
    wind.setExtrapolateScStateToStepMidpoint(True)
    wind.setPlanetOmega_N(OMEGA_PLANET)
    wind.setUseSpiceOmegaFlag(False)
    planetMsg = messaging.SpicePlanetStateMsg().write(messaging.SpicePlanetStateMsgPayload())
    wind.planetPosInMsg.subscribeTo(planetMsg)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(r)
    payload.v_BN_N = list(v)
    scMsg = messaging.SCStatesMsg().write(payload, timeNanos)
    wind.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", wind)
    recorder = wind.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    # the spacecraft rewrites its state one step later, so that the module observes the spacecraft task period
    scMsg.write(payload, timeNanos + macros.sec2nano(STEP))
    scSim.ConfigureStopTime(macros.sec2nano(2 * STEP))
    scSim.ExecuteSimulation()
    return np.array(recorder.v_air_N[-1])


def test_air_velocity_is_evaluated_at_extrapolated_position():
    """Verify the co-rotating air velocity uses the state advanced by half of the message age.

    A state with a velocity along x, written at the start of the simulation and
    rewritten one step later, must give the air velocity of the
    position advanced by v * dt / 2, which differs from the static one along y. The same state written at the
    current time is not advanced."""
    r0 = [6.778e6, 0.0, 0.0]  # [m]
    v0 = [0.0, 7.5e3, 0.0]  # [m/s]
    shift = np.array(v0) * STEP / 2.0  # [m]

    airFromVelocity = _airVelocity(r0, v0, 0)
    airAdvanced = _airVelocity(list(np.array(r0) + shift), [0.0, 0.0, 0.0], macros.sec2nano(STEP))
    airStatic = _airVelocity(r0, [0.0, 0.0, 0.0], 0)
    airWrittenNow = _airVelocity(r0, v0, macros.sec2nano(STEP))

    np.testing.assert_allclose(airFromVelocity, airAdvanced, atol=1e-9)
    assert abs(airFromVelocity[0] - airStatic[0]) > 1.0  # [m/s] the shift changes the x component
    np.testing.assert_allclose(airWrittenNow, airStatic, atol=1e-9)


def _airVelocityWithPlanet(planetVelocity):
    """Return the co-rotating air velocity at the third update for a spacecraft that moves together with the planet.

    The planet message is written at the current time and the spacecraft message one step earlier, so the relative
    position is only constant if both are evaluated at the same epoch.

    Args:
        planetVelocity (list): [m/s] common velocity of the spacecraft and the planet.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    wind = zeroWindModel.ZeroWindModel()
    wind.setExtrapolateScStateToStepMidpoint(True)
    wind.setPlanetOmega_N(OMEGA_PLANET)
    wind.setUseSpiceOmegaFlag(False)
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PositionVector = (np.array(planetVelocity) * STEP).tolist()  # [m]
    planetPayload.VelocityVector = list(planetVelocity)
    planetPayload.J20002Pfix = np.eye(3).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload, macros.sec2nano(2 * STEP))
    wind.planetPosInMsg.subscribeTo(planetMsg)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [6.778e6, 0.0, 0.0]  # [m]
    payload.v_BN_N = list(planetVelocity)
    scMsg = messaging.SCStatesMsg().write(payload, 0)
    wind.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", wind)
    recorder = wind.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    # the spacecraft rewrites its state one step later, so that the module observes the spacecraft task period
    scMsg.write(payload, 0 + macros.sec2nano(STEP))
    scSim.ConfigureStopTime(macros.sec2nano(2 * STEP))
    scSim.ExecuteSimulation()
    return np.array(recorder.v_air_N[-1])


def test_translating_planet_does_not_change_the_air_velocity():
    """Verify the spacecraft and the planet are evaluated at the same epoch.

    The spacecraft and the planet move together at 30 km/s, so the relative position and the air velocity are
    constant."""
    np.testing.assert_allclose(_airVelocityWithPlanet([0.0, 3.0e4, 0.0]), _airVelocityWithPlanet([0.0, 0.0, 0.0]),
                               atol=1e-9)


if __name__ == "__main__":
    test_air_velocity_is_evaluated_at_extrapolated_position()
