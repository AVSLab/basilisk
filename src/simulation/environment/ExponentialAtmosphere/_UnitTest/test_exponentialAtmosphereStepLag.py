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


def _density(r, v, timeNanos, stopSeconds=STEP):
    """Return the exponential atmosphere density at the second module update.

    Args:
        r (list): [m] spacecraft position written to the state message.
        v (list): [m/s] spacecraft velocity written to the state message.
        timeNanos (int): [ns] time at which the state message is written.
        stopSeconds (float): [s] simulation stop time; the density of the last module update is returned.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.setExtrapolateScStateToStepMidpoint(True)
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
    scSim.ConfigureStopTime(macros.sec2nano(stopSeconds))
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


def test_stale_message_is_extrapolated_only_once():
    """Verify a state written once at the start of the simulation is not extrapolated by later updates.

    The module updates at 0, 10 and 20 s. The state written at t = 0 is extrapolated at the first update that
    reads it, but at the update at 20 s it is stale, because it was written before the module's previous update,
    so the density must be that of the written position."""
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    v0 = [1.0e3, 0.0, 0.0]  # [m/s] radial

    densityFirstUpdate = _density(r0, v0, 0, stopSeconds=STEP)
    densityStaleUpdate = _density(r0, v0, 0, stopSeconds=2 * STEP)
    densityStatic = _density(r0, [0.0, 0.0, 0.0], 0, stopSeconds=2 * STEP)

    assert densityFirstUpdate < densityStatic
    assert densityStaleUpdate == pytest.approx(densityStatic, rel=1e-12, abs=0.0)


def _densityWithRewrittenState(moduleStep, writeTimes, stopSeconds, r, v):
    """Return the density of the last module update for a state message that is rewritten during the run.

    The state is re-written with the same position at each time in ``writeTimes``, emulating a spacecraft that is
    updated at a different task period than the atmosphere module.

    Args:
        moduleStep (float): [s] task period of the atmosphere module.
        writeTimes (list): [s] times at which the spacecraft state message is written, in increasing order.
        stopSeconds (float): [s] simulation stop time.
        r (list): [m] spacecraft position written to the state message.
        v (list): [m/s] spacecraft velocity written to the state message.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(moduleStep)))

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.setExtrapolateScStateToStepMidpoint(True)
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(r)
    payload.v_BN_N = list(v)
    scMsg = messaging.SCStatesMsg().write(payload, macros.sec2nano(writeTimes[0]))
    atmo.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    for writeTime in writeTimes[1:]:
        scSim.ConfigureStopTime(macros.sec2nano(writeTime))
        scSim.ExecuteSimulation()
        scMsg.write(payload, macros.sec2nano(writeTime))
    scSim.ConfigureStopTime(macros.sec2nano(stopSeconds))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1]


def test_translating_planet_does_not_change_the_density():
    """Verify the spacecraft and the planet are evaluated at the same epoch.

    The spacecraft and the planet move together at 30 km/s, so the altitude is constant. The planet message is
    written at the current time while the spacecraft message is one step old. Extrapolating only the spacecraft
    would shift the relative position by v * dt / 2 and change the density."""
    speed = 3.0e4  # [m/s] common velocity of the spacecraft and the planet
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]

    def run(moving):
        scSim = SimulationBaseClass.SimBaseClass()
        proc = scSim.CreateNewProcess("p")
        proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))
        atmo = exponentialAtmosphere.ExponentialAtmosphere()
        atmo.setExtrapolateScStateToStepMidpoint(True)
        simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")

        scPayload = messaging.SCStatesMsgPayload()
        scPayload.r_BN_N = r0
        scPayload.v_BN_N = [0.0, speed if moving else 0.0, 0.0]
        scMsg = messaging.SCStatesMsg().write(scPayload, 0)
        atmo.addSpacecraftToModel(scMsg)

        planetPayload = messaging.SpicePlanetStateMsgPayload()
        planetPayload.PositionVector = [0.0, speed * STEP if moving else 0.0, 0.0]
        planetPayload.VelocityVector = [0.0, speed if moving else 0.0, 0.0]
        planetPayload.J20002Pfix = np.eye(3).tolist()
        planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload, macros.sec2nano(STEP))
        atmo.planetPosInMsg.subscribeTo(planetMsg)

        scSim.AddModelToTask("t", atmo)
        recorder = atmo.envOutMsgs[0].recorder()
        scSim.AddModelToTask("t", recorder)
        scSim.InitializeSimulation()
        scSim.ConfigureStopTime(macros.sec2nano(STEP))
        scSim.ExecuteSimulation()
        return recorder.neutralDensity[-1]

    assert run(True) == pytest.approx(run(False), rel=1e-9, abs=0.0)


@pytest.mark.parametrize("moduleStep, writeTimes, stopSeconds", [
    (10.0, [0.0] + [float(t) for t in range(1, 10)], 10.0),  # spacecraft every 1 s, module every 10 s: written at 9 s
    (1.0, [0.0, 10.0], 12.0),  # spacecraft every 10 s, module every 1 s: written at 10 s, module previous at 11 s
])
def test_task_period_mismatch_disables_the_extrapolation(moduleStep, writeTimes, stopSeconds):
    """Verify a spacecraft updated faster or slower than the module is not extrapolated.

    The state written by the spacecraft is not the output of the previous module update, so half of its age is not
    the middle of the module interval. The density must be that of the written position, whichever direction the
    task period mismatch goes."""
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    v0 = [1.0e3, 0.0, 0.0]  # [m/s] radial

    density = _densityWithRewrittenState(moduleStep, writeTimes, stopSeconds, r0, v0)
    densityStatic = _densityWithRewrittenState(moduleStep, writeTimes, stopSeconds, r0, [0.0, 0.0, 0.0])

    assert density == pytest.approx(densityStatic, rel=1e-12, abs=0.0)


def _densityOfFirstOfTwoSpacecraft(secondWriteSeconds, r, v):
    """Return the density at the first module update after STEP seconds for the first of two spacecraft.

    Args:
        secondWriteSeconds (float): [s] time at which the state message of the second spacecraft is written.
        r (list): [m] position of both spacecraft.
        v (list): [m/s] velocity of the first spacecraft; the second one is at rest.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.setExtrapolateScStateToStepMidpoint(True)
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    payloads = []
    for velocity in (v, [0.0, 0.0, 0.0]):
        payload = messaging.SCStatesMsgPayload()
        payload.r_BN_N = list(r)
        payload.v_BN_N = list(velocity)
        payloads.append(payload)
    firstMsg = messaging.SCStatesMsg().write(payloads[0], 0)
    secondMsg = messaging.SCStatesMsg().write(payloads[1], macros.sec2nano(secondWriteSeconds))
    atmo.addSpacecraftToModel(firstMsg)
    atmo.addSpacecraftToModel(secondMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1]


def test_one_mismatched_spacecraft_disables_the_extrapolation_of_all():
    """Verify the extrapolation is applied to all spacecraft of a module or to none.

    The planet state is shared by all spacecraft, so moving it for one spacecraft would make the geometry of another
    one inconsistent. With both messages written at the previous module update (0 s) the first spacecraft is
    extrapolated. If the second message was written at 5 s, which is not the previous module update, the first
    spacecraft is not extrapolated either."""
    r0 = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    v0 = [1.0e3, 0.0, 0.0]  # [m/s] radial

    densityMatched = _densityOfFirstOfTwoSpacecraft(0.0, r0, v0)
    densityMismatched = _densityOfFirstOfTwoSpacecraft(STEP / 2.0, r0, v0)
    densityStatic = _densityOfFirstOfTwoSpacecraft(STEP / 2.0, r0, [0.0, 0.0, 0.0])

    assert densityMatched < densityStatic
    assert densityMismatched == pytest.approx(densityStatic, rel=1e-12, abs=0.0)


if __name__ == "__main__":
    test_density_is_evaluated_at_extrapolated_position()
    test_stale_message_is_extrapolated_only_once()
    test_translating_planet_does_not_change_the_density()
    test_one_mismatched_spacecraft_disables_the_extrapolation_of_all()
