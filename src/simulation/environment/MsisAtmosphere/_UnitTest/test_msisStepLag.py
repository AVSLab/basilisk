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

"""Test that the MSIS temporal inputs are evaluated at the epoch of the extrapolated spacecraft state."""

import datetime

import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import msisAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros

EPOCH = datetime.datetime(2026, 1, 1)  # UTC
R_SC = [6778137.0, 0.0, 0.0]  # [m] spacecraft position on the equator, planet-fixed frame is the inertial frame
STEP = 3600.0  # [s] module update period, long enough for the local solar time to change visibly


def _density(epoch, extrapolate, stopSeconds, rewriteTimes=()):
    """Return the MSIS density of the last module update.

    The spacecraft is at rest, so the extrapolation moves it nowhere and only the epoch of the temporal inputs differs.

    Args:
        epoch (datetime.datetime): UTC epoch given to the module.
        extrapolate (bool): whether the extrapolation to the step midpoint is enabled.
        stopSeconds (float): [s] simulation stop time.
        rewriteTimes (tuple): [s] times at which the spacecraft state message is rewritten after the update, emulating
            a spacecraft that runs after the module in the task with the same period.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = msisAtmosphere.MsisAtmosphere()
    atmo.setExtrapolateScStateToStepMidpoint(extrapolate)
    epochMsg = messaging.EpochMsg().write(messaging.EpochMsgPayload(
        year=epoch.year, month=epoch.month, day=epoch.day, hours=epoch.hour, minutes=epoch.minute,
        seconds=epoch.second))
    atmo.epochInMsg.subscribeTo(epochMsg)

    swMsgs = []
    for c in range(23):
        value = 150.0 if c >= 21 else 15.0  # f107 [sfu] and ap [-]
        swMsgs.append(messaging.SwDataMsg().write(messaging.SwDataMsgPayload(dataValue=value)))
        atmo.swDataInMsgs[c].subscribeTo(swMsgs[-1])

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)
    atmo.planetPosInMsg.subscribeTo(planetMsg)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(R_SC)
    scMsg = messaging.SCStatesMsg().write(payload, 0)
    atmo.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    for writeTime in rewriteTimes:
        scSim.ConfigureStopTime(macros.sec2nano(writeTime))
        scSim.ExecuteSimulation()
        scMsg.write(payload, macros.sec2nano(writeTime))
    scSim.ConfigureStopTime(macros.sec2nano(stopSeconds))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1]


def test_temporal_inputs_follow_the_extrapolated_midpoint():
    """Verify the local solar time uses the same epoch as the extrapolated spacecraft state.

    The state is written at 0 s and rewritten at 3600 s, so the update at 7200 s is extrapolated to 5400 s. Its density
    must be that of a plain update at the epoch advanced by 5400 s, and differ from the one at 7200 s."""
    midpoint = _density(EPOCH, True, 2 * STEP, rewriteTimes=(STEP,))
    atMidpointEpoch = _density(EPOCH + datetime.timedelta(seconds=1.5 * STEP), False, 0.0)
    atUpdateEpoch = _density(EPOCH + datetime.timedelta(seconds=2 * STEP), False, 0.0)

    assert midpoint == pytest.approx(atMidpointEpoch, rel=1e-12, abs=0.0)
    assert midpoint != pytest.approx(atUpdateEpoch, rel=1e-3, abs=0.0)


def test_temporal_inputs_are_unchanged_without_extrapolation():
    """Verify the temporal inputs stay at the update epoch when the extrapolation is disabled."""
    disabled = _density(EPOCH, False, 2 * STEP, rewriteTimes=(STEP,))
    atUpdateEpoch = _density(EPOCH + datetime.timedelta(seconds=2 * STEP), False, 0.0)

    assert disabled == pytest.approx(atUpdateEpoch, rel=1e-12, abs=0.0)


def test_temporal_inputs_use_the_previous_update_while_the_period_is_unknown():
    """Verify the startup update, where the spacecraft is left at the previous update, uses that epoch as well.

    The state is written at 0 s. The update at 3600 s has seen one write time only, so the geometry stays at 0 s and the
    temporal inputs must be those of 0 s."""
    startup = _density(EPOCH, True, STEP, rewriteTimes=())
    atPreviousEpoch = _density(EPOCH, False, 0.0)

    assert startup == pytest.approx(atPreviousEpoch, rel=1e-12, abs=0.0)
