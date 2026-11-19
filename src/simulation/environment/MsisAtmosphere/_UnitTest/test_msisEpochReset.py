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

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import msisAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros


def _atmosphere_after_resets(epoch_days, use_epoch_message=False):
    """Return density and temperature after each initialization with fixed inputs."""
    model = msisAtmosphere.MsisAtmosphere()
    messages = []
    for index in range(23):
        payload = messaging.SwDataMsgPayload()
        payload.dataValue = 150.0 if index >= 21 else 15.0  # [sfu] F10.7; [-] Ap
        messages.append(messaging.SwDataMsg().write(payload))
        model.swDataInMsgs[index].subscribeTo(messages[-1])
    state = messaging.SCStatesMsgPayload()
    state.r_BN_N = [model.planetRadius + 400e3, 0.0, 0.0]  # [m]
    messages.append(messaging.SCStatesMsg().write(state))
    model.addSpacecraftToModel(messages[-1])
    if use_epoch_message:
        epoch = messaging.EpochMsgPayload(year=2020, month=6, day=10, hours=12, minutes=30, seconds=0.0)  # UTC
        messages.append(messaging.EpochMsg().write(epoch))
        model.epochInMsg.subscribeTo(messages[-1])
    sim = SimulationBaseClass.SimBaseClass()
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(1.0)))  # [s]
    sim.AddModelToTask("task", model)
    results = []
    for epoch_day in epoch_days:
        model.epochDoy = epoch_day
        sim.InitializeSimulation()
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        output = model.envOutMsgs[0].read()
        results.append([output.neutralDensity, output.localTemp])  # [kg/m^3], [K]
    return np.array(results)


@pytest.mark.parametrize("epoch_day", [-1, 1, 60, 365, 366])  # [day]; -1 selects the default epoch.
def test_msis_epoch_reset(epoch_day):
    """Preserve the configured epoch over repeated initialization, including year rollover."""
    results = _atmosphere_after_resets([epoch_day]*3)
    expected = _atmosphere_after_resets([epoch_day])[0]
    assert np.all(expected > 0.0)
    for result in results:
        np.testing.assert_allclose(result, expected, rtol=1e-14, atol=0)


@pytest.mark.parametrize("next_day", [-1, 0, 1, 119])  # [day]; nonpositive values select the default epoch.
def test_msis_epoch_change_on_reset(next_day):
    """Apply a changed day or the default epoch independently of the previous month."""
    results = _atmosphere_after_resets([60, next_day])  # [day]
    expected = _atmosphere_after_resets([next_day])[0]
    np.testing.assert_allclose(results[-1], expected, rtol=1e-14, atol=0)


def test_msis_epoch_message_precedence_on_reset():
    """Keep a linked epoch message authoritative when the day-of-year setting changes."""
    results = _atmosphere_after_resets([60, 365, -1], use_epoch_message=True)  # [day]
    expected = _atmosphere_after_resets([-1], use_epoch_message=True)[0]
    for result in results:
        np.testing.assert_allclose(result, expected, rtol=1e-14, atol=0)
