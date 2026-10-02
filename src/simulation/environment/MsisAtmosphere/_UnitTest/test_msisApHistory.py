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

"""Test the optional use of the 3-hour Ap history of the MSIS atmosphere model."""

import datetime

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import msisAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros

EPOCH = datetime.datetime(2026, 1, 1, 12)  # UTC
R_SC = [6778137.0, 0.0, 0.0]  # [m] spacecraft position on the equator, planet-fixed frame is the inertial frame
STEP = 1.0  # [s]
F107 = 150.0  # [sfu]
AP_QUIET = 15.0  # [-]


def _density(apValues, useHistory=None):
    """Return the MSIS density for the given Ap input messages.

    Args:
        apValues (list): the 21 Ap values, in the order of the first 21 ``swDataInMsgs``: the daily Ap, the
            3-hour Ap values from the current one to 57 hours earlier.
        useHistory (bool): whether the 3-hour Ap history is used, ``None`` leaves the module default.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = msisAtmosphere.MsisAtmosphere()
    if useHistory is not None:
        atmo.setUseApHistory(useHistory)
    epochMsg = messaging.EpochMsg().write(messaging.EpochMsgPayload(
        year=EPOCH.year, month=EPOCH.month, day=EPOCH.day, hours=EPOCH.hour, minutes=EPOCH.minute,
        seconds=EPOCH.second))
    atmo.epochInMsg.subscribeTo(epochMsg)

    swMsgs = []
    for c in range(23):
        value = F107 if c >= 21 else apValues[c]
        swMsgs.append(messaging.SwDataMsg().write(messaging.SwDataMsgPayload(dataValue=value)))
        atmo.swDataInMsgs[c].subscribeTo(swMsgs[-1])

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.J20002Pfix = np.eye(3).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)
    atmo.planetPosInMsg.subscribeTo(planetMsg)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = list(R_SC)
    scMsg = messaging.SCStatesMsg().write(payload)
    atmo.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", atmo)
    recorder = atmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    return recorder.neutralDensity[-1]


def _storm(daily=105.0, current=300.0):
    """Return the 21 Ap values of a storm that starts in the current 3-hour interval.

    Args:
        daily (float): daily Ap.
        current (float): current 3-hour Ap, the other 3-hour values are quiet.
    """
    values = [AP_QUIET] * 21
    values[0] = daily
    values[1] = current
    return values


def test_ap_history_setter_and_getter():
    """Verify the 3-hour Ap history option defaults to off and round-trips."""
    atmo = msisAtmosphere.MsisAtmosphere()
    assert atmo.getUseApHistory() is False
    atmo.setUseApHistory(True)
    assert atmo.getUseApHistory() is True
    atmo.setUseApHistory(False)
    assert atmo.getUseApHistory() is False



def test_ap_history_ignores_daily_ap():
    """Verify the daily Ap only matters when the history is off.

    With the history the density must not change with the daily Ap, and without the history it must."""
    quietDaily = _storm(daily=AP_QUIET)
    stormDaily = _storm(daily=105.0)
    assert _density(quietDaily, True) == pytest.approx(_density(stormDaily, True), rel=1e-12, abs=0.0)
    assert _density(quietDaily, False) != pytest.approx(_density(stormDaily, False), rel=1e-6, abs=0.0)


def test_ap_history_follows_current_three_hour_ap():
    """Verify the current 3-hour Ap only matters when the history is on, and then raises the density."""
    quiet = [AP_QUIET] * 21
    stormStart = _storm(daily=AP_QUIET, current=300.0)
    assert _density(stormStart, False) == pytest.approx(_density(quiet, False), rel=1e-12, abs=0.0)
    assert _density(stormStart, True) > _density(quiet, True) * (1.0 + 1e-3)


def test_ap_history_default_is_daily_ap():
    """Verify a module that never calls the setter uses the daily Ap, and that the history gives another density."""
    values = _storm()
    assert _density(values) == _density(values, False)
    assert _density(values) != pytest.approx(_density(values, True), rel=1e-6, abs=0.0)


if __name__ == "__main__":
    test_ap_history_setter_and_getter()
    test_ap_history_ignores_daily_ap()
    test_ap_history_follows_current_three_hour_ap()
    test_ap_history_default_is_daily_ap()
