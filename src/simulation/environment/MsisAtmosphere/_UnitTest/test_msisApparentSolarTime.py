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

"""Test the optional apparent solar time of the MSIS atmosphere model."""

import datetime
import math

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import msisAtmosphere
from Basilisk.utilities import SimulationBaseClass, macros

EPOCH = datetime.datetime(2026, 1, 1)  # UTC
R_SC = [6778137.0, 0.0, 0.0]  # [m] spacecraft position on the equator, planet-fixed frame is the inertial frame
STEP = 1.0  # [s]


def equation_of_time(date):
    """Return the equation of time in seconds with the series of Meeus, Astronomical Algorithms, chapter 28.

    Args:
        date (datetime.datetime): UTC date and time.
    """
    julianDate = (date - datetime.datetime(1970, 1, 1)).total_seconds() / 86400.0 + 2440587.5
    T = (julianDate - 2451545.0) / 36525.0
    L0 = math.radians((280.46646 + 36000.76983 * T + 0.0003032 * T * T) % 360.0)
    M = math.radians(357.52911 + 35999.05029 * T - 0.0001537 * T * T)
    e = 0.016708634 - 0.000042037 * T
    y = math.tan(math.radians(23.439291 - 0.0130042 * T) / 2.0) ** 2
    E = (y * math.sin(2 * L0) - 2 * e * math.sin(M) + 4 * e * y * math.sin(M) * math.cos(2 * L0)
         - 0.5 * y * y * math.sin(4 * L0) - 1.25 * e * e * math.sin(2 * M))
    return math.degrees(E) * 240.0


def _density(epoch, useApparent):
    """Return the MSIS density for the given epoch.

    Args:
        epoch (datetime.datetime): UTC epoch given to the module.
        useApparent (bool): whether the apparent solar time is used.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    atmo = msisAtmosphere.MsisAtmosphere()
    atmo.setUseApparentSolarTime(useApparent)
    epochMsg = messaging.EpochMsg().write(messaging.EpochMsgPayload(
        year=epoch.year, month=epoch.month, day=epoch.day, hours=epoch.hour, minutes=epoch.minute,
        seconds=epoch.second + epoch.microsecond * 1e-6))
    atmo.epochInMsg.subscribeTo(epochMsg)

    swMsgs = []
    for c in range(23):
        value = 150.0 if c >= 21 else 15.0  # f107 [sfu] and ap [-]
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


def test_apparent_solar_time_setter_and_getter():
    """Verify the apparent solar time option defaults to off and round-trips."""
    atmo = msisAtmosphere.MsisAtmosphere()
    assert atmo.getUseApparentSolarTime() is False
    atmo.setUseApparentSolarTime(True)
    assert atmo.getUseApparentSolarTime() is True


@pytest.mark.parametrize("epoch", [datetime.datetime(2026, 1, 1, 6), datetime.datetime(2026, 4, 10, 18)])
def test_apparent_solar_time_matches_mean_time_shifted_by_equation_of_time(epoch):
    """Verify the apparent solar time adds the equation of time to the local solar time.

    The density with the option on must equal the density with the option off for an epoch shifted
    by the equation of time, because only the local solar time input of the model differs."""
    eot = equation_of_time(epoch)  # [s]
    shifted = epoch + datetime.timedelta(seconds=eot)

    apparent = _density(epoch, True)
    meanShifted = _density(shifted, False)
    mean = _density(epoch, False)

    assert apparent == pytest.approx(meanShifted, rel=2e-3, abs=0.0)
    assert apparent != pytest.approx(mean, rel=1e-6, abs=0.0)


def test_apparent_solar_time_rejects_epoch_before_1970():
    """Verify the apparent solar time raises an error for an epoch before 1970, where the series is not valid,
    while the mean solar time still works."""
    epoch = datetime.datetime(1969, 7, 20, 12)  # UTC
    assert _density(epoch, False) > 0.0
    with pytest.raises(BasiliskError):
        _density(epoch, True)


if __name__ == "__main__":
    test_apparent_solar_time_setter_and_getter()
    test_apparent_solar_time_matches_mean_time_shifted_by_equation_of_time(datetime.datetime(2026, 1, 1, 6))
    test_apparent_solar_time_rejects_epoch_before_1970()
