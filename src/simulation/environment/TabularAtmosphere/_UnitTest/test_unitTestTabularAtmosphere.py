#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        tabularAtmosphere
#   Author:             Mikaela Felix
#   Creation Date:      Feb 11, 2022
#

import os

import numpy as np
import pytest
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import tabularAtmosphere
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities.readAtmTable import readAtmTable
from Basilisk.utilities.supportDataTools.dataFetcher import (
    get_path,
    DataFile,
)


@pytest.mark.parametrize("planet, row_count, density, temperature", [
    # Surface density [kg/m^3] and temperature [K] from the nominal GRAM files.
    ("Venus", 401, 64.79, 735.3),
    ("Uranus", 2001, 0.3788, 76.4),
    ("Titan", 2001, 5.475, 93.9),
    ("Jupiter", 2001, 0.1740, 166.1),
])
def test_gram_surface_row(planet, row_count, density, temperature):
    """Read the complete nominal profile, including the surface sample."""
    file = DataFile.AtmosphereData[f"{planet}GRAMNominal"]
    alt, rho, temp = readAtmTable(get_path(file), f"{planet}GRAM")
    assert len(alt) == len(rho) == len(temp) == row_count
    assert (alt[0], rho[0], temp[0]) == (0.0, density, temperature)  # altitude [m]


@pytest.mark.parametrize("csv_rows", [
    "Height_km,Density_kgm3,Temperature_K\n[km],[kg/m3],[K]\n1,2,300\n0,3,310\n",
    "Temperature_K,Height_km,Density_kgm3\nKelvin,km,kg/m^3\n300,1,2\n310,0,3\n",
])
def test_gram_csv_with_units_row(tmp_path, csv_rows):
    """Read a user-supplied GRAM CSV with a units row and descending altitudes."""
    path = tmp_path / "profile.csv"
    path.write_text(csv_rows, encoding="utf-8")
    expected = ([0.0, 1000.0], [3.0, 2.0], [310.0, 300.0])  # [m], [kg/m^3], [K]
    assert readAtmTable(path, "VenusGRAM") == expected


@pytest.mark.parametrize("first_row", ["bad_height,bad_density,bad_temp", "km,g/cm3,K", "NA,NA,NA"])
def test_gram_csv_rejects_invalid_first_row(tmp_path, first_row):
    """Reject malformed samples and incompatible units even when a valid data row follows."""
    path = tmp_path / "profile.csv"
    path.write_text(f"Height_km,Density_kgm3,Temperature_K\n{first_row}\n0.5,63.15,731.5\n", encoding="utf-8")
    with pytest.raises(ValueError):
        readAtmTable(path, "VenusGRAM")


@pytest.mark.parametrize("rows", [
    "0,64.79,True\n0.5,63.15,False\n",
    "0,64.79,735.3\n0.5,NaN,731.5\n",
    "0,64.79,735.3\ninf,63.15,731.5\n",
], ids=["boolean-temperature", "nan-density", "infinite-height"])
def test_gram_csv_requires_finite_numbers(tmp_path, rows):
    """Reject Boolean fields and nonfinite samples anywhere in a GRAM profile."""
    path = tmp_path / "profile.csv"
    path.write_text("Height_km,Density_kgm3,Temperature_K\n" + rows, encoding="utf-8")
    with pytest.raises(ValueError):
        readAtmTable(path, "VenusGRAM")


def _tabular_model(table):
    model = tabularAtmosphere.TabularAtmosphere()
    file = DataFile.AtmosphereData[f"{table}Nominal"]
    alt, rho, temp = readAtmTable(get_path(file), table)
    model.altList = tabularAtmosphere.DoubleVector(alt)
    model.rhoList = tabularAtmosphere.DoubleVector(rho)
    model.tempList = tabularAtmosphere.DoubleVector(temp)
    return model


@pytest.mark.parametrize("table, altitude, direction, density, temperature", [
    # Altitude [m], radial direction [-], density [kg/m^3], temperature [K].
    ("VenusGRAM", 250.0, [1, 0, 0], 63.97, 733.4),
    ("EarthGRAM", 200000.0, [1, 0, 0], 4.2191e-10, 1240.60),
    ("VenusGRAM", 0.0, [1, 4, 4], 64.79, 735.3),
    ("EarthGRAM", 200000.0, [1, 1, 1], 4.2191e-10, 1240.60),
    ("VenusGRAM", -0.001, [1, 4, 4], 0.0, 0.0),
    ("EarthGRAM", 200000.001, [1, 1, 1], 0.0, 0.0),
])
def test_tabular_atmosphere_boundaries(table, altitude, direction, density, temperature):
    """Check interpolation, off-axis endpoints, and altitudes a millimeter outside the table."""
    model = _tabular_model(table)
    model.planetRadius = {"EarthGRAM": 6378136.6, "VenusGRAM": 6051800.0}[table]  # [m]
    state = messaging.SCStatesMsgPayload()
    state.r_BN_N = np.array(direction) * ((model.planetRadius + altitude) / np.linalg.norm(direction))  # [m]
    model.addSpacecraftToModel(messaging.SCStatesMsg().write(state))
    model.Reset(0)
    model.UpdateState(0)
    atmosphere = model.envOutMsgs[0].read()
    assert atmosphere.neutralDensity == pytest.approx(density, rel=1e-12, abs=0.0)
    assert atmosphere.localTemp == pytest.approx(temperature, rel=1e-12, abs=0.0)


@pytest.mark.parametrize("altitude, density, temperature", [
    (2000000.0, 2.589e-11, 900.5),  # [m], [kg/m^3], [K] at the last GRAM row
    (2000000.001, 0.0, 0.0),        # [m], [kg/m^3], [K] one millimeter above it
])
def test_oblate_jupiter_table_top(altitude, density, temperature):
    """Evaluate the top of the Jupiter profile using height above the reference ellipsoid."""
    model = _tabular_model("JupiterGRAM")
    model.planetRadius = 71492000.0  # [m] equatorial radius
    polar_radius = 66854000.0  # [m]
    model.setPlanetPolarRadius(polar_radius)
    planet = messaging.SpicePlanetStateMsgPayload()
    planet.J20002Pfix = np.eye(3)
    planet_msg = messaging.SpicePlanetStateMsg().write(planet)
    model.planetPosInMsg.subscribeTo(planet_msg)

    latitude, longitude = np.deg2rad([40.0, 37.0])  # [rad]
    eccentricity_sq = 1.0 - (polar_radius / model.planetRadius)**2
    prime_vertical_radius = model.planetRadius / np.sqrt(1.0 - eccentricity_sq * np.sin(latitude)**2)  # [m]
    state = messaging.SCStatesMsgPayload()
    state.r_BN_N = [
        (prime_vertical_radius + altitude) * np.cos(latitude) * np.cos(longitude),
        (prime_vertical_radius + altitude) * np.cos(latitude) * np.sin(longitude),
        (prime_vertical_radius * (1.0 - eccentricity_sq) + altitude) * np.sin(latitude),
    ]  # [m]
    model.addSpacecraftToModel(messaging.SCStatesMsg().write(state))
    model.Reset(0)
    model.UpdateState(0)
    atmosphere = model.envOutMsgs[0].read()
    assert atmosphere.neutralDensity == pytest.approx(density, rel=1e-12, abs=0.0)
    assert atmosphere.localTemp == pytest.approx(temperature, rel=1e-12, abs=0.0)


@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize(
    "altitude", [42.0, 33.33333, 10000.0, -10.0]
)  # exact, interpolate, above, below
@pytest.mark.parametrize("useMinReach", [True, False])
@pytest.mark.parametrize("useMaxReach", [True, False])
def test_tabularAtmosphere(altitude, accuracy, useMinReach, useMaxReach):
    r"""
    **Validation Test Description**

    TabularAtmosphere interpolates from user-provided data to compute density and temperature at the current s/c
    altitude. The unit test checks altitudes at, between, above, and below the values included in the table. This
    test uses a python helper function to provide data from EarthGRAM (see supportData\AtmosphereData\support).
    Data lists can also be manually-input, but check sorting and units per documentation and support info (above).
    The module returns 0 for both density and temperature if ANY ONE of the following conditions is met:

    - altitude below minimum value in provided table
    - altitude above maximum value in provided table
    - altitude below envMinReach
    - altitude above envMaxReach

    Note that this results in nonphysical behavior for temperature (absolute zero) when outside defined range.

    **Test Parameters**

    Args:

    - altitude (float): Spacecraft altitude for which density, temperature are returned
    - accuracy (float): accuracy value used in validation tests
    - useMinReach (bool): set value of envMinReach
    - useMaxReach (bool): set value of envMaxReach

    **Description of Variables Being Tested**

    The unit test checks density (kg/m^3) and temperature (K) against their expected values:

    - ``densData[0]``
    - ``tempData[0]``
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = tabularAtmosphereTestFunction(
        altitude, accuracy, useMinReach, useMaxReach
    )
    assert testResults < 1, testMessage


def tabularAtmosphereTestFunction(altitude, accuracy, useMinReach, useMaxReach):
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = tabularAtmosphere.TabularAtmosphere()  # update with current values
    module.ModelTag = "tabularAtmosphere"  # update python name of test module

    # define constants & load data
    r_eq = 6378136.6
    atm_path = get_path(DataFile.AtmosphereData.EarthGRAMNominal)
    altList, rhoList, tempList = readAtmTable(str(atm_path), "EarthGRAM")

    # assign constants & ref. data to module
    module.planetRadius = r_eq
    module.altList = tabularAtmosphere.DoubleVector(altList)
    module.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    module.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # CHECK - env min and max
    if useMinReach:
        minReach = 50.0 * 1000
        module.envMinReach = minReach
    else:
        minReach = -1.0 * 1000
    if useMaxReach:
        maxReach = 20.0 * 1000
        module.envMaxReach = maxReach
    else:
        maxReach = 5000.0 * 1000

    #   setup orbit and simulation time
    r0 = r_eq + (altitude * 1000.0)  # meters
    oe = orbitalMotion.ClassicElements()
    mu = 0.3986004415e15  # meters^3/s^2
    oe.a = r0
    oe.e = 0.0
    oe.i = 45.0 * macros.D2R
    oe.Omega = 30.0 * macros.D2R
    oe.omega = 120.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    r0N, v0N = orbitalMotion.elem2rv(mu, oe)

    # create the input messages
    scStateMsg = (
        messaging.SCStatesMsgPayload()
    )  # Create a structure for the input message
    scStateMsg.r_BN_N = np.array(r0N)
    scInMsg = messaging.SCStatesMsg().write(scStateMsg)

    # add spacecraft to environment model
    module.addSpacecraftToModel(scInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.envOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    densData = dataLog.neutralDensity
    tempData = dataLog.localTemp

    # define python function to compute truth values
    def tabAtmoComp(val, xList, yList):
        if (val < xList[0]) or (val <= minReach):
            out = 0.0
            return out
        elif (val > xList[-1]) or (val >= maxReach):
            out = 0.0
            return out
        else:
            for i, x in enumerate(xList):
                if x >= val:
                    x0 = xList[i - 1]
                    y0 = yList[i - 1]
                    y1 = yList[i]
                    m = (y1 - y0) / (x - x0)
                    out = y0 + (val - x0) * m
                    return out

    # compute truth values
    trueDensity = tabAtmoComp(altitude * 1000, altList, rhoList)
    trueTemp = tabAtmoComp(altitude * 1000, altList, tempList)
    failures = []
    for name, actual, expected in (("density", densData[0], trueDensity), ("temperature", tempData[0], trueTemp)):
        compare = unitTestSupport.isDoubleEqual if expected == 0 else unitTestSupport.isDoubleEqualRelative
        if not compare(actual, expected, accuracy):
            failures.append(f"{name} expected {expected}, got {actual}")
    return [len(failures), "\n".join(failures)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_tabularAtmosphere(
        10000.0,  # altitude
        1e-12,  # accuracy
        True,
        True,
    )
