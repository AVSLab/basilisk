import os
import pathlib
import pytest
import numpy as np
import Basilisk.utilities.tleHandling as tleHandling
import Basilisk.utilities.orbitalMotion as om
from datetime import datetime, timedelta, timezone

A_TOL = 1e-14 #[-]
DATA_DIR = pathlib.Path(__file__).parent / 'data'

###################################################################
# Expected orbital elements for oneWeb (if testing multiple satellites)
EXPECTED_OE_ONE_WEB = {
    'a': [
        7575817.78380046,  # 0
        7575897.531671279, # 1
        7575896.775957426, # 2
        7579911.683283668, # 3
        7579911.241546285, # 4
        7579912.336286875 # 5
    ],
    'e': [
        0.000175,   0.0001764, 0.0002086, 0.0001581, 0.0001714,
        0.0002029
    ],
    'i': [
        1.534083325979196,  1.5340868166376997, 1.534083325979196,
        1.5341409218445117, 1.5341496484907717, 1.5341409218445117
    ],
    'f': [
        2.2017240215026246, 4.489410705143804, 4.744271057621696,
        4.549972500184758, 4.511343702990637, 4.567673118903615
    ],
    'Omega': [
        4.888890636980372, 4.888930779553168, 4.889318242647111,
        5.4216838070781765, 5.420380046126938, 5.4207134040140685
    ],
    'omega': [
        1.372734617949328, 1.7957483234259417, 1.5408883647327218,
        1.7351853983817387, 1.773814770716129, 1.717484269108012
    ]
}

EXPECTED_OE_2LE = {
    'a': [
        6796012.0083837,
        6763487.620700561,
        6796034.881033647
    ],
    'e': [
        0.0001066,
        0.0001842,
        0.0001045
    ],
    'i': [
        0.9011588713652241,
        0.7236833210469288,
        0.9011606166944762
    ],
    'f': [
        3.019032029543272,
        3.4280122815833245,
        3.0304383654813503
    ],
    'Omega': [
        2.3957855389445846,
        1.6320242222841097,
        2.4180821201388114
    ],
    'omega': [
        3.2658775496243098,
        2.8566379826044352,
        3.2544735682917785
    ]
}

# Values to generate TLE for HYPSO 1
oeHypso1 = om.ClassicElements()
oeHypso1.i = np.deg2rad(97.3197)
oeHypso1.e = 0.0004257 # [-]
oeHypso1.a = (om.RP_EARTH + 445.195971181338)*1e3 # [m]
oeHypso1.Omega = np.deg2rad(349.0390) # [rad]
oeHypso1.omega = np.deg2rad(136.0807) # [rad]
oeHypso1.f = np.deg2rad(224.04427854247686) # [rad]
hypso1noradId = 51053 # [-]
hypso1launch = datetime(2022, 1, 13, 0, 0, 0) # [UTC]
hypso1tleEpoch = datetime(2025, 1, 1, tzinfo=timezone.utc) + timedelta(days=279.47924866 - 1) # [UTC]
hypso1nDot = 0.00028852 # [rev/day^2]
hypso1bStar = 0.00056053 # [1/Earth radii]

# Expected orbital elements for Hypso1
EXPECTED_OE_HYPSO = {
    'a': [oeHypso1.a],
    'e': [oeHypso1.e],
    'i': [oeHypso1.i],
    'f': [oeHypso1.f],
    'Omega': [oeHypso1.Omega],
    'omega': [oeHypso1.omega]
}

###################################################################
def _equalCheck(v1, v2, fieldName):
    """Compare two values within tolerance"""
    if abs(v1 - v2) < A_TOL:
        return 0
    print(f'{fieldName} failed: expected {v2}, got {v1}')
    return 1

@pytest.mark.parametrize("tlePath, expectedDict", [
    (DATA_DIR / "hypso1.tle", EXPECTED_OE_HYPSO),
    (DATA_DIR / "oneWeb25.tle", EXPECTED_OE_ONE_WEB),
    (DATA_DIR / "spacestations.2le", EXPECTED_OE_2LE),
])
def test_read_tle(tlePath, expectedDict):
    eCountTle = 0

    # Read TLE file from data folder
    tleDataList = tleHandling.satTle2elem(tlePath)

    # Check each orbital element
    for idx, _ in enumerate(tleDataList):
        for key in expectedDict:
            actualValue = getattr(tleDataList[idx].oe, key)
            eCountTle += _equalCheck(actualValue, expectedDict[key][idx], f'{tlePath}_{key}')

    assert eCountTle < 1, f"{eCountTle} functions failed in tleHandling.py script, satTLE2Elem() method"

@pytest.mark.parametrize("satName, orbitalElements, noradID, launchDate, launch_Noyear, PoL, tleEpoch, nDot, bStar, expectedTlePath", [
    ('HYPSO 1', oeHypso1, hypso1noradId, hypso1launch, 2, 'BX', hypso1tleEpoch, hypso1nDot, hypso1bStar, DATA_DIR / "hypso1.tle"),
])
def test_write_tle(satName, orbitalElements, noradID, launchDate, launch_Noyear, PoL, tleEpoch, nDot, bStar, expectedTlePath):
    eCountTle = 0

    # Make the data class
    tleData = tleHandling.TleData(oe = orbitalElements, tleEpoch = tleEpoch)
    # fill in optional data
    tleData.satName = satName
    tleData.noradID = noradID
    tleData.launchDate = launchDate
    tleData.launchNo = launch_Noyear
    tleData.pol = PoL
    tleData.nDot = nDot
    tleData.nDotDot = 0.0
    tleData.bStar = bStar
    generatedTle = tleHandling.generateTle(tleData)

    # Read the actual TLE file
    with open(expectedTlePath, 'r') as file:
        expectedTLE = file.read()
    # String compaire (line 0)
    eCountTle += int((generatedTle.splitlines()[0] != expectedTLE.splitlines()[0]))
    # String compaire (line 1)
    eCountTle += int((generatedTle.splitlines()[1] != expectedTLE.splitlines()[1]))
    # String compaire (line 2)
    eCountTle += int((generatedTle.splitlines()[2][0:63] != expectedTLE.splitlines()[2][0:63])) # Ignore revolutions count and checksum
    assert eCountTle < 1, f"{eCountTle} functions failed in tleHandling.py script, generateTleDataString() method"

@pytest.mark.parametrize("tlePath", [
    (DATA_DIR / "hypso1.tle"),
])
def test_read_write_tle(tlePath):
    eCountTle = 0
    # Read TLE file
    satTle2elem = tleHandling.satTle2elem(tlePath)

    generatedTle = tleHandling.generateTle(satTle2elem[0])
    # Read the actual TLE file
    with open(tlePath, 'r') as file:
        tleToBeTested = file.read()

    # String compaire (line 0)
    eCountTle += int((generatedTle.splitlines()[0] != tleToBeTested.splitlines()[0]))
    # String compaire (line 1)
    eCountTle += int((generatedTle.splitlines()[1] != tleToBeTested.splitlines()[1]))
    # String compaire (line 2)
    eCountTle += int((generatedTle.splitlines()[2][0:63] != tleToBeTested.splitlines()[2][0:63])) # Ignore revolutions count and checksum
    assert eCountTle < 1, f"{eCountTle} functions failed in tleHandling.py script, generateTleDataString() method"

if __name__ == "__main__":
    # Reading TLE
    test_read_tle(os.path.join(DATA_DIR, "hypso1.tle"), EXPECTED_OE_HYPSO)
    test_read_tle(DATA_DIR / "oneWeb25.tle", EXPECTED_OE_ONE_WEB)
    test_read_tle(DATA_DIR / "spacestations.2le", EXPECTED_OE_2LE)

    # Writing TLE
    test_write_tle('HYPSO 1', oeHypso1, hypso1noradId, hypso1launch, launch_Noyear=2, PoL='BX', tleEpoch=hypso1tleEpoch, nDot=hypso1nDot, bStar=hypso1bStar, expectedTlePath=DATA_DIR / "hypso1.tle")

    # Take TLE, generate orbital elements, then generate TLE again and compare to original
    test_read_write_tle(DATA_DIR / "hypso1.tle")
