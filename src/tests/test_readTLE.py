import os
import pathlib
import pytest
import numpy as np
import Basilisk.utilities.tleHandling as tleHandling
import Basilisk.utilities.orbitalMotion as om
from datetime import datetime, timedelta, timezone

A_TOL_DIST = 0.1 #[-] 1dm tolerance for orbital elements that are distance units
A_TOL  = 1e-5 #[-] Tolerance for all metrics except meters
A_TOL_ROUNDTRIP = 1e-4 #[-] Tolerance for eccentricity (roundtrip error)
A_TOL_DEG_ROUNDTRIP = 1.5  # [deg] Tolerance for angles in degrees (roundtrip error)
A_TOL_ECC_ROUNDTRIP = 1e3 #[-] Tolerance for eccentricity (roundtrip error) -> leading decimal point assumed in TLE, so physical eccentricity tolerance is 1e-4

ROOT_DIR = pathlib.Path(__file__).parent.parent.parent
DATA_DIR = ROOT_DIR / "examples" / "MultiSatBskSim" / "tleData"

###################################################################
# Expected orbital elements for oneWeb (if testing multiple satellites)
EXPECTED_OE_ONE_WEB = {
    'a': [
              np.float64(7578544.80840588),
              np.float64(7581725.314320893),
              np.float64(7581725.090937277),
              np.float64(7585736.508260437),
              np.float64(7585735.9677996775),
              np.float64(7585737.120163768)
         ],
    'e': [
              np.float64(0.0008817590462989669),
              np.float64(0.001210093768077448),
              np.float64(0.001258633728769965),
              np.float64(0.0011981313722308415),
              np.float64(0.001206952489243135),
              np.float64(0.001239828742668404)
         ],
    'i': [
              1.531618126589653,
              1.5316291459001932,
              1.5316258452568352,
              1.5322241554919225,
              1.5322307739504486,
              1.5322225935006697
             ],
    'Omega': [
              4.8831623958720565,
              4.883186443196075,
              4.883574022425417,
              5.415984679757847,
              5.414680762509103,
              5.415014177551135
             ],
    'omega': [
              1.4377465022195557,
              1.2801901375990692,
              1.2537640336417852,
              1.2645835093359827,
              1.2744200510794836,
              1.278331704552413
         ],
    'f': [
              2.1381866215235528,
              5.002606275268809,
              5.029031746586543,
              5.017006771870916,
              5.0071733437393355,
              5.003259952527291
          ],
}

EXPECTED_OE_2LE = {
    'a': [np.float64(6802505.842859207), np.float64(6770009.6048538275), np.float64(6802528.7123854)],
    'e': [np.float64(0.0011264735370325686), np.float64(0.0011530055595853114), np.float64(0.001128835473051786)],
    'i': [0.903248948599224, 0.7265482390037918, 0.9032099423215613],
    'Omega': [2.3886009471482503, 1.6261518262462817, 2.410867356948434],
    'omega': [0.8571434002531804, 0.7478010900864719, 0.8567388460152983],
    'f': [5.428341410047342, 5.535537401708238, 5.428797206538743],
}

# Values to generate TLE for HYPSO 1
oeHypso1 = om.ClassicElements()
oeHypso1.i = 1.6979291907417804
oeHypso1.e = 0.0013996087479637449
oeHypso1.a = 6808434.893655814
oeHypso1.Omega = 6.085804103135117
oeHypso1.omega = 1.43544167146666
oeHypso1.f = 4.845270389715793
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
def _equalCheck(v1, v2, fieldName, tol):
    """Compare two values within tolerance"""
    if abs(v1 - v2) < tol:
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
            if key in ['a']:
                eCountTle += _equalCheck(actualValue, expectedDict[key][idx], f'{tlePath}_{key}', A_TOL_DIST)
            elif key in ['e', 'i', 'Omega', 'omega', 'f']:
                eCountTle += _equalCheck(actualValue, expectedDict[key][idx], f'{tlePath}_{key}', A_TOL)

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
    # String compare (line 0)
    eCountTle += int((generatedTle.splitlines()[0] != expectedTLE.splitlines()[0]))
    # String compare (line 1)
    eCountTle += int((generatedTle.splitlines()[1] != expectedTLE.splitlines()[1]))
    # String compare (line 2)
    for i, (field_expected, field_generated) in enumerate(zip(expectedTLE.splitlines()[2].split(), generatedTle.splitlines()[2].split())):
        if i in [4]: # [4] Eccentricity (leading decimal point assumed)
            # Max tolerance is A_TOL_ROUNDTRIP * 1e7 => 1e-4 physical eccentricity tolerance
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_ECC_ROUNDTRIP
        elif i in [5,6]: # [5] omega, [6] Mean anomaly (degrees)
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_DEG_ROUNDTRIP
        else: # [0] Line number, [1] Satellite Number, [2] Inclination, [3] Omega, [7] Mean motion
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_ROUNDTRIP
        #NOTE: [8] Revolution number at epoch and [9] Checksum are not tested (these values cannot be known by basilisk)

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

    # String compare (line 0)
    eCountTle += int((generatedTle.splitlines()[0] != tleToBeTested.splitlines()[0]))
    # String compare (line 1)
    eCountTle += int((generatedTle.splitlines()[1] != tleToBeTested.splitlines()[1]))
    # String compare (line 2)
    for i, (field_expected, field_generated) in enumerate(zip(tleToBeTested.splitlines()[2].split(), generatedTle.splitlines()[2].split())):
        if i in [4]: # 4 = eccentricity (leading decimal point assumed), 5 = argument of perigee, 6 = mean anomaly
            # Eccentricity -> leading decimal point assume -> max tolerance is A_TOL_ROUNDTRIP * 1e7 => 1e-4 physical eccentricity tolerance
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_ROUNDTRIP * 1e7
        elif i in [5,6]: # 5 = argument of perigee, 6 = mean anomaly (degrees)
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_DEG_ROUNDTRIP
        else:
            eCountTle += abs(float(field_expected) - float(field_generated)) > A_TOL_ROUNDTRIP

    assert eCountTle < 1, f"{eCountTle} functions failed in tleHandling.py script, generateTleDataString() method"

def _write_modified_hypso_tle(tmp_path, line1=None, line2=None):
    """Create a temporary TLE file based on hypso1.tle with optional line overrides."""
    src = DATA_DIR / "hypso1.tle"
    with open(src, 'r') as file:
        lines = file.read().splitlines()
    if line1 is not None:
        lines[1] = line1
    if line2 is not None:
        lines[2] = line2
    out_path = tmp_path / "modified.tle"
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out_path


def test_read_tle_rejects_alpha5_like_id(tmp_path):
    """satTle2elem() rejects Alpha-5-like catalog IDs in the 5-character TLE field."""
    with open(DATA_DIR / "hypso1.tle", 'r') as file:
        lines = file.read().splitlines()

    line1 = lines[1][:2] + "A1053" + lines[1][7:]
    line2 = lines[2][:2] + "A1053" + lines[2][7:]

    tle_path = _write_modified_hypso_tle(tmp_path, line1=line1, line2=line2)

    with pytest.raises(ValueError, match="Alpha-5-like"):
        tleHandling.satTle2elem(tle_path)


def test_read_tle_rejects_non_numeric_id_field(tmp_path):
    """satTle2elem() rejects non-numeric, non-Alpha-5 catalog fields."""
    with open(DATA_DIR / "hypso1.tle", 'r') as file:
        lines = file.read().splitlines()

    line1 = lines[1][:2] + "12-53" + lines[1][7:]
    line2 = lines[2][:2] + "12-53" + lines[2][7:]

    tle_path = _write_modified_hypso_tle(tmp_path, line1=line1, line2=line2)

    with pytest.raises(ValueError, match="non-numeric"):
        tleHandling.satTle2elem(tle_path)


def test_read_tle_rejects_catalog_overflow(tmp_path):
    """satTle2elem() rejects likely 6-digit overflow into fixed-width TLE columns."""
    with open(DATA_DIR / "hypso1.tle", 'r') as file:
        lines = file.read().splitlines()

    line1_chars = list(lines[1])
    line2_chars = list(lines[2])
    line1_chars[7] = '1'
    line2_chars[7] = '1'
    line1 = "".join(line1_chars)
    line2 = "".join(line2_chars)

    tle_path = _write_modified_hypso_tle(tmp_path, line1=line1, line2=line2)

    with pytest.raises(ValueError, match="overflow"):
        tleHandling.satTle2elem(tle_path)


def test_write_tle_6digit_norad_raises():
    """generateTle() must raise ValueError for catalog numbers >= 100000 (6-digit NORAD IDs assigned after ~July 2026 cannot be represented in TLE format)."""
    tleData = tleHandling.TleData(oe=oeHypso1, tleEpoch=hypso1tleEpoch)
    tleData.noradID = 100000
    with pytest.raises(ValueError, match="100000"):
        tleHandling.generateTle(tleData)

if __name__ == "__main__":
    # Reading TLE
    test_read_tle(DATA_DIR / "hypso1.tle", EXPECTED_OE_HYPSO)
    test_read_tle(DATA_DIR / "oneWeb25.tle", EXPECTED_OE_ONE_WEB)
    test_read_tle(DATA_DIR / "spacestations.2le", EXPECTED_OE_2LE)

    # Writing TLE
    test_write_tle('HYPSO 1', oeHypso1, hypso1noradId, hypso1launch, launch_Noyear=2, PoL='BX', tleEpoch=hypso1tleEpoch, nDot=hypso1nDot, bStar=hypso1bStar, expectedTlePath=DATA_DIR / "hypso1.tle")

    # Take TLE, generate orbital elements, then generate TLE again and compare to original
    test_read_write_tle(DATA_DIR / "hypso1.tle")
