import pytest
import numpy as np
import Basilisk.utilities.tleHandling as tleHandling
import Basilisk.utilities.orbitalMotion as om
from datetime import datetime, timedelta, timezone

A_TOL = 1e-14 #[-]

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
oe_hypso1 = om.ClassicElements()
oe_hypso1.i = np.deg2rad(97.3197)
oe_hypso1.e = 0.0004257 # [-]
oe_hypso1.a = (om.RP_EARTH + 445.195971181338)*1e3 # [m]
oe_hypso1.Omega = np.deg2rad(349.0390) # [rad]
oe_hypso1.omega = np.deg2rad(136.0807) # [rad]
oe_hypso1.f = np.deg2rad(224.04427854247686) # [rad]
hypso_1_Norad_Id = 51053 # [-]
hypso_1_launchDate = datetime(2022, 1, 13, 0, 0, 0) # [UTC]
hypso_1_tleEpoch = datetime(2025, 1, 1, tzinfo=timezone.utc) + timedelta(days=279.47924866 - 1) # [UTC]
hypso1_nDot = 0.00028852 # [rev/day^2]
hypso1_bStar = 0.00056053 # [1/Earth radii]

# Expected orbital elements for Hypso1
EXPECTED_OE_HYPSO = {
    'a': [oe_hypso1.a],
    'e': [oe_hypso1.e],
    'i': [oe_hypso1.i],
    'f': [oe_hypso1.f],
    'Omega': [oe_hypso1.Omega],
    'omega': [oe_hypso1.omega]
}

###################################################################
def _equal_check(v1, v2, fieldName):
    """Compare two values within tolerance"""
    if abs(v1 - v2) < A_TOL:
        return 0
    print(f'{fieldName} failed: expected {v2}, got {v1}')
    return 1

@pytest.mark.parametrize("tle_path, expected_dict", [
    ("src/tests/data/hypso1.tle", EXPECTED_OE_HYPSO),
    ("src/tests/data/oneWeb25.tle", EXPECTED_OE_ONE_WEB),
    ("src/tests/data/spacestations.2le", EXPECTED_OE_2LE),
])
def test_read_tle(tlePath, expectedDict):
    e_count_tle = 0

    # Read TLE file from data folder
    orbElem, _ = tleHandling.sat_tle2elem(tlePath)

    # Check each orbital element
    for idx, _ in enumerate(orbElem):
        for key in expectedDict:
            actual_value = getattr(orbElem[idx], key)
            e_count_tle += _equal_check(actual_value, expectedDict[key][idx], f'{tlePath}_{key}')

    assert e_count_tle < 1, f"{e_count_tle} functions failed in tleHandling.py script, satTLE2Elem() method"

@pytest.mark.parametrize("satName, orbitalElements, noradID, launchDate, launch_Noyear, PoL, TLE_epoch, n_dot, bStar, expectedTlePath", [
    ('HYPSO 1', oe_hypso1, hypso_1_Norad_Id, hypso_1_launchDate, 2, 'BX', hypso_1_tleEpoch, hypso1_nDot, hypso1_bStar, "src/tests/data/hypso1.tle"),
])
def test_write_tle(satName, orbitalElements, noradID, launchDate, launch_Noyear, PoL, TLE_epoch, n_dot, bStar, expectedTlePath):
    e_count_tle = 0
    generatedTle = tleHandling.generateTleDataString(orbitalElements,
                                                      satelliteName=satName,
                                                      NORAD_ID=noradID,
                                                      launchYear_dt=launchDate,
                                                      launch_Noyear=launch_Noyear,
                                                      PoL=PoL,
                                                      tle_epoch=TLE_epoch,
                                                      n_dot=n_dot,
                                                      n_dotdot=0.0,
                                                      BStar=bStar)

    # Read the actual TLE file
    with open(expectedTlePath, 'r') as file:
        expectedTLE = file.read()
    # String compaire (line 0)
    e_count_tle += int((generatedTle.splitlines()[0] != expectedTLE.splitlines()[0]))
    # String compaire (line 1)
    e_count_tle += int((generatedTle.splitlines()[1] != expectedTLE.splitlines()[1]))
    # String compaire (line 2)
    e_count_tle += int((generatedTle.splitlines()[2][0:63] != expectedTLE.splitlines()[2][0:63])) # Ignore revolutions count and checksum
    assert e_count_tle < 1, f"{e_count_tle} functions failed in tleHandling.py script, generateTleDataString() method"

@pytest.mark.parametrize("tle_path", [
    "src/tests/data/hypso1.tle",
])
def test_read_write_tle(tlePath):
    e_count_tle = 0
    # Read TLE file
    orbElem, metaData = tleHandling.satTLE2Elem(tlePath)

    generatedTle = tleHandling.generateTleDataString(orbElem[0],
                                                      satelliteName=metaData[0]['satName'],
                                                      launchYear_dt=metaData[0]['launchYear'],
                                                      launch_Noyear=metaData[0]['launchNo'],
                                                      NORAD_ID=metaData[0]['Norad_ID'],
                                                      classification={'Unclassified': 'U', 'Classified': 'C', 'Secret': 'S'}.get(metaData[0]['classification'], 'U'),
                                                      PoL=metaData[0]['PoL'],
                                                      tle_epoch=metaData[0]['tle_epoch'],
                                                      element_set_no=metaData[0]['elemSetNo'],
                                                      n_dot=metaData[0]['n_dot'],
                                                      n_dotdot=metaData[0]['n_dotdot'],
                                                      BStar=metaData[0]['bStar'])

    # Read the actual TLE file
    with open(tlePath, 'r') as file:
        tleToBeTested = file.read()

    # String compaire (line 0)
    e_count_tle += int((generatedTle.splitlines()[0] != tleToBeTested.splitlines()[0]))
    # String compaire (line 1)
    e_count_tle += int((generatedTle.splitlines()[1] != tleToBeTested.splitlines()[1]))
    # String compaire (line 2)
    e_count_tle += int((generatedTle.splitlines()[2][0:63] != tleToBeTested.splitlines()[2][0:63])) # Ignore revolutions count and checksum
    assert e_count_tle < 1, f"{e_count_tle} functions failed in tleHandling.py script, generateTleDataString() method"

if __name__ == "__main__":
    # Reading TLE
    test_read_tle("src/tests/data/hypso1.tle", EXPECTED_OE_HYPSO)
    test_read_tle("src/tests/data/oneWeb25.tle", EXPECTED_OE_ONE_WEB)
    test_read_tle("src/tests/data/spacestations.2le", EXPECTED_OE_2LE)

    # Writing TLE
    test_write_tle('HYPSO 1', oe_hypso1, hypso_1_Norad_Id, hypso_1_launchDate, launch_Noyear=2, PoL='BX', TLE_epoch=hypso_1_tleEpoch, n_dot=hypso1_nDot, bStar=hypso1_bStar, expectedTlePath="src/tests/data/hypso1.tle")

    # Take TLE, generate orbital elements, then generate TLE again and compare to original
    test_read_write_tle("src/tests/data/hypso1.tle")
