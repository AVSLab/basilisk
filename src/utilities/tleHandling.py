from matplotlib.dates import SEC_PER_DAY
from typing import Tuple, List, Dict
import numpy as np
import datetime as dt
import re

from Basilisk.utilities import orbitalMotion as om

SPUTNIK_LAUNCHDATE = dt.datetime(1957, 10, 4, 0, 0, 0)

def _calcTleChecksum(stringArray: str) -> int:
    """
    Calculate TLE checksum per line.

    :param stringArray: array of strings to calculate checksum for a TLE line
    :return: checksum value (0-9) [-]
    """
    total = 0
    for elem in stringArray:
        for char in elem:
            if char.isdigit():
                total += int(char)
            elif char == "-":
                total += 1
    return total % 10

def _firstDer2BallistCoef(balistic_coeff_str: str) -> float:
    """
    Convert the first derivative of mean motion to physical units.

    :param balistic_coeff_str: string representing the first derivative of mean motion
    :return: first derivative of mean motion in [rev/day^2]
    """
    sign = balistic_coeff_str[0]
    if sign == "-":
        sign = -1
    else:
        sign = 1
    return sign * float(balistic_coeff_str[1:])

def _tleStr2PhysVal(tleStr: str) -> float:
    """
    Convert either the second derivative of mean motion or the B* drag term to physical units.

    :param: string representing either: second derivative of mean motion OR B* drag term
    :return: second derivative of mean motion in [rev/day^3] OR B* drag in [1/Earth radii]
    """
    sign = tleStr[0]
    exp_sign = tleStr[6]
    if sign == "-":
        sign = -1
    else:
        sign = 1
    if exp_sign == "-":
        exp_sign = -1
    else:
        exp_sign = 1
    mantissa = float(("0." + tleStr[1:6]).strip())
    exponent = float(tleStr[7:8].strip())
    return sign * mantissa * 10 ** (exp_sign * exponent)

def _parseTLE2elem(tle: str) -> Tuple[om.ClassicElements, Dict]:
    """
    Convert a single TLE to classical orbital elements.

    :param tle: a string representing either a TLE(3LE) or a 2LE
    :return: elements: classical orbital elements
             metadata: dictionary containing TLE metadata
    """
    # Check if TLE has two or three lines
    if len(tle) == 3:
        line0 = tle[0]
        line1 = tle[1]
        line2 = tle[2]
    elif len(tle) == 2:
        # If there are two lines (no title line), set line0 to empty
        line0 = ''
        line1 = tle[0]
        line2 = tle[1]
    elif len(tle) > 3:
        raise ValueError("ERROR: _parseTLE2elem() received tle with {} lines. Most likely a concatenation of multiple TLEs. \n Use constTLE2Elem() for satellite constellations.".format(len(tle)))
    else:
        raise ValueError("ERROR: _parseTLE2elem() received tle with {} lines. The TLE should have 2 or 3 lines.".format(len(tle)))

    # Extract orbital elements from the first TLE-line
    line1_str = line1[0:1]
    if line1_str != '1':
        raise ValueError("ERROR: _parseTLE2elem() received line1 starting with '{}'. The first character of line1 should be '1'.".format(line1_str))
    satID_1_str = line1[2:7] # Satellite catalog number
    class_str = line1[7:8] # Classification (U=Unclassified, C=Classified, S=Secret)
    launchYear_str = line1[9:11] # Launch year
    launchNo_str = line1[11:14] # Launch number of the year
    pol_str = line1[14:17] # Piece of launch
    epochYear = int(line1[18:20]) # Last two digits of the year
    epochDay_str = line1[20:32] # Day of the year incl fractional part [DDD.DDDDDDDD]
    nDot_str = line1[33:43] # String containint the first derivative of the mean motion
    nDotDot_str = line1[44:52] # String containing the second derivative of the mean motion (decimal point assumed)
    bStar_str = line1[53:61]
    ephemType_str = line1[62:63]
    elemSetNo_str = line1[64:68]
    checkSumLine1 = int(line1[68:69])
    calcChecksum1 = _calcTleChecksum([line1_str, satID_1_str, class_str, launchYear_str,
                                           launchNo_str, pol_str,
                                           str(epochYear), epochDay_str, nDot_str,
                                           nDotDot_str, bStar_str, ephemType_str,
                                           elemSetNo_str])
    if checkSumLine1 != calcChecksum1:
        raise ValueError("ERROR: _parseTLE2elem() checksum mismatch for line1. The calculated checksum is {} while the provided checksum is {}.".format(calcChecksum1, checkSumLine1))

    # Extract orbital elements from the second TLE-line
    line2_str = line2[0:1]
    if line2_str != '2':
        raise ValueError("ERROR: _parseTLE2elem() received line2 starting with '{}'. The first character of line2 should be '2'.".format(line2_str))
    satID_2_str = line2[2:7].strip()
    if satID_2_str != satID_1_str:
        raise ValueError("ERROR: _parseTLE2elem() received mismatched satellite ID numbers in line1 ('{}') and line2 ('{}').".format(satID_1_str, satID_2_str))
    inclDeg_str = line2[8:16] # Inclination in degrees
    OmegaDeg_str = line2[17:25] # Right Ascension of Ascending Node in degrees
    e_str = line2[26:33] # Eccentricity (decimal point assumed)
    omega_str = line2[34:42] # Argument of Perigee in degrees
    meanAnomaly_str = line2[43:51] # Mean Anomaly in degrees
    meanMotion_str = line2[52:63] # Mean Motion in revolutions per day
    revNo_str = line2[63:68] # Revolution number at epoch
    checkSumLine2 = int(line2[68:69])
    calcChecksum2 = _calcTleChecksum([line2_str, satID_2_str, inclDeg_str, OmegaDeg_str, e_str, omega_str, meanAnomaly_str, meanMotion_str, revNo_str])
    if checkSumLine2 != calcChecksum2:
        raise ValueError("ERROR: _parseTLE2elem() checksum mismatch for line2. The calculated checksum is {} while the provided checksum is {}.".format(calcChecksum2, checkSumLine2))

    # Parsing numerical values getting
    satName = line0.strip()
    Norad_ID = int(satID_1_str)
    classification = {'U': 'Unclassified', 'C': 'Classified', 'S': 'Secret'}.get(class_str, 'Unknown')
    if epochYear < 57: # First satellite launched & cataloged in 1957 (Sputnik 1) | This has to be updated in 2057
        epochYear += 2000
    else:
        epochYear += 1900
    tle_datetime = dt.datetime(epochYear, 1, 1) + dt.timedelta(days=float(epochDay_str) - 1) # subtract one day because Jan 1 is day 1 (not day 0)
    tle_datetime = tle_datetime.replace(tzinfo=dt.timezone.utc)

    # Converting TLE strings to physical values
    n_dot = _firstDer2BallistCoef(nDot_str) # dn/dt in [rev/day^2]
    n_dotdot = _tleStr2PhysVal(nDotDot_str) # d^2n/dt^2 in [rev/day^3]
    bStar = _tleStr2PhysVal(bStar_str)
    propagator = {'1': 'SGP4', '2': 'SDP4', '3': 'SGP8', '4': 'SDP8', '0': 'Unknown'}.get(ephemType_str, 'Unknown')
    elemSetNo = int(elemSetNo_str)

    # Converting TLE information to classical orbital elements
    elements = om.ClassicElements()
    elements.i = np.deg2rad(float(inclDeg_str.strip()))
    elements.Omega = np.deg2rad(float(OmegaDeg_str.strip()))
    elements.e = float(("0." + e_str).strip())
    elements.omega = np.deg2rad(float(omega_str.strip()))
    revDay = float(meanMotion_str.strip())
    elements.a = (om.MU_EARTH / ((revDay * 2.0 * np.pi) / SEC_PER_DAY) ** 2) ** (1.0 / 3.0) * 1e3
    M = np.deg2rad(float(meanAnomaly_str.strip()))
    elements.f = om.E2f(om.M2E(M, elements.e), elements.e)

    revAtEpoch = float(revNo_str.strip())
    metadata = {
        'satName': satName,
        'Norad_ID': Norad_ID,
        'classification': classification,
        'tle_epoch': tle_datetime,
        'revAtEpoch': revAtEpoch,
        'propagator': propagator,
        'elemSetNo': elemSetNo,
        'n_dot': n_dot,
        'n_dotdot': n_dotdot,
        'bStar': bStar,
        'launchYear': dt.datetime(int(launchYear_str), 1, 1),
        'launchNo': int(launchNo_str),
        'PoL': pol_str,
    }

    return elements, metadata

def satTLE2Elem(tle_path: str) -> Tuple[List[om.ClassicElements], List[Dict]]:
    """
    Convert the TLEs of a constellation to classical orbital elements for each satellite.

    :param tle_path: path to a TLE with one or multiple satellites in either 2LE or 3LE format
    :return: elementsList: list of classical orbital elements for each satellite
             metadataList: list of dictionaries containing TLE metadata for each satellite
    """
    def resetTLEStr():
        """
        Reset the TLE strings and flags for reading a new satellite.
        """
        nonlocal satTLE, satTleReady, readingOrderIndex
        readingOrderIndex = 0
        if type == 2:
            satTLE = ['', '']
            satTleReady = [False, False] # flags to check if all three lines are present
        elif type == 3:
            readingOrderIndex = 0
            satTLE = ['', '', '']
            satTleReady = [False, False, False] # flags to check if all three lines are present

    tle_lines = open(tle_path).readlines()
    noLines = len(tle_lines)
    if noLines < 2:
        raise ValueError("ERROR: constTLE2Elem() received a TLE with {} lines. A valid TLE must have at least two lines.".format(noLines))
    noOfLinesStartingWith1 = sum(1 for line in tle_lines if line.startswith('1'))
    noOfLinesStartingWith2 = sum(1 for line in tle_lines if line.startswith('2'))
    noOfLinesStartingWithOther = len(tle_lines) - noOfLinesStartingWith1 - noOfLinesStartingWith2

    # Initialize variables
    orbitElementsList = []
    metadataList = []

    readingOrderIndex = 0

    # Basic checks on the TLE input format (Robustness)
    if noOfLinesStartingWith1 != noOfLinesStartingWith2:
        raise ValueError("ERROR: constTLE2Elem() received a TLE with {} lines starting with '1' and {} lines starting with '2'. The number of lines starting with '1' and '2' must be equal.".format(noOfLinesStartingWith1, noOfLinesStartingWith2))
    if noOfLinesStartingWithOther >= noOfLinesStartingWith1:
        #TLE contains at least as many titles as lines starting with '1' or '2'=> 3LE datatype
        type = 3 # 3LE
    elif noOfLinesStartingWithOther == 0 and noLines >= 2:
        # TLE has no title lines at least two or more lines => 2LE
        type = 2 # 2LE
    elif noOfLinesStartingWithOther == 1 and noLines > 2:
        # TLE has one title line and more than two lines => 2LE (first line likely title for 2LE-file)
        type = 2 # 2LE
        if tle_lines[0].startswith('1') or tle_lines[0].startswith('2'):
            raise ValueError("ERROR: constTLE2Elem() received a TLE with one title line but the first line starts with '1' or '2'. --- TLE not valid ---.")

    # Define the reading order based on the TLE type
    if type == 2:
        readingOrder = ['line1', 'line2']
        satTLE = ['', '']
        satTleReady = [False, False] # flags to check if all three lines are present
    elif type == 3:
        readingOrder = ['title', 'line1', 'line2']
        satTLE = ['', '', '']
        satTleReady = [False, False, False] # flags to check if all three lines are present
    else:
        # This point should be unreachable, kept here for Robustness
        raise ValueError("ERROR: constTLE2Elem() received an invalid TLE type '{}'. The type must be '2LE' or '3LE'.".format(type))

    resetTLEStr()
    # Read through all lines and extract TLEs
    for lineNo, line in enumerate(tle_lines):
        expected = readingOrder[readingOrderIndex] if readingOrderIndex < len(readingOrder) else None
        if line.startswith('1') and expected == 'line1':
            if satTleReady[1]:
                print(f"WARNING: constTLE2Elem() found a new TLE line1 before completing the previous TLE (line number {lineNo}). The previous TLE will be discarded.")
                # Reset for next TLE
                resetTLEStr()
            # First TLE-line
            satTLE[type-2] = line
            satTleReady[type-2] = True
            readingOrderIndex += 1
        elif line.startswith('2') and expected == 'line2':
            if satTleReady[type-1]:
                print(f"WARNING: constTLE2Elem() found a new TLE line2 before completing the previous TLE (line number {lineNo}). The previous TLE will be discarded.")
                # Reset for next TLE
                resetTLEStr()
            # Second TLE-line
            satTLE[type-1] = line
            satTleReady[type-1] = True
        elif expected == 'title' and type == 3:
            if satTleReady[0]:
                print(f"WARNING: constTLE2Elem() found a new TLE title line before completing the previous TLE (line number {lineNo}). The previous TLE will be discarded.")
                # Reset for next TLE
                resetTLEStr()
            # Header (satellite ID / satellite name)
            satTLE[0] = line
            satTleReady[0] = True
            readingOrderIndex += 1
        else:
            # Line does not match expected format
            print(f"WARNING: constTLE2Elem() found an unexpected line: {line.strip()}, line number {lineNo}. This line will be ignored. The previous TLE will be discarded.")
            continue

        if all(satTleReady):
            elements, metadata = _parseTLE2elem(satTLE)
            orbitElementsList.append(elements)
            metadataList.append(metadata)
            # Reset for next TLE
            resetTLEStr()
    return orbitElementsList, metadataList

def _str2tle_format(value: float) -> str:
    """
    Convert one of the following numbers:
        - "Second derivative of mean motion [rev/day^3]"
        - "B*, the drag term [1/Earth radii]"
    from its numeric value to the TLE format:

    :param value: numeric value to convert
    :return: string in TLE format
    """
    # Sign of the value
    value = value * 10  # Multiply value by 10 to get the exponent right
    if value >= 0:
        sign = " "
    else:
        sign = "-"
    value = abs(value)  # Absolute value
    sci_notation = f"{value:.4e}"
    # Split the scientific notation into mantissa and exponent parts
    parts = sci_notation.split("e")
    matissa = parts[0].replace(".", "")[:5]
    exponentStr = parts[1].lstrip("+")
    exponent = int(exponentStr)
    if exponent > 0:
        exp_sign = "+"
    else:
        exp_sign = "-"
    # Format the string according to TLE format
    exponent_str = f"{exp_sign}{abs(exponent):01d}"
    tle_sub_str = f"{sign}{matissa}{exponent_str}"
    return tle_sub_str

def generateTleDataString(orbitElements: om.ClassicElements,
                             satelliteName: str = "BSK-Sat-00",
                             launchYear_dt: dt.datetime = None,
                             launch_Noyear: int = None,
                             NORAD_ID: str = "00000",
                             classification: str = "U",
                             PoL: str = None,
                             tle_epoch: dt.datetime = None,
                             element_set_no: int = 999,
                             n_dot: float = None,
                             n_dotdot: float = None,
                             BStar: float = None) -> str:
        """
        Generate TLE based on satellite orbit_n data for one satellite.

        :param orbitElements: Classical orbital elements of the satellite                    [om.ClassicElements object]
        :param satelliteName: Name of the satellite (optional, default: "BSK-Sat-00")        [string, <= 24 char]
        :param launchYear_dt: Launch date of the satellite (optional, default: current year) [datetime object]
        :param launch_Noyear: Launch number of the year (optional, default: 1)               [int]
        :param NORAD_ID: NORAD ID (5 digits) of the satellite (optional, default: "00000")   [str, 5 char]
        :param classification: Satellite classification (U: Unclassified, C: Classified, S: Secret | default: "U")
                                                                                             [str, 1 char]
        :param PoL: Piece of the launch (optional, default: "A  ")                           [str, 3 char]
        :param tle_epoch: Epoch of the TLE as datetime object (optional default: current UTC time)
                                                                                             [datetime object]
        :param element_set_no: Element set number (optional, default: 999)                   [int]
        :param n_dot: First derivative of mean motion (dn/dt)                                [float, rev/day^2]
        :param n_dotdot: Second derivative of mean motion (d^2n/dt^2)                        [float, rev/day^3]
        :param BStar: B* drag term (optional, default: 0.0)                                  [float, 1/Earth radii]
        :return: TLE string                                                                  [str, 3 lines]
        """
        # Make sure NORAD_ID is an integer and no longer than 5 characters (truncate)
        Norad_str = f"{NORAD_ID:05d}"
        if len(str(NORAD_ID)) > 5:
            print(f"TLE Writing: NORAD_ID truncated to 5 digits: {Norad_str}")

        if launchYear_dt is None:
            launchYear_dt = dt.datetime.now(dt.timezone.utc)
            print(f"TLE Writing: Launch date not provided, using current UTC time: {launchYear_dt}")

        if launch_Noyear is None:
            launch_Noyear = 1
            print(f"TLE Writing: launch_Noyear not provided, defaulting to: {launch_Noyear}")

        if PoL is None or len(PoL) > 3:
            PoL = "A"
            print(f"TLE Writing: Piece of the Launch (PoL) was not provided or is invalid, defaulting to 'A  '.")

        if tle_epoch is None:
            tle_epoch = dt.datetime.now(dt.timezone.utc)
            print(f"TLE Writing: TLE epoch not provided, using current UTC time: {tle_epoch.isoformat()}")
        epoch_str = f"{tle_epoch:%y%j}" + f"{(tle_epoch.hour * 3600 + tle_epoch.minute * 60 + tle_epoch.second + tle_epoch.microsecond / 1e6) / SEC_PER_DAY:.8f}"[1:]

        if n_dot is None:
            n_dot = 0.0
            print(f"TLE Writing: First derivative of mean motion / balistic coefficient (n_dot) not provided, defaulting to: {n_dot}")

        if n_dotdot is None:
            n_dotdot = 0.0
            print(f"TLE Writing: Second derivative of mean motion (n_dotdot) not provided, defaulting to: {n_dotdot}")

        if BStar is None:
            BStar = 0.0
            print(f"TLE Writing: BStar drag term not provided, defaulting to: {BStar}")

        classification = classification[0].upper() if classification[0].upper() in ['U', 'C', 'S'] else 'U'

        # Line 0
        line0_str = satelliteName   # Satellite name (User defined satellite name, Default: "BSK00")

        # Line 1
        line_no_1_str = "1"  # Line number "1"
        satIDStr = Norad_str  # NORAD ID or default
        classStr = classification  # The satellite is unclassified [Can be U: Unclassified, C: Classified, S: Secret]
        IntD_LYear = f"{launchYear_dt.year % 100:02d}"  # International Designator (last two digits of launch year) COSPAR ID
        IntD_LNo = f"{int(launch_Noyear):03d}"  # International Designator (launch no. of the year) COSPAR ID
        IntD_PoL = f"{PoL:<3}"[:3]  # International Designator (piece of the launch) COSPAR ID [default 'A']
        epochStr = epoch_str  # Epoch in TLE format [YYDDD.DDDDDDDD]
        n_dot_str = re.sub(r"-0", "-", f"{n_dot:10.8f}")  # Balistic coeffizient / First derivative of the mean motion in TLE format
        n_dot_str = n_dot_str.replace("0.", " .") if n_dot > 0 else n_dot_str.replace("0.", "-.")  # Remove the decimal point
        n_dotdot_str = _str2tle_format(
            n_dotdot
        )
        b_star_str = _str2tle_format(
            BStar
        )
        ephemeris_str = "0" # Ephemeris type is 0 for BSK
        elem_st_no_str = f"{element_set_no:03d}"  # Element set number (incremented for each new TLE for the same object)
        checksum1 = _calcTleChecksum(
            [
                line_no_1_str,
                satIDStr,
                IntD_LYear,
                IntD_LNo,
                epochStr,
                n_dot_str,
                n_dotdot_str,
                b_star_str,
                ephemeris_str,
                elem_st_no_str,
            ]
        )

        line1_str = f"{line_no_1_str} {satIDStr}{classStr} {IntD_LYear}{IntD_LNo}{IntD_PoL} {epochStr} {n_dot_str} {n_dotdot_str} {b_star_str} {ephemeris_str}  {elem_st_no_str}{checksum1}"

        # Line 2
        line_no_2_str = "2"  # Line number "2"
        satcat_str = satIDStr  # Satellite Catalog Number (5 digits) [same as satID]
        i_str = f"{np.rad2deg(orbitElements.i):8.4f}"  # Inclination in degrees
        raan_str = f"{np.rad2deg(orbitElements.Omega):8.4f}"  # Right Ascension of Ascending Node in degrees
        eccen_str = f"{int(orbitElements.e * 1e7):07d}"  # Eccentricity  * 10^7 as decimal value / decimal point assumed
        per_str = f"{np.rad2deg(orbitElements.omega):8.4f}"  # Argument of perigee in degrees
        # Calculate M from orbital elements
        M = om.E2M(om.f2E(orbitElements.f, orbitElements.e), orbitElements.e)
        mean_anom_str = f"{np.rad2deg(M):8.4f}"  # Mean anomaly
        # Calculate mean motion in [rev/day]
        n = np.sqrt(om.MU_EARTH / (orbitElements.a / 1000.0) ** 3) * SEC_PER_DAY / (2.0 * np.pi)
        mean_Mot_str = f"{n:11.8f}"  # Revolutions per day
        rev_str = f"{0:05d}"  # Revolution number at epoch [hardcoded to '00000']
        checksum2 = _calcTleChecksum(
            [
                line_no_2_str,
                satcat_str,
                i_str,
                raan_str,
                eccen_str,
                per_str,
                mean_anom_str,
                mean_Mot_str,
                rev_str,
            ]
        )  # Checksum for line 2 (modulo 10)

        line2_str = f"{line_no_2_str} {satcat_str} {i_str} {raan_str} {eccen_str} {per_str} {mean_anom_str} {mean_Mot_str}{rev_str}{checksum2}"

        # Combine the lines to the full TLE
        tle_str = f"{line0_str}\n{line1_str}\n{line2_str}"
        return tle_str
