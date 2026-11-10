#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Orbit Mean-Elements Message (OMM) handling
#   Author: robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date: September 16, 2026
#

"""
Read CCSDS Orbit Mean-Elements Message (OMM) general perturbations data and convert it to
osculating classical orbital elements in the Basilisk inertial (J2000/ICRF) frame.

OMM supports NORAD catalog numbers of up to nine digits, removing the classic numeric
TLE field's five-digit restriction. The U.S. Space Force (USSF) assigns catalog numbers;
CelesTrak distributes the data. This reader supports IDs through ``999999999`` and
preserves them in ``OmmData.noradID``, independently of SGP4's internal identifier. See
`CelesTrak's GP format documentation
<https://celestrak.org/NORAD/documentation/gp-data-formats.php>`__.

All four CelesTrak encodings are accepted and auto-detected: XML, JSON, CSV and KVN.  The
mean elements are handed to SGP4 exactly as :ref:`tleHandling` does for a TLE, so both paths
produce osculating elements through the same propagation and frame conversion. Only the
Earth-centered, TEME, UTC, SGP4 profile is supported.
"""

import csv
import datetime as dt
import io
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
from sgp4 import omm as sgp4omm
from sgp4.api import Satrec

from Basilisk.utilities import orbitalMotion as om
# The TEME -> J2000 conversion is shared with the TLE reader so both formats land in the
# same frame through identical precession, nutation and frame-bias terms.
from Basilisk.utilities.tleHandling import _teme2j2000

#: Fields required in normalized records before conversion.
_REQUIRED_OMM_FIELDS = (
    "EPOCH",
    "MEAN_MOTION",
    "ECCENTRICITY",
    "INCLINATION",
    "RA_OF_ASC_NODE",
    "ARG_OF_PERICENTER",
    "MEAN_ANOMALY",
    "NORAD_CAT_ID",
    "CLASSIFICATION_TYPE",
    "EPHEMERIS_TYPE",
    "ELEMENT_SET_NO",
    "REV_AT_EPOCH",
    "BSTAR",
    "MEAN_MOTION_DOT",
    "MEAN_MOTION_DDOT",
)

#: Defaults for fields that CelesTrak sometimes omits but SGP4 still requires.
_OPTIONAL_OMM_DEFAULTS = {
    "CLASSIFICATION_TYPE": "U",
    "EPHEMERIS_TYPE": "0",
    "ELEMENT_SET_NO": "999",
    "REV_AT_EPOCH": "0",
    "BSTAR": "0.0",
    "MEAN_MOTION_DOT": "0.0",
    "MEAN_MOTION_DDOT": "0.0",
}

#: Supported physical metadata, omitted by CelesTrak's JSON and CSV encodings.
_SUPPORTED_OMM_METADATA = {
    "CENTER_NAME": "EARTH",
    "REF_FRAME": "TEME",
    "TIME_SYSTEM": "UTC",
    "MEAN_ELEMENT_THEORY": "SGP4",
}

#: Consumed XML fields must appear in their CCSDS owning blocks.
_OMM_XML_BLOCK_FIELDS = {
    "metadata": frozenset(("OBJECT_NAME", "OBJECT_ID", *_SUPPORTED_OMM_METADATA)),
    "meanElements": frozenset((
        "EPOCH", "MEAN_MOTION", "ECCENTRICITY", "INCLINATION",
        "RA_OF_ASC_NODE", "ARG_OF_PERICENTER", "MEAN_ANOMALY",
    )),
    "tleParameters": frozenset((
        "EPHEMERIS_TYPE", "CLASSIFICATION_TYPE", "NORAD_CAT_ID", "ELEMENT_SET_NO",
        "REV_AT_EPOCH", "BSTAR", "MEAN_MOTION_DOT", "MEAN_MOTION_DDOT",
    )),
}

#: Recognized containers to validate recursively; unrelated extension subtrees are ignored.
_OMM_XML_BLOCK_CHILDREN = {
    "header": frozenset(),
    "body": frozenset(("segment",)),
    "data": frozenset((
        "meanElements", "tleParameters", "spacecraftParameters", "covarianceMatrix",
        "userDefinedParameters",
    )),
    **_OMM_XML_BLOCK_FIELDS,
    "spacecraftParameters": frozenset(),
    "covarianceMatrix": frozenset(),
    "userDefinedParameters": frozenset(),
}

#: Segments are checked separately so a malformed segment does not reject its siblings.
_OMM_XML_BLOCKS = frozenset(("ndm", "omm", "segment", *_OMM_XML_BLOCK_CHILDREN))

#: Fields consumed by this reader; comments and unknown extension fields may repeat.
_OMM_FIELDS = frozenset().union(*_OMM_XML_BLOCK_FIELDS.values())

#: Preserve duplicate fields or XML blocks until validation can reject the record.
_DUPLICATE_OMM_FIELD = object()

#: Human-readable names for the supported classification codes.
_CLASSIFICATION_NAMES = {"U": "Unclassified", "C": "Classified", "S": "Secret"}

#: CCSDS units for numeric fields consumed by this reader; empty tuples forbid units.
_OMM_FIELD_UNITS = {
    "MEAN_MOTION": ("rev/day",),
    "ECCENTRICITY": (),
    "INCLINATION": ("deg",),
    "RA_OF_ASC_NODE": ("deg",),
    "ARG_OF_PERICENTER": ("deg",),
    "MEAN_ANOMALY": ("deg",),
    "NORAD_CAT_ID": (),
    "EPHEMERIS_TYPE": (),
    "ELEMENT_SET_NO": (),
    "REV_AT_EPOCH": (),
    # CCSDS lists inverse Earth radii as both a descriptive unit and the 1/ER symbol.
    "BSTAR": ("1/ER", "1/[Earth radii]"),
    "MEAN_MOTION_DOT": ("rev/day**2",),
    "MEAN_MOTION_DDOT": ("rev/day**3",),
}


@dataclass(frozen=True)
class _OmmFieldWithUnits:
    """Preserve a numeric field's unit declaration until per-record validation."""

    value: str
    units: str


@dataclass(frozen=True)
class _OmmInvalidRecord:
    """Retain a parser diagnostic at the original record position."""

    reason: str


# ---------------------------------------------------------------------------------------------------------- #
#                                          OMM Data Class                                                     #
# ---------------------------------------------------------------------------------------------------------- #
@dataclass
class OmmData:
    """
    Hold the orbital elements and metadata of a single OMM record.

    Mirrors :py:class:`Basilisk.utilities.tleHandling.TleData`, with the OMM-only metadata
    added and the TLE-only line-format fields dropped.

    ``nDot`` and ``nDotDot`` retain the supplied ``MEAN_MOTION_DOT`` and
    ``MEAN_MOTION_DDOT`` values without rescaling, in ``rev/day**2`` and ``rev/day**3``.
    CelesTrak GP data uses the TLE coefficient convention: one-half of the first
    physical derivative and one-sixth of the second physical derivative.
    Other producers' conventions must be interpreted using their documentation.
    These coefficients are passed unchanged to the SGP4 initializer, which converts
    their units; SGP4 itself does not use them for propagation. See
    `CelesTrak's TLE field explanation <https://celestrak.org/columns/v04n03/>`__.
    """

    # Required (orbit elements and the time at which they are valid)
    oe: om.ClassicElements
    ommEpoch: dt.datetime

    # Optional metadata
    satName: str = field(default="BSK-Sat-00")  # OBJECT_NAME
    noradID: str = field(default="00000")  # NORAD_CAT_ID; unlike a TLE this may be >= 100000
    objectID: str = field(default="")  # international designator; empty when unavailable
    classification: str = field(default="Unknown")  # "Unclassified", "Classified", "Secret"
    revAtEpoch: int = field(default=0)  # revolution number at epoch [-]
    propagator: str = field(default="0")  # EPHEMERIS_TYPE: 0 (CelesTrak default) or 2 (SGP4)
    elemSetNo: int = field(default=999)  # element set number [-]
    meanMotion: float = field(default=0.0)  # mean motion at epoch [rev/day]
    nDot: float = field(default=0.0)  # supplied MEAN_MOTION_DOT [rev/day^2]; see class convention
    nDotDot: float = field(default=0.0)  # supplied MEAN_MOTION_DDOT [rev/day^3]; see class convention
    bStar: float = field(default=0.0)  # B* drag term [1/Earth radii]
    centerName: str = field(default="EARTH")  # CENTER_NAME
    refFrame: str = field(default="TEME")  # REF_FRAME
    meanElementTheory: str = field(default="SGP4")  # MEAN_ELEMENT_THEORY
    timeSystem: str = field(default="UTC")  # TIME_SYSTEM of ommEpoch

    def __setattr__(self, name, value):
        # Only allow setting attributes that are defined in the dataclass
        if name not in self.__dataclass_fields__:
            raise AttributeError(f"Cannot set attribute '{name}' on OmmData instance")
        object.__setattr__(self, name, value)


# ---------------------------------------------------------------------------------------------------------- #
#                                        OMM format detection                                                 #
# ---------------------------------------------------------------------------------------------------------- #
def _detectOmmFormat(text: str) -> str:
    """
    Identify which of the four CelesTrak OMM encodings a file holds.

    :param text: full text of the OMM file
    :return: one of "xml", "json", "csv", "kvn"
    """
    stripped = text.lstrip()
    if not stripped:
        raise ValueError("satOmm2elem() was given an empty OMM file.")

    if stripped.startswith("<"):
        return "xml"
    if stripped[0] in "[{":
        return "json"

    firstLine = stripped.splitlines()[0]
    # KVN assigns one field per line; the CSV header is a bare comma-separated field list.
    if "=" in firstLine:
        return "kvn"
    if "," in firstLine and "EPOCH" in firstLine.upper():
        return "csv"

    raise ValueError(
        "satOmm2elem() could not determine the OMM format. Expected XML, JSON, CSV or KVN; "
        f"the first line was: {firstLine.strip()!r}"
    )


# ---------------------------------------------------------------------------------------------------------- #
#                                          OMM parsers                                                        #
# ---------------------------------------------------------------------------------------------------------- #
def _collect_omm_fields(pairs) -> dict:
    """Normalize field names while retaining duplicate consumed fields for rejection.

    :param pairs: iterable of field name/value pairs, including possible duplicates
    :return: field dictionary with duplicate markers that survive later occurrences
    """
    fields = {}
    for key, value in pairs:
        key = str(key).upper()
        if key in _OMM_FIELDS and key in fields:
            value = _DUPLICATE_OMM_FIELD
        fields[key] = value
    return fields


def _parseOmmKvn(text: str) -> list:
    """
    Parse the CCSDS keyword-value notation (KVN) encoding.

    Each record starts at a ``CCSDS_OMM_VERS`` key, so a single file may hold many satellites.

    :param text: full text of the KVN file
    :return: list of raw field dictionaries retaining numeric unit declarations
    """
    records = []
    current = {}
    for rawLine in text.splitlines():
        line = rawLine.strip()
        # Skip blanks and CCSDS comment lines
        if not line or line.startswith("COMMENT"):
            continue
        if "=" not in line:
            continue

        key, _, value = line.partition("=")
        key = key.strip().upper()
        value = value.strip()
        # Only numeric fields have units; brackets in names are ordinary text.
        if key in _OMM_FIELD_UNITS and value.endswith("]") and "[" in value:
            number, _, units = value.partition("[")
            value = _OmmFieldWithUnits(number.strip(), units[:-1].strip())

        if key == "CCSDS_OMM_VERS" and current:
            records.append(current)
            current = {}
        current[key] = _DUPLICATE_OMM_FIELD if key in _OMM_FIELDS and key in current else value

    if current:
        records.append(current)
    return records


def _parseOmmJson(text: str) -> list:
    """
    Parse the JSON encoding, which CelesTrak serves as a list of flat field objects.

    Keep every array entry so validation can report malformed records individually
    using their original positions in the file.

    :param text: full text of the JSON file
    :return: list of raw JSON records, to be validated and normalized individually
    :raises ValueError: if the JSON document is invalid or is not an object or array
    """
    payload = json.loads(text, object_pairs_hook=_collect_omm_fields)
    if isinstance(payload, dict):
        return [payload]
    if not isinstance(payload, list):
        raise ValueError("OMM JSON must contain an object or an array of records.")
    return payload


def _validate_omm_xml_children(block, allowed) -> None:
    """Check recognized containers recursively without interpreting extensions.

    Segments are validated separately to retain per-record error handling. Standard
    containers whose fields are not consumed still reject misplaced consumed fields.

    :param block: XML element whose direct children are being validated
    :param allowed: field or block names permitted at this location
    :raises ValueError: if a consumed field or recognized block is misplaced
    """
    for child in block:
        name = child.tag.upper() if child.tag.upper() in _OMM_FIELDS else child.tag
        if name in _OMM_FIELDS or name in _OMM_XML_BLOCKS:
            if name not in allowed:
                raise ValueError(f"Misplaced OMM XML element {name} in {block.tag}.")
            if name in _OMM_FIELDS and len(child):
                raise ValueError(f"OMM XML field {name} must contain text without child elements.")
            if name in _OMM_XML_BLOCK_CHILDREN:
                _validate_omm_xml_children(child, _OMM_XML_BLOCK_CHILDREN[name])


def _parse_omm_xml_segment(segment) -> dict:
    """Extract one segment while checking block placement and duplicate elements.

    :param segment: OMM segment with normalized CCSDS tag names
    :return: raw fields, retaining units and duplicate markers
    :raises ValueError: if a field or block is misplaced
    """
    blocks = {
        "metadata": segment.findall("metadata"),
        "data": segment.findall("data"),
    }
    if len(blocks["data"]) == 1:
        data = blocks["data"][0]
        blocks["meanElements"] = data.findall("meanElements")
        blocks["tleParameters"] = data.findall("tleParameters")
    duplicates = {
        name: _DUPLICATE_OMM_FIELD for name, matches in blocks.items() if len(matches) > 1
    }
    if duplicates:
        return duplicates

    _validate_omm_xml_children(segment, {"metadata", "data"})
    fields = {}
    for name in _OMM_XML_BLOCK_FIELDS:
        for block in blocks.get(name, []):
            for element in block:
                key = element.tag.upper()
                if key in _OMM_FIELDS and key in fields:
                    fields[key] = _DUPLICATE_OMM_FIELD
                    continue
                value = (element.text or "").strip()
                if key in _OMM_FIELD_UNITS and "units" in element.attrib:
                    value = _OmmFieldWithUnits(value, element.attrib["units"])
                fields[key] = value
    return fields


def _parseOmmXml(text: str) -> list:
    """
    Parse the XML (CCSDS NDM) encoding.

    Accept unqualified elements and elements in the ``urn:ccsds:schema:ndmxml``
    namespace, independently of their prefix or use of a default namespace.
    Retain the available fields of every segment, including those missing metadata
    or data, so per-record validation reports their original positions in the file.
    Mark duplicate OMM fields and blocks for rejection during per-record validation.

    :param text: full text of the XML file
    :return: raw fields or invalid-record markers, retaining segment positions
    """
    root = ET.fromstring(text)
    # Normalize only CCSDS tags so foreign namespaces cannot override OMM fields.
    for element in root.iter():
        element.tag = element.tag.removeprefix("{urn:ccsds:schema:ndmxml}")
    if root.tag == "ndm":
        _validate_omm_xml_children(root, {"omm"})
    messages = [root] if root.tag == "omm" else root.findall("omm") if root.tag == "ndm" else []
    records = []
    for message in messages:
        message_error = None
        try:
            if len(message.findall("body")) > 1:
                raise ValueError("Duplicate OMM XML element: BODY.")
            _validate_omm_xml_children(message, {"header", "body"})
        except ValueError as error:
            message_error = _OmmInvalidRecord(str(error))
        for segment in message.findall("body/segment"):
            if message_error is not None:
                records.append(message_error)
                continue
            try:
                records.append(_parse_omm_xml_segment(segment))
            except ValueError as error:
                records.append(_OmmInvalidRecord(str(error)))
    return records


def _parseOmmCsv(text: str) -> list:
    """
    Parse CSV records with explicit header and row-width validation.

    :param text: full text of the CSV file
    :return: field dictionaries or invalid-record markers, in original record order
    :raises ValueError: if consumed header names are duplicated
    :raises csv.Error: if CSV quoting is malformed
    """
    # Match format detection's leading-whitespace handling without changing quoted cells.
    reader = csv.reader(io.StringIO(text.lstrip(), newline=""), strict=True)
    headers = next(reader, [])
    fields = _collect_omm_fields((name, "") for name in headers)
    duplicates = [name for name, value in fields.items() if value is _DUPLICATE_OMM_FIELD]
    if duplicates:
        raise ValueError(f"Duplicate OMM CSV field(s) in header: {', '.join(duplicates)}.")
    records = []
    for row in reader:
        if not row:
            continue
        if len(row) != len(headers):
            records.append(_OmmInvalidRecord(
                f"OMM CSV row has {len(row)} cells; expected {len(headers)} from the header."
            ))
            continue
        records.append(_collect_omm_fields(zip(headers, row)))
    return records


_OMM_PARSERS = {
    "xml": _parseOmmXml,
    "json": _parseOmmJson,
    "csv": _parseOmmCsv,
    "kvn": _parseOmmKvn,
}


# ---------------------------------------------------------------------------------------------------------- #
#                                      OMM field normalization                                                #
# ---------------------------------------------------------------------------------------------------------- #
def _parseOmmEpoch(epochStr: str) -> dt.datetime:
    """
    Parse an OMM ISO-8601 epoch, with or without fractional seconds or a trailing "Z".

    Additional fractional digits are truncated to microseconds, matching the
    precision of ``datetime`` and the ``sgp4.omm.initialize`` input. Day-of-year
    dates must fall within the stated calendar year.

    :param epochStr: the raw ``EPOCH`` field
    :return: the epoch as a datetime
    :raises ValueError: if the epoch is malformed or its date is invalid
    """
    cleaned = epochStr.strip().rstrip("Zz")
    whole_seconds, separator, fraction = cleaned.partition(".")
    # Validate the entire fraction before trimming so malformed suffixes cannot disappear.
    if separator and fraction.isascii() and fraction.isdecimal():
        cleaned = f"{whole_seconds}.{fraction[:6]}"

    for epochFormat in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S", "%Y-%m-%dT%H:%M"):
        try:
            return dt.datetime.strptime(cleaned, epochFormat)
        except ValueError:
            continue

    # CCSDS also permits a day-of-year form, e.g. "2026-259T12:00:00.000"
    for epochFormat in ("%Y-%jT%H:%M:%S.%f", "%Y-%jT%H:%M:%S"):
        try:
            epoch = dt.datetime.strptime(cleaned, epochFormat)
        except ValueError:
            continue
        # strptime rolls day 366 of a non-leap year into the following year.
        if epoch.year == int(cleaned[:4]):
            return epoch

    raise ValueError(f"satOmm2elem() could not parse the OMM EPOCH field: {epochStr!r}")


def _validate_omm_numeric_fields(fields: dict) -> None:
    """
    Validate numeric OMM fields before constructing metadata or initializing SGP4.

    Mean motion must be positive, eccentricity must lie in ``[0, 1)``, inclination
    in ``[0, 180]`` degrees, and the remaining angles in ``[0, 360]`` degrees.
    Drag terms and mean-motion derivatives may be negative but must be finite.

    :param fields: OMM field strings with required fields and defaults populated
    :raises ValueError: if a field is non-finite, malformed, or outside its allowed range
    """
    numeric_fields = {}
    for name in (
        "MEAN_MOTION", "ECCENTRICITY", "INCLINATION", "RA_OF_ASC_NODE",
        "ARG_OF_PERICENTER", "MEAN_ANOMALY", "BSTAR", "MEAN_MOTION_DOT",
        "MEAN_MOTION_DDOT",
    ):
        try:
            value = float(fields[name])
        except ValueError as error:
            raise ValueError(f"OMM {name} must be a finite number: {fields[name]!r}") from error
        if not np.isfinite(value):
            raise ValueError(f"OMM {name} must be a finite number: {fields[name]!r}")
        numeric_fields[name] = value

    if numeric_fields["MEAN_MOTION"] <= 0.0:  # [rev/day]
        raise ValueError("OMM MEAN_MOTION must be positive [rev/day].")
    if not 0.0 <= numeric_fields["ECCENTRICITY"] < 1.0:  # [-]
        raise ValueError("OMM ECCENTRICITY must satisfy 0 <= e < 1.")
    for name, upper_bound in (
        ("INCLINATION", 180.0),  # [deg]
        ("RA_OF_ASC_NODE", 360.0),  # [deg]
        ("ARG_OF_PERICENTER", 360.0),  # [deg]
        ("MEAN_ANOMALY", 360.0),  # [deg]
    ):
        if not 0.0 <= numeric_fields[name] <= upper_bound:  # [deg]
            raise ValueError(f"OMM {name} must be between 0 and {upper_bound} [deg].")

    for name in ("EPHEMERIS_TYPE", "ELEMENT_SET_NO", "REV_AT_EPOCH"):
        try:
            value = int(fields[name])
        except ValueError as error:
            raise ValueError(f"OMM {name} must be a non-negative integer: {fields[name]!r}") from error
        if value < 0:
            raise ValueError(f"OMM {name} must be a non-negative integer: {fields[name]!r}")


def _normalize_omm_physical_metadata(fields: dict, omm_format: str) -> None:
    """
    Validate and normalize physical metadata in place for the supported SGP4 profile.

    CelesTrak JSON and CSV may omit the four fixed metadata fields. XML and KVN
    must declare them. Explicitly empty or incompatible declarations are rejected
    in every encoding. Ephemeris type 0 is accepted for CelesTrak compatibility;
    type 2 explicitly identifies SGP4. CelesTrak's ``SGP/SGP4`` mean-element theory
    label is normalized to ``SGP4`` in every encoding.

    :param fields: normalized field strings with numeric fields already validated
    :param omm_format: detected encoding (``xml``, ``kvn``, ``json``, or ``csv``)
    :raises ValueError: if physical metadata is missing or unsupported
    """
    for name, expected in _SUPPORTED_OMM_METADATA.items():
        if name not in fields:
            if omm_format not in ("json", "csv"):
                raise ValueError(f"OMM {name} is required for {omm_format.upper()} records.")
            fields[name] = expected
        value = fields[name].strip().upper()
        if name == "MEAN_ELEMENT_THEORY" and value == "SGP/SGP4":
            value = "SGP4"
        if value != expected:
            raise ValueError(f"Unsupported OMM {name}: {fields[name]!r}; expected {expected}.")
        fields[name] = value

    if int(fields["EPHEMERIS_TYPE"]) not in (0, 2):
        raise ValueError(
            f"Unsupported OMM EPHEMERIS_TYPE: {fields['EPHEMERIS_TYPE']!r}; "
            "expected 0 (CelesTrak default) or 2 (SGP4)."
        )


def _normalizeOmmFields(fields: dict, omm_format: str) -> dict:
    """
    Validate one OMM record and normalize its fields for ``sgp4.omm.initialize``.

    :param fields: raw OMM field dictionary
    :param omm_format: detected encoding (``xml``, ``kvn``, ``json``, or ``csv``)
    :return: a normalized copy safe to hand to SGP4
    :raises ValueError: if the record is not a field dictionary or its fields are invalid
    """
    if isinstance(fields, _OmmInvalidRecord):
        raise ValueError(fields.reason)
    if not isinstance(fields, dict):
        raise ValueError(f"OMM record must be a field object, got {type(fields).__name__}.")

    # Check declared units before discarding them or supplying optional defaults.
    # Validation stays inside the per-record error handler for XML and KVN alike.
    normalized = {}
    for key, value in _collect_omm_fields(fields.items()).items():
        if value is _DUPLICATE_OMM_FIELD:
            kind = "element" if omm_format == "xml" else "field"
            raise ValueError(f"Duplicate OMM {omm_format.upper()} {kind}: {key}.")
        if isinstance(value, _OmmFieldWithUnits):
            supported_units = _OMM_FIELD_UNITS[key]
            if value.units not in supported_units:
                expected = " or ".join(supported_units) if supported_units else "no unit declaration"
                raise ValueError(
                    f"Unsupported OMM {key} units: {value.units!r}; expected {expected}."
                )
            if not value.value:
                raise ValueError(f"OMM {key} has a unit declaration but no numeric value.")
            value = value.value
        # JSON also delivers native numbers and nulls.
        normalized[key] = "" if value is None else str(value)

    for key, default in _OPTIONAL_OMM_DEFAULTS.items():
        if not normalized.get(key, "").strip():
            normalized[key] = default

    missing = [key for key in _REQUIRED_OMM_FIELDS if not normalized.get(key, "").strip()]
    if missing:
        raise ValueError(
            f"satOmm2elem() found an OMM record missing required field(s): {', '.join(sorted(missing))}"
        )

    # Validate the external ID before replacing it in the private SGP4 input.
    catalog_id = normalized["NORAD_CAT_ID"].strip()
    catalog_digits = catalog_id.removeprefix("+")
    if not catalog_digits.isascii() or not catalog_digits.isdecimal():
        raise ValueError(
            f"satOmm2elem() requires NORAD_CAT_ID to be a non-negative decimal integer: {catalog_id!r}"
        )
    # Check the value's range without converting arbitrarily long digit strings to int.
    if len(catalog_digits.lstrip("0")) > 9:
        raise ValueError(f"OMM NORAD_CAT_ID must be between 0 and 999999999: {catalog_id!r}")

    _validate_omm_numeric_fields(normalized)
    _normalize_omm_physical_metadata(normalized, omm_format)

    classification_code = normalized["CLASSIFICATION_TYPE"].strip()
    if not classification_code.isascii() or classification_code.upper() not in _CLASSIFICATION_NAMES:
        raise ValueError(
            f"Unsupported OMM CLASSIFICATION_TYPE: {normalized['CLASSIFICATION_TYPE']!r}; "
            "expected U, C, or S."
        )
    normalized["CLASSIFICATION_TYPE"] = classification_code.upper()

    # sgp4.omm.initialize() parses EPOCH with a strict "%Y-%m-%dT%H:%M:%S.%f" format.
    epoch = _parseOmmEpoch(normalized["EPOCH"])
    normalized["EPOCH"] = epoch.strftime("%Y-%m-%dT%H:%M:%S.%f")

    # Preserve missing or unknown designators in public metadata without inventing an ID.
    normalized["OBJECT_ID"] = normalized.get("OBJECT_ID", "").strip()

    return normalized


def _ommFields2Data(fields: dict, epoch: dt.datetime) -> OmmData:
    """
    Build an :py:class:`OmmData` from a normalized OMM field dictionary.

    The osculating elements are filled in later by :py:func:`_convertOmmMean2osculating`.

    :param fields: normalized OMM field dictionary
    :param epoch: the parsed OMM epoch
    :return: the populated OmmData, with a placeholder ``oe``
    """
    return OmmData(
        oe=om.ClassicElements(),
        ommEpoch=epoch,
        satName=fields.get("OBJECT_NAME", "BSK-Sat-00").strip() or "BSK-Sat-00",
        noradID=str(fields["NORAD_CAT_ID"]).strip(),
        objectID=fields.get("OBJECT_ID", "").strip(),
        classification=_CLASSIFICATION_NAMES[fields["CLASSIFICATION_TYPE"]],
        revAtEpoch=int(fields["REV_AT_EPOCH"]),
        propagator=str(fields.get("EPHEMERIS_TYPE", "0")).strip(),
        elemSetNo=int(fields["ELEMENT_SET_NO"]),
        meanMotion=float(fields["MEAN_MOTION"]),  # [rev/day]
        nDot=float(fields["MEAN_MOTION_DOT"]),  # [rev/day^2]
        nDotDot=float(fields["MEAN_MOTION_DDOT"]),  # [rev/day^3]
        bStar=float(fields["BSTAR"]),  # [1/Earth radii]
        centerName=fields["CENTER_NAME"],
        refFrame=fields["REF_FRAME"],
        meanElementTheory=fields["MEAN_ELEMENT_THEORY"],
        timeSystem=fields["TIME_SYSTEM"],
    )


# ---------------------------------------------------------------------------------------------------------- #
#           Conversion of OMM (mean OE in TEME frame) -> osculating OE in J2000/ICRS frame                    #
# ---------------------------------------------------------------------------------------------------------- #
def _convertOmmMean2osculating(fields: dict, ommData: OmmData) -> om.ClassicElements:
    """
    Convert OMM mean orbital elements (NORAD/SGP4) to the osculating elements Basilisk uses.

    sgp4MeanOE -> SGP4 -> state vector -> osculating OE

    Mirrors :py:func:`Basilisk.utilities.tleHandling._convertMean2osculating`; the only
    difference is that the SGP4 record is initialized from OMM fields rather than two lines.

    Propagation uses SGP4's stored initialization epoch, avoiding a separate calendar
    conversion that could shift the propagation time.

    :param fields: normalized OMM field dictionary
    :param ommData: the OmmData carrying the epoch and identifiers, used for error reporting
    :return: osculating classical orbital elements in the Basilisk inertial frame
    """
    # SGP4 limits its internal Alpha-5 ID to 339999, but the ID does not affect
    # propagation. Preserve the real catalog number in fields and ommData.
    sgp4_fields = dict(fields, NORAD_CAT_ID="0")
    # Older SGP4 initializers slice every OBJECT_ID as a YYYY-NNNP designator.
    # Use an empty internal identifier for unavailable or non-designator text.
    object_id = fields["OBJECT_ID"]
    sgp4_fields["OBJECT_ID"] = object_id if object_id[4:5] == "-" else ""
    satellite = Satrec()

    # Propagate to epoch to get the True Equator, Mean Equinox (TEME) state vector.
    # Extreme finite inputs can still exceed the backend's numerical capacity.
    try:
        sgp4omm.initialize(satellite, sgp4_fields)
        e, r, v = satellite.sgp4(satellite.jdsatepoch, satellite.jdsatepochF)
    except (OverflowError, ZeroDivisionError) as error:
        raise ValueError(
            f"SGP4 numerical failure for satellite {ommData.satName} with NORAD ID "
            f"{ommData.noradID}: {error}"
        ) from error

    if e != 0:
        raise ValueError(
            f"SGP4 propagation failed for satellite {ommData.satName} with NORAD ID "
            f"{ommData.noradID} at epoch {ommData.ommEpoch}. Error code: {e}"
        )

    # Convert km -> m for Basilisk
    with np.errstate(over="ignore"):
        r_teme_m = np.array(r) * 1e3  # [m]
        v_teme_m = np.array(v) * 1e3  # [m/s]
    if not np.all(np.isfinite(r_teme_m)) or not np.all(np.isfinite(v_teme_m)):
        raise ValueError(
            f"SGP4 produced a non-finite state in SI units for satellite {ommData.satName} "
            f"with NORAD ID {ommData.noradID} at epoch {ommData.ommEpoch}."
        )

    # Convert TEME -> J2000/ICRF (Basilisk inertial frame)
    r_m, v_m = _teme2j2000(r_teme_m, v_teme_m, ommData.ommEpoch)

    # Convert state vector to osculating orbital elements
    return om.rv2elem(om.MU_EARTH * 1e9, r_m, v_m)  # MU_EARTH km^3/s^2 -> [m^3/s^2]


# ---------------------------------------------------------------------------------------------------------- #
#                                            Public interface                                                 #
# ---------------------------------------------------------------------------------------------------------- #
def satOmm2elem(omm_path: str) -> list:
    """
    Convert the OMM records of a constellation to osculating orbital elements for each satellite.

    The encoding (XML, JSON, CSV or KVN) is detected from the file contents. Catalog numbers,
    including nine-digit OMM IDs, are preserved in ``noradID`` without being constrained by
    SGP4's internal identifier storage. Values must be in ``[0, 999999999]``;
    optional leading ``+`` signs and leading zeros are accepted and preserved.
    Missing, null or blank international designators yield ``objectID=""``.
    Supplied designators, including the literal ``UNKNOWN``, are preserved after
    trimming whitespace; backend identifier accommodations do not change this metadata.
    Mean-motion derivative fields retain the supplied coefficients without rescaling;
    see :py:class:`OmmData` for CelesTrak's TLE coefficient convention.

    XML accepts unqualified elements and the ``urn:ccsds:schema:ndmxml`` namespace,
    using either prefixes or a default namespace. Records are read from
    ``omm/body/segment`` in a standalone OMM or a direct OMM child of an NDM container.
    Segments inside extensions or other message types are ignored. Consumed fields
    must appear in their owning metadata, mean-elements or TLE-parameters blocks.
    This placement check includes headers, bodies and recognized optional containers.
    Errors outside a segment invalidate all segments in that OMM message; errors
    within a segment invalidate only that record. Other OMM messages remain available.
    Misplaced fields or blocks directly under NDM are file-level errors.
    XML records containing misplaced or duplicate OMM fields or blocks are skipped
    with a warning before SGP4 is invoked.
    KVN and JSON records with duplicate consumed fields are also skipped, including
    names that collide after case normalization. Duplicate consumed CSV headers
    make the whole file ambiguous and raise ``ValueError`` before conversion.
    CSV accepts leading blank lines and quoted cells, including embedded newlines.
    Rows with missing or extra cells are skipped before optional defaults are applied;
    optional columns omitted from the header still receive their normal defaults.

    Calendar-date and day-of-year epochs accept arbitrary fractional-second digits.
    Digits beyond microsecond precision are truncated consistently for ``ommEpoch``,
    SGP4 initialization and propagation, and frame conversion.

    Only Earth-centered, TEME, UTC, SGP4 records with ephemeris type 0 (CelesTrak
    default) or 2 (SGP4) are supported. JSON and CSV records may omit
    ``CENTER_NAME``, ``REF_FRAME``, ``TIME_SYSTEM``, and ``MEAN_ELEMENT_THEORY``;
    CelesTrak's EARTH/TEME/UTC/SGP4 defaults then apply. XML and KVN require these
    fields explicitly. Empty or incompatible declarations cause the record to
    be skipped with a warning before SGP4 is invoked.

    CelesTrak's ``MEAN_ELEMENT_THEORY = SGP/SGP4`` is accepted as an alias for
    ``SGP4`` in every encoding, after trimming whitespace and normalizing case.
    ``OmmData.meanElementTheory`` stores the canonical value ``SGP4``.

    Numeric XML/KVN unit declarations must match the CCSDS units used by SGP4:
    ``deg``, ``rev/day``, ``rev/day**2``, ``rev/day**3``, and inverse Earth radii
    (``1/ER`` or ``1/[Earth radii]``) for ``BSTAR``. Dimensionless fields must
    omit units. Unsupported declarations cause the record to be skipped;
    values are not converted between units. Omitted units use these same conventions.

    ``CLASSIFICATION_TYPE`` accepts ``U``, ``C``, or ``S`` after trimming whitespace
    and normalizing case. Missing or blank values default to ``U``; unsupported
    values cause the record to be skipped before SGP4 is invoked.

    :param omm_path: path to an OMM file holding one or many satellites
    :return: ommDataList: list of :py:class:`OmmData`, one per satellite, each carrying the
             osculating orbital elements in ``oe`` plus the record's metadata
    """
    with open(omm_path, "r", encoding="utf-8-sig") as ommFile:
        text = ommFile.read()

    ommFormat = _detectOmmFormat(text)
    records = _OMM_PARSERS[ommFormat](text)

    if not records:
        raise ValueError(f"satOmm2elem() found no OMM records in {omm_path}.")

    ommDataList = []
    for recordNo, rawFields in enumerate(records, start=1):
        try:
            fields = _normalizeOmmFields(rawFields, ommFormat)
            epoch = _parseOmmEpoch(fields["EPOCH"])
            ommData = _ommFields2Data(fields, epoch)
            ommData.oe = _convertOmmMean2osculating(fields, ommData)
        except (ValueError, KeyError) as error:
            print(f"WARNING: satOmm2elem() skipped OMM record {recordNo} in {omm_path}: {error}")
            continue
        ommDataList.append(ommData)

    return ommDataList
