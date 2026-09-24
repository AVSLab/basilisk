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
#   Unit Test Script
#   Module Name: ommHandling
#   Author: robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date: September 16, 2026
#

import datetime as dt
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from sgp4.model import Satrec as PythonSatrec

import Basilisk.utilities.ommHandling as ommHandling

A_TOL_DIST = 1e-6  # [m] semi-major axis agreement between OMM encodings
A_TOL = 1e-12  # [-] eccentricity; [rad] angles; agreement between encodings
A_TOL_TLE_DIST = 0.1  # [m] OMM vs TLE agreement, limited by the TLE's fixed field widths
A_TOL_TLE_ANG = 1e-6  # [rad] OMM vs TLE angular agreement

# ISS (ZARYA), one OMM record expressed in each of the four CelesTrak encodings.
# All four carry bit-identical field values, so every encoding must yield the same elements.
_OMM_FIELDS = {
    "OBJECT_NAME": "ISS (ZARYA)",
    "OBJECT_ID": "1998-067A",
    "CENTER_NAME": "EARTH",
    "REF_FRAME": "TEME",
    "TIME_SYSTEM": "UTC",
    "MEAN_ELEMENT_THEORY": "SGP4",
    "EPOCH": "2026-09-10T12:00:00.000000",
    "MEAN_MOTION": "15.50103472",
    "ECCENTRICITY": "0.0004364",
    "INCLINATION": "51.6416",
    "RA_OF_ASC_NODE": "247.4627",
    "ARG_OF_PERICENTER": "130.5360",
    "MEAN_ANOMALY": "325.0288",
    "EPHEMERIS_TYPE": "0",
    "CLASSIFICATION_TYPE": "U",
    "NORAD_CAT_ID": "25544",
    "ELEMENT_SET_NO": "999",
    "REV_AT_EPOCH": "47859",
    "BSTAR": "0.00016717",
    "MEAN_MOTION_DOT": "0.00002182",
    "MEAN_MOTION_DDOT": "0.0",
}


def _writeKvn(path, fieldSets):
    lines = []
    for fields in fieldSets:
        lines.append("CCSDS_OMM_VERS = 2.0")
        lines.append("COMMENT this comment line must be ignored")
        for key, value in fields.items():
            lines.append(f"{key} = {value}")
    path.write_text("\n".join(lines) + "\n")
    return path


def _writeJson(path, fieldSets):
    path.write_text(json.dumps(list(fieldSets)))
    return path


def _writeCsv(path, fieldSets):
    fieldSets = list(fieldSets)
    header = ",".join(fieldSets[0].keys())
    rows = [",".join(str(v) for v in fields.values()) for fields in fieldSets]
    path.write_text("\n".join([header] + rows) + "\n")
    return path


def _writeXml(path, fieldSets):
    segments = []
    for fields in fieldSets:
        metadataKeys = ("OBJECT_NAME", "OBJECT_ID", "CENTER_NAME", "REF_FRAME",
                        "TIME_SYSTEM", "MEAN_ELEMENT_THEORY")
        meanKeys = ("EPOCH", "MEAN_MOTION", "ECCENTRICITY", "INCLINATION",
                    "RA_OF_ASC_NODE", "ARG_OF_PERICENTER", "MEAN_ANOMALY")
        tleKeys = ("EPHEMERIS_TYPE", "CLASSIFICATION_TYPE", "NORAD_CAT_ID",
                   "ELEMENT_SET_NO", "REV_AT_EPOCH", "BSTAR",
                   "MEAN_MOTION_DOT", "MEAN_MOTION_DDOT")

        def block(name, keys):
            inner = "".join(f"<{k}>{fields[k]}</{k}>" for k in keys if k in fields)
            return f"<{name}>{inner}</{name}>"

        segments.append(
            "<segment>"
            + block("metadata", metadataKeys)
            + "<data>" + block("meanElements", meanKeys) + block("tleParameters", tleKeys) + "</data>"
            + "</segment>"
        )
    path.write_text(f"<ndm><omm>{''.join(segments)}</omm></ndm>")
    return path


_WRITERS = {"kvn": _writeKvn, "json": _writeJson, "csv": _writeCsv, "xml": _writeXml}


def _write_omm_with_units(path, encoding, field_sets, unit_sets):
    """Add KVN annotations or XML attributes to selected numeric fields."""
    if encoding == "kvn":
        annotated = []
        for fields, units in zip(field_sets, unit_sets):
            annotated.append({name: f"{value} [{units[name]}]" if name in units else value
                              for name, value in fields.items()})
        return _writeKvn(path, annotated)

    _writeXml(path, field_sets)
    tree = ET.parse(path)
    for segment, units in zip(tree.findall(".//segment"), unit_sets):
        for name, unit in units.items():
            segment.find(f".//{name}").set("units", unit)
    tree.write(path, encoding="utf-8")
    return path


@pytest.mark.parametrize("encoding", ["kvn", "xml"])
@pytest.mark.parametrize("bstar_unit", ["1/ER", "1/[Earth radii]"])
def test_omm_supported_units(tmp_path, capsys, encoding, bstar_unit):
    """CCSDS unit declarations preserve the orbit and retained mean elements."""
    units = {
        "MEAN_MOTION": "rev/day",
        "INCLINATION": "deg",
        "RA_OF_ASC_NODE": "deg",
        "ARG_OF_PERICENTER": "deg",
        "MEAN_ANOMALY": "deg",
        "BSTAR": bstar_unit,
        "MEAN_MOTION_DOT": "rev/day**2",
        "MEAN_MOTION_DDOT": "rev/day**3",
    }
    path = _write_omm_with_units(tmp_path / f"units.{encoding}", encoding, [_OMM_FIELDS], [units])
    reference_path = _WRITERS[encoding](tmp_path / f"reference.{encoding}", [_OMM_FIELDS])

    records = ommHandling.satOmm2elem(str(path))
    reference = ommHandling.satOmm2elem(str(reference_path))[0]

    assert len(records) == 1
    for name in ("a", "e", "i", "Omega", "omega", "f"):
        assert getattr(records[0].oe, name) == getattr(reference.oe, name)
    for name in ("meanMotion", "nDot", "nDotDot", "bStar"):
        assert getattr(records[0], name) == getattr(reference, name)
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("encoding", ["kvn", "xml"])
@pytest.mark.parametrize("field_name,unit", [
    ("MEAN_MOTION", "rad/s"),
    ("INCLINATION", "rad"),
    ("RA_OF_ASC_NODE", "rad"),
    ("ARG_OF_PERICENTER", "rad"),
    ("MEAN_ANOMALY", "rad"),
    ("BSTAR", "1/km"),
    ("MEAN_MOTION_DOT", "rev/day"),
    ("MEAN_MOTION_DDOT", "rev/day**2"),
    ("ECCENTRICITY", "n/a"),
    ("NORAD_CAT_ID", "km"),
    ("EPHEMERIS_TYPE", "km"),
    ("ELEMENT_SET_NO", "km"),
    ("REV_AT_EPOCH", "km"),
    ("INCLINATION", "DEG"),
    ("INCLINATION", "unknown"),
    ("INCLINATION", ""),
])
def test_omm_unsupported_units_are_skipped(tmp_path, monkeypatch, capsys, encoding, field_name, unit):
    """Incompatible, unknown, or empty units never reach SGP4; later records survive."""
    warning = _check_invalid_record(
        tmp_path, monkeypatch, capsys, encoding, field_name, _OMM_FIELDS[field_name], units=unit
    )
    assert "units" in warning
    assert repr(unit) in warning


@pytest.mark.parametrize("encoding", ["kvn", "xml"])
@pytest.mark.parametrize("unit", ["1/km", "1/ER"])
def test_omm_units_without_numeric_value_are_skipped(tmp_path, monkeypatch, capsys, encoding, unit):
    """An annotation without a number cannot silently select an optional default."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, "BSTAR", "", units=unit)


@pytest.mark.parametrize("name", ["SATELLITE [TEST]", "SATELLITE [deg]", "123 [km]", "[TEST]"])
def test_omm_kvn_preserves_bracketed_names(tmp_path, capsys, name):
    """Brackets in textual metadata are not interpreted as numeric unit declarations."""
    path = _writeKvn(tmp_path / "bracketed-name.kvn", [dict(_OMM_FIELDS, OBJECT_NAME=name)])

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    assert records[0].satName == name
    assert capsys.readouterr().out == ""


_PHYSICAL_METADATA_FIELDS = ("CENTER_NAME", "REF_FRAME", "TIME_SYSTEM", "MEAN_ELEMENT_THEORY")


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("field_name,value", [
    ("CENTER_NAME", "MARS"),
    ("REF_FRAME", "GCRF"),
    ("REF_FRAME", "ITRF"),
    ("TIME_SYSTEM", "TAI"),
    ("TIME_SYSTEM", "TT"),
    ("MEAN_ELEMENT_THEORY", "DSST"),
    ("MEAN_ELEMENT_THEORY", "SGP4-XP"),
    ("MEAN_ELEMENT_THEORY", "SGP"),
    ("EPHEMERIS_TYPE", "1"),
    ("EPHEMERIS_TYPE", "3"),
    ("EPHEMERIS_TYPE", "4"),
    ("EPHEMERIS_TYPE", "5"),
    ("EPHEMERIS_TYPE", "6"),
    ("EPHEMERIS_TYPE", "99"),
])
def test_omm_unsupported_physical_metadata_is_skipped(
    tmp_path, monkeypatch, capsys, encoding, field_name, value
):
    """Unsupported physical declarations never reach SGP4; valid records still return."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, field_name, value)


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("field_name", _PHYSICAL_METADATA_FIELDS)
@pytest.mark.parametrize("value", ["", " ", None])
def test_omm_empty_physical_metadata_is_skipped(
    tmp_path, monkeypatch, capsys, encoding, field_name, value
):
    """Explicitly empty or null physical metadata cannot silently select defaults."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, field_name, value)


@pytest.mark.parametrize("encoding", ["json", "csv"])
@pytest.mark.parametrize("omitted_fields", [
    ("CENTER_NAME",), ("REF_FRAME",), ("TIME_SYSTEM",), ("MEAN_ELEMENT_THEORY",),
    _PHYSICAL_METADATA_FIELDS,
])
def test_omm_celestrak_physical_metadata_defaults(tmp_path, capsys, encoding, omitted_fields):
    """CelesTrak JSON/CSV omissions give the same orbit as explicit physical metadata."""
    fields = {name: value for name, value in _OMM_FIELDS.items() if name not in omitted_fields}
    path = _WRITERS[encoding](tmp_path / f"defaults.{encoding}", [fields])
    reference_path = _WRITERS[encoding](tmp_path / f"explicit.{encoding}", [_OMM_FIELDS])

    records = ommHandling.satOmm2elem(str(path))
    reference = ommHandling.satOmm2elem(str(reference_path))[0]

    assert len(records) == 1
    record = records[0]
    assert (record.centerName, record.refFrame, record.timeSystem, record.meanElementTheory) == (
        "EARTH", "TEME", "UTC", "SGP4",
    )
    for name in ("a", "e", "i", "Omega", "omega", "f"):
        assert getattr(record.oe, name) == getattr(reference.oe, name)
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("encoding", ["kvn", "xml"])
@pytest.mark.parametrize("field_name", _PHYSICAL_METADATA_FIELDS)
def test_omm_missing_required_physical_metadata_is_skipped(
    tmp_path, monkeypatch, capsys, encoding, field_name
):
    """CCSDS XML/KVN records must explicitly declare their physical metadata."""
    fields = {name: value for name, value in _OMM_FIELDS.items() if name != field_name}
    path = _WRITERS[encoding](tmp_path / f"missing-metadata.{encoding}", [fields])

    def unexpected_initialize(*args, **kwargs):
        """Fail if a record with missing metadata reaches SGP4."""
        pytest.fail("SGP4 received a record with missing physical metadata")

    monkeypatch.setattr(ommHandling.sgp4omm, "initialize", unexpected_initialize)

    assert ommHandling.satOmm2elem(str(path)) == []
    warning = capsys.readouterr().out
    assert "skipped OMM record 1" in warning
    assert field_name in warning


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("ephemeris_type", [None, "0", "2"])
def test_omm_supported_physical_metadata(tmp_path, capsys, encoding, ephemeris_type):
    """Supported metadata accepts case/whitespace variations and compatible ephemeris types."""
    fields = dict(_OMM_FIELDS)
    for name in _PHYSICAL_METADATA_FIELDS:
        fields[name] = f" {_OMM_FIELDS[name].lower()} "
    if ephemeris_type is None:
        del fields["EPHEMERIS_TYPE"]
    else:
        fields["EPHEMERIS_TYPE"] = ephemeris_type
    path = _WRITERS[encoding](tmp_path / f"supported-metadata.{encoding}", [fields])

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    record = records[0]
    assert (record.centerName, record.refFrame, record.timeSystem, record.meanElementTheory) == (
        "EARTH", "TEME", "UTC", "SGP4",
    )
    assert record.propagator == (ephemeris_type or "0")
    assert all(np.isfinite(getattr(record.oe, name)) for name in ("a", "e", "i", "Omega", "omega", "f"))
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
def test_omm_each_encoding_parses(tmp_path, encoding):
    """Every CelesTrak OMM encoding is detected and parsed into one populated record."""
    path = _WRITERS[encoding](tmp_path / f"iss.{encoding}", [_OMM_FIELDS])

    ommDataList = ommHandling.satOmm2elem(str(path))

    assert len(ommDataList) == 1
    ommData = ommDataList[0]
    assert ommData.satName == "ISS (ZARYA)"
    assert ommData.noradID == "25544"
    assert ommData.objectID == "1998-067A"
    assert ommData.classification == "Unclassified"
    assert ommData.refFrame == "TEME"
    assert ommData.ommEpoch.year == 2026
    assert ommData.ommEpoch.month == 9
    assert ommData.ommEpoch.day == 10
    # A sane LEO orbit came back
    assert 6.6e6 < ommData.oe.a < 7.0e6  # [m]
    assert 0.0 <= ommData.oe.e < 0.01  # [-]
    assert np.isclose(np.degrees(ommData.oe.i), 51.6416, rtol=0.0, atol=0.5)  # [deg], atol [deg]


def test_omm_encodings_agree(tmp_path):
    """All four encodings of identical field values agree within absolute tolerances."""
    results = {}
    for encoding, writer in _WRITERS.items():
        path = writer(tmp_path / f"iss.{encoding}", [_OMM_FIELDS])
        results[encoding] = ommHandling.satOmm2elem(str(path))[0].oe

    reference = results["kvn"]
    for encoding, oe in results.items():
        assert np.isclose(oe.a, reference.a, rtol=0.0, atol=A_TOL_DIST), encoding
        for name in ("e", "i", "Omega", "omega", "f"):
            assert np.isclose(
                getattr(oe, name), getattr(reference, name), rtol=0.0, atol=A_TOL
            ), (encoding, name)


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("catalog_id", [
    "00001", "99999", "100000", "100001", "339999", "340000",
    "999999", "799500001", "999999999",
])
def test_omm_catalog_numbers(tmp_path, monkeypatch, encoding, catalog_id):
    """Catalog IDs survive ingestion unchanged without affecting the orbital elements."""
    monkeypatch.setattr(ommHandling, "Satrec", PythonSatrec)

    fields = dict(_OMM_FIELDS)
    fields["NORAD_CAT_ID"] = catalog_id
    fields["OBJECT_NAME"] = "NEW-CATALOG-OBJECT"
    path = _WRITERS[encoding](tmp_path / f"catalog.{encoding}", [_OMM_FIELDS, fields])

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 2
    reference, actual = records
    assert reference.noradID == _OMM_FIELDS["NORAD_CAT_ID"]
    assert actual.noradID == catalog_id
    assert actual.satName == "NEW-CATALOG-OBJECT"
    for name in ("a", "e", "i", "Omega", "omega", "f"):
        assert getattr(actual.oe, name) == getattr(reference.oe, name), name


@pytest.mark.parametrize("catalog_id", ["A0001", "100000.5", "-1", "nan"])
def test_omm_invalid_catalog_number_is_skipped(tmp_path, capsys, catalog_id):
    """Substituting an internal SGP4 ID must not admit malformed catalog numbers."""
    fields = dict(_OMM_FIELDS, NORAD_CAT_ID=catalog_id)
    path = _writeJson(tmp_path / "invalid-catalog.json", [fields, _OMM_FIELDS])

    records = ommHandling.satOmm2elem(str(path))

    assert [record.noradID for record in records] == [_OMM_FIELDS["NORAD_CAT_ID"]]
    warning = capsys.readouterr().out
    assert "skipped OMM record 1" in warning
    assert "NORAD_CAT_ID" in warning
    assert catalog_id in warning


def test_omm_multiple_records(tmp_path):
    """A constellation file yields one record per satellite, in file order."""
    second = dict(_OMM_FIELDS)
    second["OBJECT_NAME"] = "SECOND-SAT"
    second["NORAD_CAT_ID"] = "25545"
    second["INCLINATION"] = "97.4000"
    path = _writeKvn(tmp_path / "constellation.kvn", [_OMM_FIELDS, second])

    ommDataList = ommHandling.satOmm2elem(str(path))

    assert [d.satName for d in ommDataList] == ["ISS (ZARYA)", "SECOND-SAT"]
    assert not np.isclose(ommDataList[0].oe.i, ommDataList[1].oe.i)


def test_omm_optional_fields_defaulted(tmp_path):
    """Records omitting the optional SGP4 fields still parse, using CCSDS defaults."""
    fields = {k: v for k, v in _OMM_FIELDS.items()
              if k not in ("BSTAR", "MEAN_MOTION_DOT", "MEAN_MOTION_DDOT",
                           "ELEMENT_SET_NO", "REV_AT_EPOCH")}
    path = _writeKvn(tmp_path / "sparse.kvn", [fields])

    ommDataList = ommHandling.satOmm2elem(str(path))

    assert len(ommDataList) == 1
    assert ommDataList[0].bStar == 0.0
    assert ommDataList[0].revAtEpoch == 0


@pytest.mark.parametrize("epochStr,expectedSecond", [
    ("2026-09-10T12:00:00.000000", 0),
    ("2026-09-10T12:00:30", 30),
    ("2026-09-10T12:00:30Z", 30),
    ("2026-253T12:00:30", 30),
])
def test_omm_epoch_formats(tmp_path, epochStr, expectedSecond):
    """The permitted CCSDS epoch spellings all parse, including day-of-year and 'Z'."""
    fields = dict(_OMM_FIELDS)
    fields["EPOCH"] = epochStr
    path = _writeKvn(tmp_path / "epoch.kvn", [fields])

    ommDataList = ommHandling.satOmm2elem(str(path))

    assert len(ommDataList) == 1
    assert ommDataList[0].ommEpoch.second == expectedSecond


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("epoch_string,normalized_epoch", [
    ("2026-09-10T12:00:30.1234567", "2026-09-10T12:00:30.123456"),
    ("2026-253T12:00:30.1234567", "2026-09-10T12:00:30.123456"),
    ("2026-09-10T12:00:30.123456789Z", "2026-09-10T12:00:30.123456"),
    ("2026-253T12:00:30.000000001Z", "2026-09-10T12:00:30.000000"),
    ("2026-09-10T12:00:30.123456789123456789", "2026-09-10T12:00:30.123456"),
    ("2026-253T12:00:30.000000000", "2026-09-10T12:00:30.000000"),
    ("2026-12-31T23:59:59.999999999Z", "2026-12-31T23:59:59.999999"),
    ("2026-365T23:59:59.999999999", "2026-12-31T23:59:59.999999"),
])
def test_omm_additional_epoch_fraction_digits(
    tmp_path, monkeypatch, capsys, encoding, epoch_string, normalized_epoch
):
    """Long epoch fractions truncate consistently to microseconds through propagation."""
    fields = dict(_OMM_FIELDS, EPOCH=epoch_string)
    reference_fields = dict(_OMM_FIELDS, EPOCH=normalized_epoch)
    path = _WRITERS[encoding](tmp_path / f"fraction.{encoding}", [fields])
    reference_path = _WRITERS[encoding](tmp_path / f"reference.{encoding}", [reference_fields])
    initialize = ommHandling.sgp4omm.initialize
    initialized_epochs = []

    def initialize_at_normalized_epoch(satellite, sgp4_fields):
        """Capture the epoch actually passed to the SGP4 initializer."""
        initialized_epochs.append(sgp4_fields["EPOCH"])
        return initialize(satellite, sgp4_fields)

    monkeypatch.setattr(ommHandling.sgp4omm, "initialize", initialize_at_normalized_epoch)
    records = ommHandling.satOmm2elem(str(path))
    reference = ommHandling.satOmm2elem(str(reference_path))[0]

    assert len(records) == 1
    assert records[0].ommEpoch == dt.datetime.fromisoformat(normalized_epoch)
    assert initialized_epochs == [normalized_epoch, normalized_epoch]
    for name in ("a", "e", "i", "Omega", "omega", "f"):
        assert getattr(records[0].oe, name) == getattr(reference.oe, name)
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("epoch_string", [
    "2026-09-10T12:00:30.1234567junk",
    "2026-253T12:00:30.123456789badZ",
    "2026-09-10T12:00:30.1234567.89",
    "2026-253T12:00:30.1234567+00:00",
    "2026-09-10T12:00:30.1234567 extra",
    "2026-253T12:00:30.123456\u0667",
])
def test_omm_malformed_epoch_fraction_is_skipped(tmp_path, monkeypatch, capsys, encoding, epoch_string):
    """Discarded fraction digits cannot hide invalid suffixes; later records survive."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, "EPOCH", epoch_string)


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("epoch_string", [
    "2026-366T12:00:30",
    "2026-366T12:00:30.123456Z",
    "2026-366T23:59:59.999999999Z",
    "1900-366T12:00:30",
    "2100-366T12:00:30.123456",
    "2026-000T12:00:30",
    "2024-367T12:00:30",
])
def test_omm_invalid_day_of_year_is_skipped(tmp_path, monkeypatch, capsys, encoding, epoch_string):
    """Out-of-year dates never reach SGP4; later valid records remain available."""
    warning = _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, "EPOCH", epoch_string)
    assert epoch_string in warning


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("epoch_string,calendar_epoch", [
    ("2026-001T00:00:00", "2026-01-01T00:00:00"),
    ("2026-365T23:59:59Z", "2026-12-31T23:59:59"),
    ("2024-366T23:59:59.999999999Z", "2024-12-31T23:59:59.999999"),
    ("2000-366T12:00:30.123456", "2000-12-31T12:00:30.123456"),
    ("2024-060T12:00:30", "2024-02-29T12:00:30"),
    ("2026-060T12:00:30", "2026-03-01T12:00:30"),
])
def test_omm_valid_day_of_year(tmp_path, capsys, encoding, epoch_string, calendar_epoch):
    """Gregorian leap years and valid year boundaries retain the exact calendar date."""
    fields = dict(_OMM_FIELDS, EPOCH=epoch_string)
    path = _WRITERS[encoding](tmp_path / f"day-of-year.{encoding}", [fields])

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    assert records[0].ommEpoch == dt.datetime.fromisoformat(calendar_epoch)
    assert capsys.readouterr().out == ""


def test_omm_bad_record_is_skipped_not_fatal(tmp_path):
    """A record missing a required element is skipped while good records still return."""
    broken = dict(_OMM_FIELDS)
    del broken["MEAN_MOTION"]
    broken["OBJECT_NAME"] = "BROKEN-SAT"
    path = _writeKvn(tmp_path / "mixed.kvn", [broken, _OMM_FIELDS])

    ommDataList = ommHandling.satOmm2elem(str(path))

    assert [d.satName for d in ommDataList] == ["ISS (ZARYA)"]


@pytest.mark.parametrize("invalid_entry", [
    None, 42, 1.5, True, "not-an-object", [], [["OBJECT_NAME", "BROKEN-SAT"]],
])
def test_omm_json_invalid_entries_preserve_record_numbers(tmp_path, capsys, invalid_entry):
    """Malformed JSON entries are skipped without losing records or shifting diagnostics."""
    incomplete = dict(_OMM_FIELDS)
    del incomplete["MEAN_MOTION"]
    second = dict(_OMM_FIELDS, OBJECT_NAME="SECOND-SAT", NORAD_CAT_ID="25545")
    path = _writeJson(tmp_path / "mixed.json", [invalid_entry, _OMM_FIELDS, incomplete, second])

    records = ommHandling.satOmm2elem(str(path))

    assert [record.satName for record in records] == ["ISS (ZARYA)", "SECOND-SAT"]
    warnings = capsys.readouterr().out.splitlines()
    assert len(warnings) == 2
    assert f"skipped OMM record 1 in {path}" in warnings[0]
    assert "object" in warnings[0]
    assert f"skipped OMM record 3 in {path}" in warnings[1]
    assert "MEAN_MOTION" in warnings[1]


def test_omm_json_all_invalid_entries_are_reported(tmp_path, capsys):
    """An array of malformed records returns an empty result with one warning per entry."""
    path = _writeJson(tmp_path / "invalid.json", [None, 42, []])

    assert ommHandling.satOmm2elem(str(path)) == []

    warnings = capsys.readouterr().out.splitlines()
    assert len(warnings) == 3
    for record_number, warning in enumerate(warnings, start=1):
        assert f"skipped OMM record {record_number} in {path}" in warning
        assert "object" in warning


@pytest.mark.parametrize("as_array", [False, True])
def test_omm_json_field_normalization(tmp_path, capsys, as_array):
    """Single objects and arrays still accept native numbers, lowercase keys, and nulls."""
    fields = {name.lower(): value for name, value in _OMM_FIELDS.items()}
    fields["norad_cat_id"] = 799500001
    fields["mean_motion"] = float(_OMM_FIELDS["MEAN_MOTION"])  # [rev/day]
    fields["inclination"] = float(_OMM_FIELDS["INCLINATION"])  # [deg]
    fields["element_set_no"] = 999
    fields["bstar"] = None
    path = tmp_path / "normalized.json"
    path.write_text(json.dumps([fields] if as_array else fields))

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    assert records[0].satName == _OMM_FIELDS["OBJECT_NAME"]
    assert records[0].noradID == "799500001"
    assert records[0].meanMotion == fields["mean_motion"]
    assert records[0].elemSetNo == fields["element_set_no"]
    assert records[0].bStar == 0.0  # [1/Earth radii]
    assert capsys.readouterr().out == ""


def test_omm_invalid_json_syntax_raises(tmp_path):
    """A syntax error affecting the JSON document remains a file-level failure."""
    path = tmp_path / "truncated.json"
    path.write_text("[" + json.dumps(_OMM_FIELDS) + ",")

    with pytest.raises(json.JSONDecodeError):
        ommHandling.satOmm2elem(str(path))


def test_omm_empty_json_array_raises(tmp_path):
    """An empty JSON array contains no records to process."""
    path = _writeJson(tmp_path / "empty.json", [])

    with pytest.raises(ValueError, match="found no OMM records"):
        ommHandling.satOmm2elem(str(path))


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("field_name", [
    "MEAN_MOTION", "ECCENTRICITY", "INCLINATION", "RA_OF_ASC_NODE",
    "ARG_OF_PERICENTER", "MEAN_ANOMALY", "BSTAR", "MEAN_MOTION_DOT",
    "MEAN_MOTION_DDOT", "EPHEMERIS_TYPE", "ELEMENT_SET_NO", "REV_AT_EPOCH",
])
@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "1e309", "not-a-number"])
def test_omm_invalid_numeric_record_is_skipped(
    tmp_path, monkeypatch, capsys, encoding, field_name, value
):
    """Invalid numbers are rejected before SGP4 while subsequent records survive."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, field_name, value)


@pytest.mark.parametrize("encoding", ["kvn", "json", "csv", "xml"])
@pytest.mark.parametrize("field_name,value", [
    ("MEAN_MOTION", "0"),  # [rev/day]
    ("MEAN_MOTION", "-1"),  # [rev/day]
    ("ECCENTRICITY", "-0.1"),  # [-]
    ("ECCENTRICITY", "1"),  # [-]
    ("INCLINATION", "-1"),  # [deg]
    ("INCLINATION", "180.1"),  # [deg]
    ("RA_OF_ASC_NODE", "-1"),  # [deg]
    ("RA_OF_ASC_NODE", "360.1"),  # [deg]
    ("ARG_OF_PERICENTER", "-1"),  # [deg]
    ("ARG_OF_PERICENTER", "360.1"),  # [deg]
    ("MEAN_ANOMALY", "-1"),  # [deg]
    ("MEAN_ANOMALY", "360.1"),  # [deg]
    ("EPHEMERIS_TYPE", "-1"),
    ("EPHEMERIS_TYPE", "1.5"),
    ("ELEMENT_SET_NO", "-1"),
    ("ELEMENT_SET_NO", "1.5"),
    ("REV_AT_EPOCH", "-1"),
    ("REV_AT_EPOCH", "1.5"),
])
def test_omm_out_of_range_record_is_skipped(
    tmp_path, monkeypatch, capsys, encoding, field_name, value
):
    """Invalid orbital ranges and integer metadata do not reach SGP4."""
    _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, field_name, value)


def _check_invalid_record(tmp_path, monkeypatch, capsys, encoding, field_name, value, units=None):
    """Check record recovery and verify that SGP4 only receives the valid record."""
    broken = dict(_OMM_FIELDS, OBJECT_NAME="BROKEN-SAT")
    broken[field_name] = value
    path = tmp_path / f"invalid-record.{encoding}"
    if units is None:
        _WRITERS[encoding](path, [broken, _OMM_FIELDS])
    else:
        _write_omm_with_units(path, encoding, [broken, _OMM_FIELDS], [{field_name: units}, {}])
    initialize = ommHandling.sgp4omm.initialize
    initialized_names = []

    def initialize_valid_record(satellite, fields):
        """Fail if invalid input reaches the external library."""
        assert fields["OBJECT_NAME"] == _OMM_FIELDS["OBJECT_NAME"]
        initialized_names.append(fields["OBJECT_NAME"])
        return initialize(satellite, fields)

    monkeypatch.setattr(ommHandling.sgp4omm, "initialize", initialize_valid_record)
    records = ommHandling.satOmm2elem(str(path))

    assert [record.satName for record in records] == [_OMM_FIELDS["OBJECT_NAME"]]
    assert initialized_names == [_OMM_FIELDS["OBJECT_NAME"]]
    warning = capsys.readouterr().out
    assert "skipped OMM record 1" in warning
    assert field_name in warning
    return warning


@pytest.mark.parametrize("field_name,value", [
    ("ECCENTRICITY", "0"),  # [-]
    ("INCLINATION", "0"),  # [deg]
    ("INCLINATION", "180"),  # [deg]
    ("RA_OF_ASC_NODE", "0"),  # [deg]
    ("RA_OF_ASC_NODE", "360"),  # [deg]
    ("ARG_OF_PERICENTER", "0"),  # [deg]
    ("ARG_OF_PERICENTER", "360"),  # [deg]
    ("MEAN_ANOMALY", "0"),  # [deg]
    ("MEAN_ANOMALY", "360"),  # [deg]
    ("BSTAR", "-0.00016717"),  # [1/Earth radii]
    ("MEAN_MOTION_DOT", "-0.00002182"),  # [rev/day^2]
    ("MEAN_MOTION_DDOT", "-0.00000001"),  # [rev/day^3]
    ("ELEMENT_SET_NO", "0"),
    ("REV_AT_EPOCH", "0"),
])
def test_omm_valid_numeric_boundaries(tmp_path, capsys, field_name, value):
    """Valid endpoints and signed drag terms remain accepted by the reader."""
    fields = dict(_OMM_FIELDS)
    fields[field_name] = value
    path = _writeKvn(tmp_path / "boundary.kvn", [fields])

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    assert all(np.isfinite(getattr(records[0].oe, name))
               for name in ("a", "e", "i", "Omega", "omega", "f"))
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("mean_motion", ["5e-324", "1e-300", "1e308"])  # [rev/day]
def test_omm_sgp4_numeric_failure_is_skipped(tmp_path, monkeypatch, capsys, mean_motion):
    """Finite inputs outside the backend's numerical capacity do not abort the file."""
    broken = dict(_OMM_FIELDS, OBJECT_NAME="BROKEN-SAT", MEAN_MOTION=mean_motion)
    path = _writeKvn(tmp_path / "extreme-motion.kvn", [broken, _OMM_FIELDS])
    monkeypatch.setattr(ommHandling, "Satrec", PythonSatrec)

    records = ommHandling.satOmm2elem(str(path))

    assert [record.satName for record in records] == [_OMM_FIELDS["OBJECT_NAME"]]
    warning = capsys.readouterr().out
    assert "skipped OMM record 1" in warning
    assert "SGP4" in warning
    assert "BROKEN-SAT" in warning


@pytest.mark.parametrize("field_name,attribute", [
    ("ELEMENT_SET_NO", "elemSetNo"), ("REV_AT_EPOCH", "revAtEpoch"),
])
def test_omm_integer_metadata_is_exact(tmp_path, monkeypatch, field_name, attribute):
    """Integer metadata must not be rounded through a floating-point conversion."""
    value = 9007199254740993
    fields = dict(_OMM_FIELDS)
    fields[field_name] = str(value)
    path = _writeKvn(tmp_path / "integer-metadata.kvn", [fields])
    # Exercise precision independently of platform-specific C integer widths.
    monkeypatch.setattr(ommHandling, "Satrec", PythonSatrec)

    records = ommHandling.satOmm2elem(str(path))

    assert len(records) == 1
    assert getattr(records[0], attribute) == value


@pytest.mark.parametrize("component", ["position", "velocity"])
@pytest.mark.parametrize("value", [
    float("nan"), float("inf"), -float("inf"),
    1e308,  # [km] for position, [km/s] for velocity; overflows in SI units
])
def test_omm_nonfinite_sgp4_state_is_skipped(tmp_path, monkeypatch, capsys, component, value):
    """Non-finite states or overflow during SI conversion must not reach orbitalMotion."""
    broken = dict(_OMM_FIELDS, OBJECT_NAME="BROKEN-SAT")
    path = _writeKvn(tmp_path / "invalid-state.kvn", [broken, _OMM_FIELDS])
    sgp4 = PythonSatrec.sgp4
    propagated_count = 0

    def propagate_with_invalid_first_state(satellite, jd, fraction):
        """Corrupt only the first state even though SGP4 reports success."""
        nonlocal propagated_count
        error, position, velocity = sgp4(satellite, jd, fraction)
        propagated_count += 1
        if propagated_count == 1:
            assert error == 0
            position, velocity = list(position), list(velocity)
            state_vector = position if component == "position" else velocity
            state_vector[0] = value  # [km] for position, [km/s] for velocity
        return error, position, velocity

    monkeypatch.setattr(ommHandling, "Satrec", PythonSatrec)
    monkeypatch.setattr(PythonSatrec, "sgp4", propagate_with_invalid_first_state)
    records = ommHandling.satOmm2elem(str(path))

    assert [record.satName for record in records] == [_OMM_FIELDS["OBJECT_NAME"]]
    warning = capsys.readouterr().out
    assert "skipped OMM record 1" in warning
    assert "non-finite" in warning
    assert "BROKEN-SAT" in warning


def test_omm_unknown_format_raises(tmp_path):
    """A file that is none of the four encodings is reported clearly."""
    path = tmp_path / "junk.txt"
    path.write_text("this is not an OMM file\n")

    with pytest.raises(ValueError, match="could not determine the OMM format"):
        ommHandling.satOmm2elem(str(path))


def test_omm_empty_file_raises(tmp_path):
    """An empty file is reported rather than silently returning nothing."""
    path = tmp_path / "empty.kvn"
    path.write_text("")

    with pytest.raises(ValueError, match="empty OMM file"):
        ommHandling.satOmm2elem(str(path))


def test_omm_matches_tle_for_same_elements(tmp_path):
    """
    The OMM path reproduces the TLE path.

    The same mean elements are fed through both readers; both run SGP4 at the epoch and the
    same TEME -> J2000 conversion, so the osculating elements must agree within absolute
    tolerances based on the resolution the TLE's fixed-width fields can express.
    """
    import Basilisk.utilities.tleHandling as tleHandling

    # ISS TLE whose fields match _OMM_FIELDS exactly.
    line1 = "1 25544U 98067A   26253.50000000  .00002182  00000-0  16717-3 0  9991"
    line2 = "2 25544  51.6416 247.4627 0004364 130.5360 325.0288 15.50103472478591"
    tlePath = tmp_path / "iss.2le"
    tlePath.write_text(f"{line1}\n{line2}\n")

    tleDataList = tleHandling.satTle2elem(str(tlePath))
    assert len(tleDataList) == 1

    ommPath = _writeKvn(tmp_path / "iss.kvn", [_OMM_FIELDS])
    ommDataList = ommHandling.satOmm2elem(str(ommPath))
    assert len(ommDataList) == 1

    tleOe = tleDataList[0].oe
    ommOe = ommDataList[0].oe

    assert np.isclose(ommOe.a, tleOe.a, rtol=0.0, atol=A_TOL_TLE_DIST)
    assert np.isclose(ommOe.e, tleOe.e, rtol=0.0, atol=1e-9)  # [-]
    for name in ("i", "Omega", "omega", "f"):
        assert np.isclose(
            getattr(ommOe, name), getattr(tleOe, name), rtol=0.0, atol=A_TOL_TLE_ANG
        ), name
