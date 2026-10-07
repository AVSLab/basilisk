#
#  ISC License
#
#  Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""Tests of the reference manifest of the accuracy comparison. They need neither GMAT, Orekit nor Basilisk."""

import copy
import sys
from pathlib import Path

import numpy as np
import pytest

ACCURACY_DIR = Path(__file__).resolve().parents[1] / "accuracyComparison"
sys.path.insert(0, str(ACCURACY_DIR))

import comparisonCommon as common  # noqa: E402

CASE = "leo_twobody"


def _external(spec, tool="orekit"):
    """Return external-data records with every identifier that a reference of ``tool`` must carry."""
    return {key: {"id": key, "sha256": "0" * 64} for key in common.REQUIRED_EXTERNAL_DATA[tool]} | {
        "gravity_coefficients": common.gravityCoefficientsRecord(spec)}


def _write(folder, spec, name=CASE, tool="orekit", version="Orekit test", variant=common.DEFAULT_VARIANT, options=None):
    """Write a reference ephemeris and its manifest entry; return the file path."""
    csv = folder / f"{tool}_{name}.csv"
    csv.write_text("t_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s\n0.0,1,2,3,4,5,6\n")
    common.writeManifestEntry(folder, tool, spec, name, csv, spec["inertial_frame"], version, options or {},
                              _external(spec, tool), variant)
    return csv


@pytest.fixture
def reference(tmp_path):
    """Write a reference ephemeris with a manifest entry for the current definition of the case."""
    spec = common.loadSpec()
    return spec, tmp_path, _write(tmp_path, spec)


def test_matching_reference_is_accepted(reference):
    """Verify a reference generated for the current case definition passes the validation."""
    spec, folder, csv = reference
    entry = common.validateManifestEntry(folder, "orekit", spec, CASE, csv)
    assert entry["variant"] == common.DEFAULT_VARIANT
    assert entry["frame"] == "ICRF"


def test_changed_gravity_coefficients_are_rejected(reference, tmp_path, monkeypatch):
    """Verify that changing a coefficient without changing the file name or cases.json invalidates the reference."""
    spec, folder, csv = reference
    source = common.HERE / spec["gravity_coefficients_file"]
    copy_dir = tmp_path / "current_input"
    (copy_dir / source.parent.relative_to(common.HERE)).mkdir(parents=True)
    lines = source.read_text().splitlines()
    n, m, c20, s20 = lines[1].split(",")  # first data row: n = 2, m = 0
    assert (n, m) == ("2", "0")
    c20 = repr(float(c20) * 1.01)  # [-] 1 % change of the normalized C20
    lines[1] = ",".join((n, m, c20, s20))
    (copy_dir / spec["gravity_coefficients_file"]).write_text("\n".join(lines) + "\n")
    monkeypatch.setattr(common, "HERE", copy_dir)  # same relative path, different content
    with pytest.raises(ValueError, match="gravity_coefficients"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)


def test_missing_gravity_checksum_is_rejected(tmp_path):
    """Verify that a manifest entry without the gravity coefficient checksum is not accepted."""
    spec = common.loadSpec()
    csv = tmp_path / f"orekit_{CASE}.csv"
    csv.write_text("t_s,x_m\n0.0,1\n")
    common.writeManifestEntry(tmp_path, "orekit", spec, CASE, csv, spec["inertial_frame"], "Orekit test", {}, {})
    with pytest.raises(ValueError, match="gravity_coefficients"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)


def test_changed_epoch_is_rejected(reference):
    """Verify that changing the epoch in cases.json is detected although the elapsed sample times are unchanged."""
    spec, folder, csv = reference
    changed = copy.deepcopy(spec)
    changed["cases"][CASE]["epoch_utc"] = "2026-06-01T00:00:00.000"
    with pytest.raises(ValueError, match="config_hash"):
        common.validateManifestEntry(folder, "orekit", changed, CASE, csv)


def test_changed_global_epoch_is_rejected(reference):
    """Verify that a change of the global epoch invalidates the cases that inherit it."""
    spec, folder, csv = reference
    changed = copy.deepcopy(spec)
    changed["epoch_utc"] = "2026-06-01T00:00:00.000"
    with pytest.raises(ValueError, match="config_hash"):
        common.validateManifestEntry(folder, "orekit", changed, CASE, csv)


def test_basilisk_time_step_does_not_invalidate_the_reference(reference):
    """Verify that changing the Basilisk task period, which the references do not depend on, is accepted."""
    spec, folder, csv = reference
    changed = copy.deepcopy(spec)
    changed["cases"][CASE]["basilisk_step_s"] = 0.5
    changed["cases"][CASE]["references"] = ["orekit"]  # the list of tools does not change a reference either
    assert common.validateManifestEntry(folder, "orekit", changed, CASE, csv)["variant"] == common.DEFAULT_VARIANT


def test_alternative_variant_is_rejected_unless_requested(tmp_path):
    """Verify a reference generated with an alternative configuration is not accepted as the default one."""
    spec = common.loadSpec()
    csv = _write(tmp_path, spec, variant="oblate_shadow", options={"oblate_shadow": True})
    with pytest.raises(ValueError, match="variant"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)
    entry = common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv, "oblate_shadow")
    assert entry["variant"] == "oblate_shadow"


def test_modified_csv_is_rejected(reference):
    """Verify a reference file that was changed after generation does not match its recorded checksum."""
    spec, folder, csv = reference
    csv.write_text(csv.read_text() + "3600.0,1,2,3,4,5,6\n")
    with pytest.raises(ValueError, match="csv_sha256"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)


def test_wrong_frame_is_rejected(tmp_path):
    """Verify a reference expressed in another inertial frame is rejected."""
    spec = common.loadSpec()
    csv = tmp_path / f"orekit_{CASE}.csv"
    csv.write_text("t_s,x_m\n0.0,1\n")
    common.writeManifestEntry(tmp_path, "orekit", spec, CASE, csv, "EME2000", "Orekit test", {}, _external(spec))
    with pytest.raises(ValueError, match="frame"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)


def test_missing_manifest_is_rejected(tmp_path):
    """Verify a reference without a manifest is rejected."""
    spec = common.loadSpec()
    csv = tmp_path / f"orekit_{CASE}.csv"
    csv.write_text("t_s,x_m\n0.0,1\n")
    with pytest.raises(ValueError, match="manifest"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)


def _edit(folder, edit):
    """Apply ``edit`` to the manifest of the Orekit references in ``folder``."""
    path = common.manifestPath(folder, "orekit")
    manifest = common.json.loads(path.read_text())
    edit(manifest)
    path.write_text(common.json.dumps(manifest))


def test_missing_external_data_identifiers_are_rejected(tmp_path):
    """Verify a reference that records only the gravity file, not the tool data or build, is not accepted."""
    spec = common.loadSpec()
    csv = _write(tmp_path, spec)
    _edit(tmp_path, lambda m: m["cases"][CASE]["external_data"].pop("orekit_jar"))
    with pytest.raises(ValueError, match="orekit_jar"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)


def test_unknown_tool_version_is_rejected(tmp_path):
    """Verify a reference without an identifiable tool build is rejected."""
    spec = common.loadSpec()
    csv = _write(tmp_path, spec, version="Orekit unknown, orekit_jpype 13")
    with pytest.raises(ValueError, match="tool_version"):
        common.validateManifestEntry(tmp_path, "orekit", spec, CASE, csv)


def test_changed_generator_code_is_rejected(reference):
    """Verify that a reference generated by different generator code (e.g. another constant) is rejected."""
    spec, folder, csv = reference
    _edit(folder, lambda m: m["cases"][CASE].update(generator_sha256="0" * 64))
    with pytest.raises(ValueError, match="generator_sha256"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)


def test_generator_fingerprint_ignores_comments_and_docstrings():
    """Verify documentation edits do not change the generator fingerprint but a changed constant does."""
    source = 'def f():\n    """Doc."""\n    x = 1.0  # [s] comment\n    return x\n'
    edited = 'def f():\n    """Other doc."""\n\n    x = 1.0  # [m] other\n    return x\n'
    constant = 'def f():\n    """Doc."""\n    x = 2.0  # [s] comment\n    return x\n'
    lines = lambda text: common._executableLines(text, common.ast.parse(text))  # noqa: E731
    assert lines(source) == lines(edited)
    assert lines(source) != lines(constant)


def test_wrong_manifest_version_or_tool_is_rejected(reference):
    """Verify a manifest of another format version, or an entry that names another tool, is rejected."""
    spec, folder, csv = reference
    _edit(folder, lambda m: m["cases"][CASE].update(tool="gmat"))
    with pytest.raises(ValueError, match="tool"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)
    _edit(folder, lambda m: m.update(version=99))
    with pytest.raises(ValueError, match="version"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)


def test_hand_edited_config_is_rejected(reference):
    """Verify an entry whose recorded configuration no longer matches its hash is rejected."""
    spec, folder, csv = reference
    _edit(folder, lambda m: m["cases"][CASE]["config"].update(epoch_utc="2030-01-01T00:00:00.000"))
    with pytest.raises(ValueError, match="edited by hand"):
        common.validateManifestEntry(folder, "orekit", spec, CASE, csv)


def test_mixed_tool_builds_are_rejected(tmp_path):
    """Verify references of different tool builds in one manifest are not compared together."""
    spec = common.loadSpec()
    _write(tmp_path, spec, "leo_twobody", version="Orekit A")
    _write(tmp_path, spec, "leo_zonal6", version="Orekit B")
    manifest = common.json.loads(common.manifestPath(tmp_path, "orekit").read_text())["cases"]
    with pytest.raises(ValueError, match="tool_version"):
        common.checkConsistentProvenance(tmp_path, "orekit", manifest)


def test_mixed_external_data_is_rejected(tmp_path):
    """Verify references generated with different external data sets are not compared together."""
    spec = common.loadSpec()
    _write(tmp_path, spec, "leo_twobody")
    _write(tmp_path, spec, "leo_zonal6")
    _edit(tmp_path, lambda m: m["cases"]["leo_zonal6"]["external_data"]["orekit_data"].update(sha256="1" * 64))
    manifest = common.json.loads(common.manifestPath(tmp_path, "orekit").read_text())["cases"]
    with pytest.raises(ValueError, match="external_data"):
        common.checkConsistentProvenance(tmp_path, "orekit", manifest)


def test_per_case_default_max_step_is_accepted(tmp_path):
    """Verify cases whose default maximum step differs per ``cases.json`` are accepted, an overridden one is not."""
    spec = common.loadSpec()
    _write(tmp_path, spec, "leo_twobody")
    _write(tmp_path, spec, "leo_zonal6")
    _edit(tmp_path, lambda m: m["cases"]["leo_zonal6"]["generator_options"].update(max_step_s=2.0))
    manifest = common.json.loads(common.manifestPath(tmp_path, "orekit").read_text())["cases"]
    common.checkConsistentProvenance(tmp_path, "orekit", manifest)
    for entry in manifest.values():
        entry["generator_options"]["max_step_overridden"] = True
    with pytest.raises(ValueError, match="generator_options"):
        common.checkConsistentProvenance(tmp_path, "orekit", manifest)


def test_icrf_is_required_by_the_generators():
    """Verify a generator refuses a frame it does not implement instead of labeling its output with it."""
    spec = common.loadSpec()
    common.requireIcrf(spec, "orekit")
    with pytest.raises(ValueError, match="ICRF"):
        common.requireIcrf({**spec, "inertial_frame": "EME2000"}, "gmat")


def test_probe_tools_follow_the_selected_drag_cases():
    """Verify only the references of the selected drag cases need a density probe."""
    spec = common.loadSpec()
    boxCases = [n for n, c in spec["cases"].items() if c["drag"] and common.caseReferences(c) == ("orekit",)]
    assert boxCases, "cases.json has no Orekit-only drag case"
    assert common.requiredProbeTools(spec, boxCases) == ("orekit",)
    assert common.requiredProbeTools(spec, ["leo_twobody"]) == ()
    assert common.requiredProbeTools(spec, boxCases + ["leo_drag"]) == ("gmat", "orekit")


def test_orekit_only_selection_needs_no_gmat_data(tmp_path):
    """Verify the probe comparison of an Orekit-only drag selection runs with no GMAT probe or manifest present."""
    pytest.importorskip("Basilisk")
    import compare_with_basilisk as compare

    spec = common.loadSpec()
    common.writeProbe(tmp_path, "orekit", spec, _analyticProbe(spec), "Orekit test", {})
    names = [n for n, c in spec["cases"].items() if c["drag"] and common.caseReferences(c) == ("orekit",)]
    compare.compareDensityProbe(spec, tmp_path, common.requiredProbeTools(spec, names))
    assert not common.probePath(tmp_path, "gmat").exists()


def _analyticProbe(spec, altitudeOffset=0.0, densityScale=1.0):
    """Return probe rows (x, y, z, altitude, density) of an exponential atmosphere evaluated at the nominal altitude.

    Args:
        spec (dict): parsed ``cases.json`` content.
        altitudeOffset (float): [m] error added to the reported altitude, emulating a different altitude definition.
        densityScale (float): [-] factor applied to the reported density, emulating a different model.
    """
    atm = spec["exponential_atmosphere"]
    rows = []
    for x, y, z, altitude in common.probePoints(spec):
        density = atm["density_at_zero_altitude_kg_m3"] * np.exp(-altitude / atm["scale_height_m"])  # [kg/m^3]
        rows.append((x, y, z, altitude + altitudeOffset, density * densityScale))
    return rows


@pytest.fixture
def probe(tmp_path):
    """Write the density probe of a tool whose atmosphere equals the shared model."""
    spec = common.loadSpec()
    common.writeProbe(tmp_path, "orekit", spec, _analyticProbe(spec), "Orekit test", {})
    return spec, tmp_path


def test_matching_probe_is_accepted(probe):
    """Verify a probe generated for the current atmosphere loads and agrees with the shared model."""
    spec, folder = probe
    rows = common.loadProbe(folder, "orekit", spec)
    results = common.checkProbe(spec, "orekit", rows, rows[:, 4])
    assert len(results) == len(spec["density_probe"]["points_lat_lon_alt"])
    assert max(abs(r[1]) for r in results) < 1e-6


def test_changed_atmosphere_invalidates_the_probe(probe):
    """Verify that changing the scale height in cases.json is detected by the probe manifest."""
    spec, folder = probe
    changed = copy.deepcopy(spec)
    changed["exponential_atmosphere"]["scale_height_m"] *= 1.1
    with pytest.raises(ValueError, match="probe_hash"):
        common.loadProbe(folder, "orekit", changed)


def test_modified_probe_file_is_rejected(probe):
    """Verify that editing a probe file after it was generated is detected."""
    spec, folder = probe
    path = common.probePath(folder, "orekit")
    path.write_text(path.read_text().replace("e+", "e+0", 1))
    with pytest.raises(ValueError, match="csv_sha256"):
        common.loadProbe(folder, "orekit", spec)


def test_probe_does_not_invalidate_the_case_references(reference):
    """Verify that the probe definition is not part of the case fingerprint, so editing it keeps the ephemerides valid."""
    spec, folder, csv = reference
    changed = copy.deepcopy(spec)
    changed["density_probe"]["density_rel_tolerance"] = 1e-3
    common.validateManifestEntry(folder, "orekit", changed, CASE, csv)


def test_different_altitude_definition_is_detected():
    """Verify a tool that reports another altitude (for example a spherical one) is rejected before the density."""
    spec = common.loadSpec()
    rows = np.array(_analyticProbe(spec, altitudeOffset=5.0e3))
    with pytest.raises(ValueError, match="altitude definition"):
        common.checkProbe(spec, "gmat", rows, rows[:, 4])


def test_different_density_model_is_detected():
    """Verify a density that differs from Basilisk by more than the tolerance is rejected."""
    spec = common.loadSpec()
    rows = np.array(_analyticProbe(spec, densityScale=1.0 + 1.0e-3))
    bskDensity = np.array(_analyticProbe(spec))[:, 4]
    with pytest.raises(ValueError, match="not equivalent"):
        common.checkProbe(spec, "gmat", rows, bskDensity)


def test_probe_points_have_the_nominal_geodetic_altitude():
    """Verify the Earth-fixed probe positions are above the ellipsoid by the nominal altitude, also at the poles.

    The altitude is recovered with the fixed-point iteration of the inverse geodetic problem, independent of the
    forward conversion that generated the positions."""
    spec = common.loadSpec()
    a = spec["equatorial_radius_m"]  # [m]
    e2 = spec["earth_flattening"] * (2.0 - spec["earth_flattening"])  # [-]
    for x, y, z, altitude in common.probePoints(spec):
        p = np.hypot(x, y)  # [m]
        lat = np.arctan2(z, p * (1.0 - e2))  # [rad]
        for _ in range(20):
            n = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)  # [m]
            h = p * np.cos(lat) + z * np.sin(lat) - a * a / n  # [m] altitude, stable at the poles
            lat = np.arctan2(z, p * (1.0 - e2 * n / (n + h)))  # [rad]
        assert h == pytest.approx(altitude, abs=1e-3)
