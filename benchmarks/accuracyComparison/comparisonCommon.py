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

r"""
Shared helpers for the Basilisk vs GMAT vs Orekit accuracy comparison.

The case definitions live in ``cases.json`` and the Earth gravity coefficients in
``data/egm96_normalized_coefficients.csv``. This module loads them and writes the
gravity-field files in the formats read by GMAT (``.cof``), Orekit (ICGEM ``.gfc``)
and Basilisk (text), so that all three tools use identical coefficients.
"""

import csv
import datetime
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TOOLS = ("gmat", "orekit")


def loadSpec():
    """Return the parsed ``cases.json``."""
    return json.loads((HERE / "cases.json").read_text())


def caseEpoch(spec, case):
    """Return the UTC epoch of a case, ``"YYYY-MM-DDTHH:MM:SS.sss"``; cases may override the global epoch.

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
    """
    return case.get("epoch_utc", spec["epoch_utc"])


def caseDuration(spec, case):
    """Return the propagation time of a case in seconds [s]; cases may override the global duration.

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
    """
    return case.get("duration_s", spec["duration_s"])


def caseReferences(case):
    """Return the tools that provide a reference ephemeris for a case, ``("gmat", "orekit")`` by default.

    Args:
        case (dict): the case entry.
    """
    return tuple(case.get("references", TOOLS))


def mrp2dcm(sigma):
    """Return the direction cosine matrix [BN] of the modified Rodrigues parameters ``sigma_BN``.

    Args:
        sigma (list): MRP set of the body frame B relative to the inertial frame N.
    """
    s = np.asarray(sigma, dtype=float)
    s2 = s @ s
    tilde = np.array([[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]])
    return np.eye(3) + (8.0 * tilde @ tilde - 4.0 * (1.0 - s2) * tilde) / (1.0 + s2) ** 2


def boxFacets(spec):
    """Return the six facets of the box spacecraft as ``(area [m^2], unit normal in the body frame)`` tuples.

    Args:
        spec (dict): parsed ``cases.json`` content.
    """
    lx, ly, lz = spec["box"]["size_m"]  # [m]
    areas = {0: ly * lz, 1: lx * lz, 2: lx * ly}  # [m^2] by the axis of the facet normal
    facets = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            facets.append((areas[axis], normal))
    return facets


MANIFEST_VERSION = 1  # [-] version of the reference manifest format
DEFAULT_VARIANT = "default"
NON_REFERENCE_KEYS = ("basilisk_step_s", "references")  # case keys that do not affect the reference ephemerides


def effectiveCase(spec, name):
    """Return the effective configuration of a case: the global settings merged with the case entry.

    The global ``description`` is left out because it is prose, the ``density_probe`` because it has its own manifest
    (:func:`probeFingerprint`), and so are the case keys that do not change a reference
    (:data:`NON_REFERENCE_KEYS`: the Basilisk time step and the list of tools), so that changing them does not invalidate
    the references. The epoch and the duration are resolved, so that a
    change of the global value is seen by every case that uses it.

    Args:
        spec (dict): parsed ``cases.json`` content.
        name (str): case name.
    """
    # the Basilisk task period and the list of tools with a reference do not influence the reference ephemerides
    case = {k: v for k, v in spec["cases"][name].items() if k not in NON_REFERENCE_KEYS}
    settings = {k: v for k, v in spec.items() if k not in ("cases", "description", "density_probe")}
    return {"settings": settings, "case": case, "epoch_utc": caseEpoch(spec, spec["cases"][name]),
            "duration_s": caseDuration(spec, spec["cases"][name])}


def caseFingerprint(spec, name):
    """Return the SHA-256 hash of the canonical JSON of the effective configuration of a case.

    Args:
        spec (dict): parsed ``cases.json`` content.
        name (str): case name.
    """
    canonical = json.dumps(effectiveCase(spec, name), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


def fileSha256(path):
    """Return the SHA-256 hash of a file, or of the files of a folder in sorted order.

    Args:
        path (Path): file or folder.
    """
    digest = hashlib.sha256()
    path = Path(path)
    files = sorted(p for p in path.rglob("*") if p.is_file()) if path.is_dir() else [path]
    for item in files:
        if path.is_dir():
            digest.update(str(item.relative_to(path)).encode())
        with open(item, "rb") as f:
            for block in iter(lambda: f.read(1 << 20), b""):
                digest.update(block)
    return digest.hexdigest()


def manifestPath(dataDir, tool):
    """Return the path of the reference manifest of a tool.

    Args:
        dataDir (Path): folder holding the reference ephemerides.
        tool (str): ``"gmat"`` or ``"orekit"``.
    """
    return Path(dataDir) / f"{tool}_manifest.json"


def writeManifestEntry(dataDir, tool, spec, name, csvPath, frame, toolVersion, generatorOptions, externalData,
                       variant=DEFAULT_VARIANT):
    """Record the provenance of a reference ephemeris in the manifest of its tool, merging with the other cases.

    Args:
        dataDir (Path): folder holding the reference ephemerides.
        tool (str): ``"gmat"`` or ``"orekit"``.
        spec (dict): parsed ``cases.json`` content.
        name (str): case name.
        csvPath (Path): the reference ephemeris written for the case.
        frame (str): inertial frame of the ephemeris.
        toolVersion (str): version of the tool that generated the ephemeris.
        generatorOptions (dict): generator options and integrator settings that affect the physics.
        externalData (dict): identifier and SHA-256 hash of each external data set used, keyed by name.
        variant (str): ``"default"`` or the name of an intentional alternative configuration.
    """
    path = manifestPath(dataDir, tool)
    manifest = json.loads(path.read_text()) if path.exists() else {"version": MANIFEST_VERSION, "cases": {}}
    manifest["cases"][name] = {
        "config_hash": caseFingerprint(spec, name),
        "config": effectiveCase(spec, name),
        "frame": frame,
        "tool": tool,
        "tool_version": toolVersion,
        "generator_options": generatorOptions,
        "external_data": externalData,
        "variant": variant,
        "csv_sha256": fileSha256(csvPath),
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
    }
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def validateManifestEntry(dataDir, tool, spec, name, csvPath, allowedVariant=DEFAULT_VARIANT):
    """Check that a reference ephemeris was generated for the current definition of a case.

    The manifest of the tool must hold an entry of the case whose configuration hash, inertial frame and ephemeris
    checksum match the current ``cases.json`` and the file on disk. A reference generated with an alternative
    configuration (e.g. ``--oblate-shadow``) is rejected unless that variant is explicitly allowed.

    Args:
        dataDir (Path): folder holding the reference ephemerides.
        tool (str): ``"gmat"`` or ``"orekit"``.
        spec (dict): parsed ``cases.json`` content.
        name (str): case name.
        csvPath (Path): the reference ephemeris of the case.
        allowedVariant (str): variant of the reference that is accepted.

    Returns:
        dict: the manifest entry of the case.

    Raises:
        ValueError: naming the manifest field that does not match.
    """
    path = manifestPath(dataDir, tool)
    if not path.exists():
        raise ValueError(f"No manifest {path}: regenerate the {tool} references with the generator script.")
    entry = json.loads(path.read_text()).get("cases", {}).get(name)
    where = f"{tool} reference of case {name}"
    if entry is None:
        raise ValueError(f"{where} has no entry in {path}: regenerate it.")
    if entry["config_hash"] != caseFingerprint(spec, name):
        raise ValueError(f"{where} was generated for a different case configuration (config_hash, e.g. epoch, "
                         "initial state or force models changed in cases.json): regenerate it.")
    if entry["frame"] != spec["inertial_frame"]:
        raise ValueError(f"{where} is expressed in {entry['frame']} (frame), but the comparison uses "
                         f"{spec['inertial_frame']}: regenerate it.")
    if entry["variant"] != allowedVariant:
        raise ValueError(f"{where} was generated as variant '{entry['variant']}' (variant), but the comparison "
                         f"expects variant '{allowedVariant}'. Alternative configurations such as --oblate-shadow are "
                         f"only accepted knowingly: pass --{tool}-variant {entry['variant']} to compare against it, "
                         "or regenerate the default reference.")
    if entry["csv_sha256"] != fileSha256(csvPath):
        raise ValueError(f"{csvPath} does not match the checksum recorded in the manifest (csv_sha256): the file was "
                         "modified or overwritten after it was generated.")
    return entry


def loadCoefficients(spec):
    """Return the fully normalized Earth coefficients as ``{(n, m): (C, S)}``.

    Args:
        spec (dict): parsed ``cases.json`` content.
    """
    table = {}
    with open(HERE / spec["gravity_coefficients_file"], newline="") as f:
        for row in csv.DictReader(f):
            table[(int(row["n"]), int(row["m"]))] = (float(row["C_norm"]), float(row["S_norm"]))
    return table


def writeGmatCof(path, spec, degree, order):
    """Write a GMAT ``.cof`` gravity file.

    Args:
        path (Path): output file path.
        spec (dict): parsed ``cases.json`` content.
        degree (int): maximum degree to write.
        order (int): maximum order to write.
    """
    table = loadCoefficients(spec)
    lines = [
        "COMMENT   Earth field for the Basilisk accuracy comparison",
        f"POTFIELD{degree:3d}{degree:3d}{1:3d}"
        f"{spec['mu_m3_s2']:21.14E}{spec['equatorial_radius_m']:21.14E}{1.0:21.14E}",
    ]
    for n in range(2, degree + 1):
        for m in range(n + 1):
            c, s = table[(n, m)] if m <= order else (0.0, 0.0)
            line = f"RECOEF{n:5d}{m:3d}{c:24.14E}"
            lines.append(line + f"{s: .14E}" if m else line)
    path.write_text("\n".join(lines) + "\n")


def writeGmatExponentialAtmosphere(path, spec):
    """Write the GMAT exponential atmosphere table of the single-scale density model shared by all tools.

    GMAT's table has one band per row: bottom altitude [km], density at that altitude [kg/m^3] and scale height [km].
    A single band from 0 km is the model :math:`\\rho = \\rho_0 \\exp(-h/H)` given by the density at zero altitude and
    the scale height of ``cases.json``, which every tool receives without conversion.

    Args:
        path (Path): output file path.
        spec (dict): parsed ``cases.json`` content.
    """
    atm = spec["exponential_atmosphere"]
    path.write_text(f"0.0, {float(atm['density_at_zero_altitude_kg_m3'])!r}, {atm['scale_height_m'] / 1000.0!r}\n")


def writeOrekitGfc(path, spec, degree, order):
    """Write an ICGEM ``.gfc`` gravity file for Orekit.

    Args:
        path (Path): output file path.
        spec (dict): parsed ``cases.json`` content.
        degree (int): maximum degree to write.
        order (int): maximum order to write.
    """
    table = loadCoefficients(spec)
    lines = [
        "product_type            gravity_field",
        "modelname               COMPARISON",
        f"earth_gravity_constant  {spec['mu_m3_s2']:.14E}",
        f"radius                  {spec['equatorial_radius_m']:.14E}",
        f"max_degree              {degree}",
        "errors                  no",
        "norm                    fully_normalized",
        "tide_system             unknown",
        "end_of_head",
    ]
    for n in range(2, degree + 1):
        for m in range(min(n, order) + 1):
            c, s = table[(n, m)]
            lines.append(f"gfc {n:3d} {m:3d} {c:.14E} {s:.14E}")
    path.write_text("\n".join(lines) + "\n")


def writeBasiliskGravity(path, spec, degree, order):
    """Write the gravity file in the Basilisk spherical-harmonics text format.

    Args:
        path (Path): output file path.
        spec (dict): parsed ``cases.json`` content.
        degree (int): maximum degree to write.
        order (int): maximum order to write.
    """
    table = loadCoefficients(spec)
    from Basilisk.utilities import orbitalMotion  # only the Basilisk comparison writes this file
    omegaEarth = orbitalMotion.OMEGA_EARTH  # [rad/s] required by the file format, unused here
    lines = [f"{spec['equatorial_radius_m']:.10E}, {spec['mu_m3_s2']:.10E}, {omegaEarth:.7E}, "
             f"{degree}, {degree}, 1, 0.0, 0.0"]
    for n in range(degree + 1):
        for m in range(n + 1):
            if n == 0:
                c, s = 1.0, 0.0
            elif n >= 2 and m <= order:
                c, s = table[(n, m)]
            else:
                c, s = 0.0, 0.0
            lines.append(f"{n}, {m}, {c:.12E}, {s:.12E}, 0.0, 0.0")
    path.write_text("\n".join(lines) + "\n")


PROBE_COLUMNS = ["x_m", "y_m", "z_m", "altitude_m", "density_kg_m3"]  # columns of the density probe files


def geodeticToEcef(spec, latDeg, lonDeg, altitude):
    """Return the Earth-fixed position [m] of a geodetic point above the ellipsoid of ``cases.json``.

    Args:
        spec (dict): parsed ``cases.json`` content.
        latDeg (float): [deg] geodetic latitude.
        lonDeg (float): [deg] longitude.
        altitude (float): [m] altitude above the ellipsoid.
    """
    a = spec["equatorial_radius_m"]  # [m]
    f = spec["earth_flattening"]  # [-]
    e2 = f * (2.0 - f)  # [-]
    lat, lon = np.radians(latDeg), np.radians(lonDeg)  # [rad]
    n = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)  # [m]
    return np.array([(n + altitude) * np.cos(lat) * np.cos(lon),
                     (n + altitude) * np.cos(lat) * np.sin(lon),
                     (n * (1.0 - e2) + altitude) * np.sin(lat)])


def probePoints(spec):
    """Return the density probe points as an array of rows ``(x, y, z [m], nominal altitude [m])``, Earth-fixed.

    Args:
        spec (dict): parsed ``cases.json`` content.
    """
    return np.array([[*geodeticToEcef(spec, lat, lon, alt), alt]
                     for lat, lon, alt in spec["density_probe"]["points_lat_lon_alt"]])


def probeFingerprint(spec):
    """Return the SHA-256 hash of everything that defines the density probe of a tool.

    Args:
        spec (dict): parsed ``cases.json`` content.
    """
    canonical = json.dumps({"probe": spec["density_probe"]["points_lat_lon_alt"],
                            "atmosphere": spec["exponential_atmosphere"],
                            "equatorial_radius_m": spec["equatorial_radius_m"],
                            "earth_flattening": spec["earth_flattening"]},
                           sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


def probePath(dataDir, tool):
    """Return the path of the density probe file of a tool.

    Args:
        dataDir (Path): folder holding the reference files.
        tool (str): ``"gmat"`` or ``"orekit"``.
    """
    return Path(dataDir) / f"{tool}_density_probe.csv"


def writeProbe(dataDir, tool, spec, rows, toolVersion, generatorOptions):
    """Write the density probe of a tool and record its provenance in the manifest of the tool.

    Args:
        dataDir (Path): folder holding the reference files.
        tool (str): ``"gmat"`` or ``"orekit"``.
        spec (dict): parsed ``cases.json`` content.
        rows (list): one ``(x, y, z [m], altitude [m], density [kg/m^3])`` row per probe point, in the order of
            ``points_lat_lon_alt``, with the position and the altitude as reported by the tool.
        toolVersion (str): version of the tool.
        generatorOptions (dict): generator options that affect the evaluation.
    """
    path = probePath(dataDir, tool)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(PROBE_COLUMNS)
        for row in rows:
            writer.writerow([f"{v:.12e}" for v in row])
    manifestFile = manifestPath(dataDir, tool)
    manifest = json.loads(manifestFile.read_text()) if manifestFile.exists() else {"version": MANIFEST_VERSION,
                                                                                    "cases": {}}
    manifest["density_probe"] = {"probe_hash": probeFingerprint(spec), "tool": tool, "tool_version": toolVersion,
                                 "generator_options": generatorOptions, "csv_sha256": fileSha256(path),
                                 "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
                                     timespec="seconds")}
    manifestFile.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def loadProbe(dataDir, tool, spec):
    """Return the density probe of a tool as an array of rows ``PROBE_COLUMNS``, after validating its manifest.

    Args:
        dataDir (Path): folder holding the reference files.
        tool (str): ``"gmat"`` or ``"orekit"``.
        spec (dict): parsed ``cases.json`` content.

    Raises:
        ValueError: naming the manifest field that does not match, or if the file has the wrong number of points.
    """
    path = probePath(dataDir, tool)
    manifestFile = manifestPath(dataDir, tool)
    where = f"{tool} density probe {path}"
    if not path.exists():
        raise ValueError(f"The {where} does not exist: regenerate the {tool} references with the generator script.")
    entry = json.loads(manifestFile.read_text()).get("density_probe") if manifestFile.exists() else None
    if entry is None:
        raise ValueError(f"The {where} has no density_probe entry in {manifestFile}: regenerate it.")
    if entry["probe_hash"] != probeFingerprint(spec):
        raise ValueError(f"The {where} was generated for a different probe or atmosphere (probe_hash): regenerate it.")
    if entry["csv_sha256"] != fileSha256(path):
        raise ValueError(f"The {where} does not match the checksum recorded in the manifest (csv_sha256).")
    rows = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if rows.shape != (len(spec["density_probe"]["points_lat_lon_alt"]), len(PROBE_COLUMNS)):
        raise ValueError(f"The {where} has shape {rows.shape}, which does not match the probe points of cases.json.")
    return rows


def checkProbe(spec, tool, rows, bskDensity):
    """Compare the density and altitude of a tool at the probe points with Basilisk and the nominal altitude.

    The tool reports the position it evaluated and the altitude and density it computed there. Basilisk evaluates its
    atmosphere at the same positions. The altitude of the tool must agree with the geodetic altitude of the point
    (altitude definition) and the densities must agree (model).

    Args:
        spec (dict): parsed ``cases.json`` content.
        tool (str): ``"gmat"`` or ``"orekit"``.
        rows (ndarray): the probe of the tool, columns ``PROBE_COLUMNS``.
        bskDensity (ndarray): [kg/m^3] density of Basilisk at the positions of ``rows``.

    Returns:
        list: one ``(nominal altitude [m], altitude error [m], relative density difference [-])`` tuple per point.

    Raises:
        ValueError: naming the tool and the first point that exceeds a tolerance.
    """
    probe = spec["density_probe"]
    nominal = probePoints(spec)[:, 3]  # [m]
    results = []
    for k, (row, h0, rhoBsk) in enumerate(zip(rows, nominal, bskDensity)):
        altitudeError = row[3] - h0  # [m]
        densityError = (rhoBsk - row[4]) / row[4]  # [-]
        results.append((h0, altitudeError, densityError))
        point = probe["points_lat_lon_alt"][k]
        if abs(altitudeError) > probe["altitude_tolerance_m"]:
            raise ValueError(f"{tool} reports an altitude {altitudeError:+.3g} m different from the geodetic altitude "
                             f"at probe point {k} {point}: the altitude definition differs.")
        if abs(densityError) > probe["density_rel_tolerance"]:
            raise ValueError(f"Basilisk and {tool} differ in density by {densityError:+.3e} (relative) at probe point "
                             f"{k} {point}: the atmosphere models are not equivalent.")
    return results
