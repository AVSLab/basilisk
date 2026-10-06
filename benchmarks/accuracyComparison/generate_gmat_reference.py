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
Generate the GMAT reference ephemerides used by the accuracy comparison.

This is a maintainer tool and is not run by CI. It reads ``cases.json``, writes one
GMAT script (and gravity file) per case, runs ``GmatConsole``, and stores the propagated
state in ``<output-dir>/gmat_<case>.csv`` (SI units, default ``data/``).

Usage::

    python generate_gmat_reference.py /path/to/GMAT/R2026a [case ...] [--output-dir DIR]

Every case is propagated and reported in the ICRF axes of the DE430 SPICE kernels that Basilisk uses. GMAT's
``MJ2000Eq`` axes differ from them by the frame bias, a rotation of about 20 mas.

Cases with Earth rotation use GMAT's built-in Earth with its default orientation model and its ``EarthICRF`` coordinate
system. Cases without Earth rotation use a user-defined planet with a constant spin axis, because the orientation of
GMAT's built-in Earth cannot be modified. GMAT's ICRF axes about this planet did not give a gravity pole consistent with
the other tools, so these cases are propagated in the ``TerraMJ2000Eq`` system.
The initial state is rotated from ICRF to MJ2000Eq with the frame bias matrix
:data:`BIAS_ICRF_TO_MJ2000`, the spin axis is the ICRF +Z axis expressed in MJ2000Eq, and the reported states are
rotated back to ICRF.

The drag uses GMAT's ``Exponential`` atmosphere configured with ``Drag.InputFile`` as a single band from 0 km, which is
the single-scale density of ``cases.json`` (see ``writeGmatExponentialAtmosphere``), which is passed to GMAT without
conversion. The generator also writes ``gmat_density_probe.csv``: the Earth-fixed position, the altitude (``Earth.Altitude``)
and the density (``fm.AtmosDensity``, in kg/km^3, converted to kg/m^3) that GMAT reports at the points of ``density_probe``, which ``compare_with_basilisk.py``
compares with Basilisk and Orekit before it propagates anything. Cases that GMAT cannot model
(a box spacecraft with attitude-dependent drag) do not list ``gmat`` in their ``references`` and are skipped.

Next to the ephemerides, ``<output-dir>/gmat_manifest.json`` records for every case the effective configuration and its
hash, the frame, the tool version, the checksums of the external data and of the ephemeris, and a ``variant`` label,
which ``compare_with_basilisk.py`` validates before running Basilisk.
"""

import argparse
import csv
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from comparisonCommon import (HERE, caseDuration, caseEpoch, caseReferences, fileSha256, loadSpec,
                              probePoints, writeGmatCof, writeGmatExponentialAtmosphere, writeManifestEntry, writeProbe)

KM = 1000.0  # [m/km]
GMAT_BODY = {"sun": "Sun", "moon": "Luna"}
# Frame bias matrix of the rotation from the ICRF (GCRF) axes to the MJ2000Eq (EME2000) axes of GMAT, at J2000, from
# Orekit's GCRF to EME2000 transform. The columns are the ICRF axes in MJ2000Eq components.
BIAS_ICRF_TO_MJ2000 = np.array([[0.9999999999999942, -7.078279744199198e-08, 8.056217146976134e-08],
                                [7.078279477857338e-08, 0.9999999999999971, 3.3060414542221364e-08],
                                [-8.056217380986972e-08, -3.3060408839805523e-08, 0.9999999999999962]])  # [-]
ICRF_POLE_MJ2000 = BIAS_ICRF_TO_MJ2000[:, 2]  # [-] ICRF +Z axis in MJ2000Eq components
ICRF_POLE_RA_DEG = math.degrees(math.atan2(ICRF_POLE_MJ2000[1], ICRF_POLE_MJ2000[0])) % 360.0  # [deg]
ICRF_POLE_DEC_DEG = math.degrees(math.asin(ICRF_POLE_MJ2000[2]))  # [deg]


def gmatScript(spec, case, cofPath, reportPath, atmospherePath=None):
    """Return the GMAT script text for one case.

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
        cofPath (Path): gravity file (used when the gravity degree is positive or the case has drag).
        atmospherePath (Path): exponential atmosphere table, used by the cases with drag.
        reportPath (Path): GMAT report file to write.
    """
    sc = spec["spacecraft"]
    r0 = [x / KM for x in case["r0_m"]]  # [km]
    v0 = [x / KM for x in case["v0_m_s"]]  # [km/s]
    degree = case["gravity"]["degree"]
    rotating = case["earth_rotation"]
    body = "Earth" if rotating else "Terra"
    cs = f"{body}ICRF" if rotating else f"{body}MJ2000Eq"  # Terra: see the module docstring
    if not rotating:
        r0 = list(BIAS_ICRF_TO_MJ2000 @ r0)  # [km]
        v0 = list(BIAS_ICRF_TO_MJ2000 @ v0)  # [km/s]
    epoch = caseEpoch(spec, case)  # "YYYY-MM-DDTHH:MM:SS.sss"
    month = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov",
             "Dec"][int(epoch[5:7]) - 1]
    gmatEpoch = f"{epoch[8:10]} {month} {epoch[0:4]} {epoch[11:]}"
    steps = int(round(caseDuration(spec, case) / spec["sample_period_s"]))
    thirdBodies = [GMAT_BODY[b] for b in case["third_bodies"]]

    lines = []
    if not rotating:
        lines += [
            # User-defined planet: the built-in Earth orientation cannot be modified in GMAT
            "Create Planet Terra;",
            "GMAT Terra.CentralBody = 'Sun';",
            "GMAT Terra.NAIFId = 399;",
            "GMAT Terra.EquatorialRadius = %.12f;" % (spec["equatorial_radius_m"] / KM),
            "GMAT Terra.Flattening = %.12f;" % spec["earth_flattening"],
            "GMAT Terra.Mu = %.12f;" % (spec["mu_m3_s2"] / KM**3),
            "GMAT Terra.RotationDataSource = 'IAUSimplified';",
            "GMAT Terra.SpinAxisRAConstant = %.12f;" % ICRF_POLE_RA_DEG,
            "GMAT Terra.SpinAxisRARate = 0;",
            "GMAT Terra.SpinAxisDECConstant = %.12f;" % ICRF_POLE_DEC_DEG,
            "GMAT Terra.SpinAxisDECRate = 0;",
            "GMAT Terra.RotationConstant = 190.147;",
            "GMAT Terra.RotationRate = 360.9856235;",
            "Create CoordinateSystem TerraMJ2000Eq;",
            "GMAT TerraMJ2000Eq.Origin = Terra;",
            "GMAT TerraMJ2000Eq.Axes = MJ2000Eq;",
        ]
    lines += [
        "Create Spacecraft sat;",
        "sat.DateFormat = UTCGregorian;",
        f"sat.Epoch = '{gmatEpoch}';",
        f"sat.CoordinateSystem = {cs};",
        "sat.DisplayStateType = Cartesian;",
        "sat.X = %.12f;" % r0[0],
        "sat.Y = %.12f;" % r0[1],
        "sat.Z = %.12f;" % r0[2],
        "sat.VX = %.12f;" % v0[0],
        "sat.VY = %.12f;" % v0[1],
        "sat.VZ = %.12f;" % v0[2],
        "sat.DryMass = %.6f;" % sc["mass_kg"],
        "sat.Cd = %.6f;" % sc["drag_cd"],
        "sat.Cr = %.6f;" % sc["srp_cr"],
        "sat.DragArea = %.6f;" % sc["drag_area_m2"],
        "sat.SRPArea = %.6f;" % sc["srp_area_m2"],
        "Create ForceModel fm;",
        f"fm.CentralBody = {body};",
    ]
    if degree > 0 or case["drag"]:  # GMAT supports drag only for a gravity-field primary body
        lines += [
            f"fm.PrimaryBodies = {{{body}}};",
            f"fm.PointMasses = {{{', '.join(thirdBodies)}}};",
            f"fm.GravityField.{body}.PotentialFile = '{cofPath}';",
            f"fm.GravityField.{body}.Degree = {degree};",
            f"fm.GravityField.{body}.Order = {case['gravity']['order']};",
            f"fm.GravityField.{body}.TideModel = 'None';",
        ]
    else:
        lines += ["fm.PrimaryBodies = {};",
                  f"fm.PointMasses = {{{', '.join([body] + thirdBodies)}}};"]
    if case["srp"]:
        lines += [
            "fm.SRP = On;",
            "fm.SRP.Flux = %.6f;" % spec["srp"]["solar_flux_w_m2"],
            "fm.SRP.SRPModel = Spherical;",
            "fm.SRP.Nominal_Sun = %.6f;" % spec["srp"]["astronomical_unit_km"],
        ]
    else:
        lines += ["fm.SRP = Off;"]
    if case["drag"]:
        lines += [
            "fm.Drag.AtmosphereModel = Exponential;",
            f"fm.Drag.InputFile = '{atmospherePath}';",
        ]
    else:
        lines += ["fm.Drag = None;"]
    report = "Report rf sat.ElapsedSecs " + " ".join(
        f"sat.{cs}.{c}" for c in ("X", "Y", "Z", "VX", "VY", "VZ")) + ";"
    lines += [
        "Create Propagator prop;",
        "prop.FM = fm;",
        "prop.Type = PrinceDormand78;",
        "prop.InitialStepSize = 10;",
        "prop.Accuracy = 1e-12;",
        "prop.MinStep = 1e-6;",
        "prop.MaxStep = 300;",
        "prop.MaxStepAttempts = 50;",
        "Create ReportFile rf;",
        f"rf.Filename = '{reportPath}';",
        "rf.Precision = 16;",
        "rf.WriteHeaders = false;",
        "rf.LeftJustify = On;",
        "rf.ZeroFill = Off;",
        "rf.FixedWidth = false;",
        "rf.Delimiter = ',';",
        "rf.WriteReport = true;",
        "Create Variable i;",
        "BeginMissionSequence;",
    ]
    lines += [
        report,
        f"For i = 1:{steps}",
        "   Propagate prop(sat) {sat.ElapsedSecs = %.1f};" % spec["sample_period_s"],
        "   " + report,
        "EndFor;",
    ]
    return "\n".join(lines) + "\n"


def gmatProbeScript(spec, cofPath, atmospherePath, reportPath):
    """Return the GMAT script that reports the altitude and the density of the exponential atmosphere at the probe points.

    The spacecraft is placed at rest in the Earth-fixed system and propagated for a millisecond with the drag force
    model, after which GMAT reports its Earth-fixed position, its altitude above the Earth ellipsoid and the density of
    the atmosphere at its position.

    Args:
        spec (dict): parsed ``cases.json`` content.
        cofPath (Path): gravity file (GMAT needs a gravity field primary body for drag).
        atmospherePath (Path): exponential atmosphere table.
        reportPath (Path): GMAT report file to write.
    """
    epoch = spec["epoch_utc"]  # "YYYY-MM-DDTHH:MM:SS.sss"
    month = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov",
             "Dec"][int(epoch[5:7]) - 1]
    lines = [
        "Create Spacecraft sat;",
        "sat.DateFormat = UTCGregorian;",
        f"sat.Epoch = '{epoch[8:10]} {month} {epoch[0:4]} {epoch[11:]}';",
        "sat.CoordinateSystem = EarthFixed;",
        "sat.DisplayStateType = Cartesian;",
        "sat.DryMass = %.6f;" % spec["spacecraft"]["mass_kg"],
        "sat.DragArea = %.6f;" % spec["spacecraft"]["drag_area_m2"],
        "sat.Cd = %.6f;" % spec["spacecraft"]["drag_cd"],
        "Create ForceModel fm;",
        "fm.CentralBody = Earth;",
        "fm.PrimaryBodies = {Earth};",
        "fm.PointMasses = {};",
        f"fm.GravityField.Earth.PotentialFile = '{cofPath}';",
        "fm.GravityField.Earth.Degree = 2;",
        "fm.GravityField.Earth.Order = 0;",
        "fm.GravityField.Earth.TideModel = 'None';",
        "fm.SRP = Off;",
        "fm.Drag.AtmosphereModel = Exponential;",
        f"fm.Drag.InputFile = '{atmospherePath}';",
        "Create Propagator prop;",
        "prop.FM = fm;",
        "prop.Type = PrinceDormand78;",
        "prop.InitialStepSize = 0.001;",
        "prop.Accuracy = 1e-12;",
        "prop.MinStep = 1e-6;",
        "prop.MaxStep = 300;",
        "Create ReportFile rf;",
        f"rf.Filename = '{reportPath}';",
        "rf.Precision = 16;",
        "rf.WriteHeaders = false;",
        "rf.LeftJustify = On;",
        "rf.ZeroFill = Off;",
        "rf.FixedWidth = false;",
        "rf.Delimiter = ',';",
        "rf.WriteReport = true;",
        "BeginMissionSequence;",
    ]
    for x, y, z, _ in probePoints(spec):
        lines += [
            "sat.X = %.12f;" % (x / KM),
            "sat.Y = %.12f;" % (y / KM),
            "sat.Z = %.12f;" % (z / KM),
            "sat.VX = 0;",
            "sat.VY = 0;",
            "sat.VZ = 0;",
            "Propagate prop(sat) {sat.ElapsedSecs = 0.001};",
            "Report rf sat.EarthFixed.X sat.EarthFixed.Y sat.EarthFixed.Z sat.Earth.Altitude sat.fm.AtmosDensity;",
        ]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("gmatRoot", type=Path, help="GMAT installation folder (contains bin/)")
    parser.add_argument("cases", nargs="*", help="case names to generate (default: all)")
    parser.add_argument("--output-dir", type=Path, default=HERE / "data",
                        help="folder where the reference ephemerides are written (default: data next to this script)")
    args = parser.parse_args()

    spec = loadSpec()
    outDir = args.output_dir
    outDir.mkdir(parents=True, exist_ok=True)
    console = args.gmatRoot / "bin" / "GmatConsole"

    gmatVersion = f"GMAT (install folder {args.gmatRoot.resolve().name})"
    externalData = {"gravity_coefficients": {"id": spec["gravity_coefficients_file"],
                                             "sha256": fileSha256(HERE / spec["gravity_coefficients_file"])}}
    options = {"propagator": "PrinceDormand78", "initial_step_s": 10.0, "accuracy": 1.0e-12, "min_step_s": 1.0e-6,
               "max_step_s": 300.0, "srp_model": "Spherical", "third_body_model": "point mass",
               "atmosphere": "Exponential, single band from 0 km"}

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        probeCof, probeAtmosphere = tmp / "probe.cof", tmp / "probe_atmosphere.txt"
        writeGmatCof(probeCof, spec, 2, 0)
        writeGmatExponentialAtmosphere(probeAtmosphere, spec)
        probeReport, probeScript = tmp / "probe.txt", tmp / "probe.script"
        probeScript.write_text(gmatProbeScript(spec, probeCof, probeAtmosphere, probeReport))
        subprocess.run([str(console), "--run", str(probeScript), "--exit"], cwd=console.parent, check=True)
        probeRows = []
        for line in probeReport.read_text().splitlines():
            if line.strip():
                x, y, z, altitude, density = (float(v) for v in line.split(","))
                probeRows.append((x * KM, y * KM, z * KM, altitude * KM, density / KM**3))  # [m] x3, [m], [kg/m^3]
        writeProbe(outDir, "gmat", spec, probeRows, gmatVersion,
                   {"atmosphere": "Exponential, single band from 0 km", "probe_step_s": 0.001})
        print(f"density probe: {len(probeRows)} points")

        for name, case in spec["cases"].items():
            if args.cases and name not in args.cases:
                continue
            if "gmat" not in caseReferences(case):
                continue
            cof = tmp / f"{name}.cof"
            if case["gravity"]["degree"] > 0 or case["drag"]:
                writeGmatCof(cof, spec, max(case["gravity"]["degree"], 2), case["gravity"]["order"])
            atmosphere = tmp / f"{name}_atmosphere.txt"
            writeGmatExponentialAtmosphere(atmosphere, spec)
            report = tmp / f"{name}.txt"
            script = tmp / f"{name}.script"
            script.write_text(gmatScript(spec, case, cof, report, atmosphere))
            subprocess.run([str(console), "--run", str(script), "--exit"],
                           cwd=console.parent, check=True)
            rows = [[float(v) for v in line.split(",")]
                    for line in report.read_text().splitlines() if line.strip()]
            csvPath = outDir / f"gmat_{name}.csv"
            with open(csvPath, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
                for k, row in enumerate(rows):
                    # GMAT stops at ElapsedSecs within ~1e-6 s of the request. Shift the state
                    # back to the nominal sample time with its own velocity (first order).
                    tNominal = k * spec["sample_period_s"]  # [s]
                    dt = row[0] - tNominal  # [s]
                    r = np.array([(x - v * dt) * KM for x, v in zip(row[1:4], row[4:7])])  # [m]
                    v = np.array(row[4:7]) * KM  # [m/s]
                    if not case["earth_rotation"]:  # propagated in MJ2000Eq, reported in ICRF
                        r, v = BIAS_ICRF_TO_MJ2000.T @ r, BIAS_ICRF_TO_MJ2000.T @ v
                    writer.writerow([f"{tNominal:.1f}"] + [f"{c:.9f}" for c in r] + [f"{c:.9f}" for c in v])
            writeManifestEntry(outDir, "gmat", spec, name, csvPath, spec["inertial_frame"], gmatVersion, options,
                               externalData)
            print(f"{name}: {len(rows)} samples")


if __name__ == "__main__":
    main()
