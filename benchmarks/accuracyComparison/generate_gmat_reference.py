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

Cases without Earth rotation use a user-defined planet with a constant spin axis along
inertial +Z, because the orientation of GMAT's built-in Earth cannot be modified. Cases
with Earth rotation use GMAT's built-in Earth with its default orientation model.
"""

import argparse
import csv
import subprocess
import tempfile
from pathlib import Path

from comparisonCommon import HERE, loadSpec, writeGmatCof

KM = 1000.0  # [m/km]
GMAT_BODY = {"sun": "Sun", "moon": "Luna"}


def gmatScript(spec, case, cofPath, reportPath):
    """Return the GMAT script text for one case.

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
        cofPath (Path): gravity file (used when the gravity degree is positive).
        reportPath (Path): GMAT report file to write.
    """
    sc = spec["spacecraft"]
    r0 = [x / KM for x in case["r0_m"]]  # [km]
    v0 = [x / KM for x in case["v0_m_s"]]  # [km/s]
    degree = case["gravity"]["degree"]
    rotating = case["earth_rotation"]
    body = "Earth" if rotating else "Terra"
    cs = f"{body}MJ2000Eq"
    epoch = spec["epoch_utc"]  # "YYYY-MM-DDTHH:MM:SS.sss"
    month = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov",
             "Dec"][int(epoch[5:7]) - 1]
    gmatEpoch = f"{epoch[8:10]} {month} {epoch[0:4]} {epoch[11:]}"
    steps = int(round(spec["duration_s"] / spec["sample_period_s"]))
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
            "GMAT Terra.SpinAxisRAConstant = 0;",
            "GMAT Terra.SpinAxisRARate = 0;",
            "GMAT Terra.SpinAxisDECConstant = 90;",
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
        sw = spec["space_weather"]
        lines += [
            "fm.Drag.AtmosphereModel = NRLMSISE00;",
            "fm.Drag.HistoricWeatherSource = 'ConstantFluxAndGeoMag';",
            "fm.Drag.PredictedWeatherSource = 'ConstantFluxAndGeoMag';",
            "fm.Drag.F107 = %.6f;" % sw["f107"],
            "fm.Drag.F107A = %.6f;" % sw["f107"],
            "fm.Drag.MagneticIndex = %.6f;" % sw["kp"],
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
        report,
        f"For i = 1:{steps}",
        "   Propagate prop(sat) {sat.ElapsedSecs = %.1f};" % spec["sample_period_s"],
        "   " + report,
        "EndFor;",
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

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for name, case in spec["cases"].items():
            if args.cases and name not in args.cases:
                continue
            cof = tmp / f"{name}.cof"
            if case["gravity"]["degree"] > 0 or case["drag"]:
                writeGmatCof(cof, spec, max(case["gravity"]["degree"], 2), case["gravity"]["order"])
            report = tmp / f"{name}.txt"
            script = tmp / f"{name}.script"
            script.write_text(gmatScript(spec, case, cof, report))
            subprocess.run([str(console), "--run", str(script), "--exit"],
                           cwd=console.parent, check=True)
            rows = [[float(v) for v in line.split(",")]
                    for line in report.read_text().splitlines() if line.strip()]
            with open(outDir / f"gmat_{name}.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
                for k, row in enumerate(rows):
                    # GMAT stops at ElapsedSecs within ~1e-6 s of the request. Shift the state
                    # back to the nominal sample time with its own velocity (first order).
                    tNominal = k * spec["sample_period_s"]  # [s]
                    dt = row[0] - tNominal  # [s]
                    state = [(x - v * dt) * KM for x, v in zip(row[1:4], row[4:7])]
                    writer.writerow([f"{tNominal:.1f}"] + [f"{v:.9f}" for v in state]
                                    + [f"{v * KM:.9f}" for v in row[4:7]])
            print(f"{name}: {len(rows)} samples")


if __name__ == "__main__":
    main()
