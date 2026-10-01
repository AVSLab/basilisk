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
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def loadSpec():
    """Return the parsed ``cases.json``."""
    return json.loads((HERE / "cases.json").read_text())


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
    omegaEarth = 7.2921150e-5  # [rad/s] required by the file format, unused here
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
