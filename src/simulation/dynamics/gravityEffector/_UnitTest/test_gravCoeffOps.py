# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import csv
import importlib.util
from pathlib import Path

import numpy as np
import pytest

GRAV_COEFF_OPS_PATH = Path(__file__).resolve().parents[1] / "gravCoeffOps.py"
GGM03S_PATH = Path(__file__).resolve().with_name("GGM03S.txt")

grav_coeff_ops_spec = importlib.util.spec_from_file_location(
    "gravCoeffOps", GRAV_COEFF_OPS_PATH
)
if grav_coeff_ops_spec is None or grav_coeff_ops_spec.loader is None:
    raise ImportError(f"Unable to load module from {GRAV_COEFF_OPS_PATH}")
grav_coeff_ops = importlib.util.module_from_spec(grav_coeff_ops_spec)
grav_coeff_ops_spec.loader.exec_module(grav_coeff_ops)

loadGravFromFileToList = grav_coeff_ops.loadGravFromFileToList


def _load_reference_coefficients(file_path: Path, max_degree: int):
    with file_path.open("r", newline="") as grav_file:
        grav_reader = csv.reader(grav_file, delimiter=",")
        first_row = next(grav_reader)
        rad_equator = float(first_row[0])
        mu = float(first_row[1])
        max_degree_file = int(first_row[3])
        max_order_file = int(first_row[4])

        c_ref = [[0.0] * (degree + 1) for degree in range(max_degree + 1)]
        s_ref = [[0.0] * (degree + 1) for degree in range(max_degree + 1)]

        for grav_row in grav_reader:
            degree = int(grav_row[0])
            order = int(grav_row[1])
            if degree > max_degree:
                break
            c_ref[degree][order] = float(grav_row[2])
            s_ref[degree][order] = float(grav_row[3])

    return c_ref, s_ref, mu, rad_equator, max_degree_file, max_order_file


def test_load_grav_from_file_to_list_respects_max_degree():
    """Verify spherical-harmonic loading is truncated to the requested degree."""
    max_degree = 8

    c_list, s_list, _, _ = loadGravFromFileToList(str(GGM03S_PATH), maxDeg=max_degree)

    assert len(c_list) == max_degree + 1
    assert len(s_list) == max_degree + 1
    for degree in range(max_degree + 1):
        assert len(c_list[degree]) == degree + 1
        assert len(s_list[degree]) == degree + 1


def test_load_grav_from_file_to_list_includes_requested_last_degree():
    """Verify coefficients for the highest requested degree are loaded."""
    max_degree = 20

    c_ref, s_ref, mu_ref, rad_ref, _, _ = _load_reference_coefficients(
        GGM03S_PATH, max_degree
    )
    c_list, s_list, mu, rad_equator = loadGravFromFileToList(
        str(GGM03S_PATH), maxDeg=max_degree
    )

    assert mu == mu_ref
    assert rad_equator == rad_ref
    for degree in range(max_degree + 1):
        np.testing.assert_allclose(c_list[degree], c_ref[degree], rtol=0.0, atol=0.0)
        np.testing.assert_allclose(s_list[degree], s_ref[degree], rtol=0.0, atol=0.0)


def test_load_grav_from_file_to_list_rejects_degree_above_file_limit():
    """Verify an out-of-range degree request raises a clear ValueError."""
    _, _, _, _, max_degree_file, max_order_file = _load_reference_coefficients(
        GGM03S_PATH, max_degree=0
    )
    requested_degree = min(max_degree_file, max_order_file) + 1

    with pytest.raises(ValueError, match="maximum degree/order"):
        loadGravFromFileToList(str(GGM03S_PATH), maxDeg=requested_degree)
