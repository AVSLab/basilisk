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
loadPolyFromFileToList = grav_coeff_ops.loadPolyFromFileToList


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


def _load_header_and_rows(file_path: Path, max_degree: int):
    with file_path.open("r", newline="") as grav_file:
        grav_reader = csv.reader(grav_file, delimiter=",")
        header = next(grav_reader)
        rows = []
        for grav_row in grav_reader:
            if int(grav_row[0]) > max_degree:
                break
            rows.append(grav_row)

    return header, rows


def _write_temp_gravity_file(tmp_path: Path, header, rows):
    temp_gravity_file = tmp_path / "temporaryGravityFile.csv"
    with temp_gravity_file.open("w", newline="") as grav_file:
        grav_writer = csv.writer(grav_file)
        grav_writer.writerow(header)
        grav_writer.writerows(rows)

    return temp_gravity_file


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


def test_load_grav_from_file_to_list_rejects_negative_degree():
    """Verify negative requested degree raises a clear ValueError."""
    with pytest.raises(ValueError, match="must be non-negative"):
        loadGravFromFileToList(str(GGM03S_PATH), maxDeg=-1)


def test_load_grav_from_file_to_list_uses_row_order_column(tmp_path):
    """Verify rows are placed by explicit order value, even if row order is shuffled."""
    header, rows = _load_header_and_rows(GGM03S_PATH, max_degree=2)
    rows_by_index = {(int(row[0]), int(row[1])): row for row in rows}
    shuffled_rows = [
        rows_by_index[(0, 0)],
        rows_by_index[(1, 0)],
        rows_by_index[(1, 1)],
        rows_by_index[(2, 0)],
        rows_by_index[(2, 2)],
        rows_by_index[(2, 1)],
    ]
    temp_file = _write_temp_gravity_file(tmp_path, header, shuffled_rows)

    c_ref, s_ref, _, _, _, _ = _load_reference_coefficients(GGM03S_PATH, max_degree=2)
    c_list, s_list, _, _ = loadGravFromFileToList(str(temp_file), maxDeg=2)

    for degree in range(3):
        np.testing.assert_allclose(c_list[degree], c_ref[degree], rtol=0.0, atol=0.0)
        np.testing.assert_allclose(s_list[degree], s_ref[degree], rtol=0.0, atol=0.0)


def test_load_grav_from_file_to_list_preserves_zero_for_missing_order(tmp_path):
    """Verify missing coefficients remain zero at their degree/order index."""
    header, rows = _load_header_and_rows(GGM03S_PATH, max_degree=2)
    rows_by_index = {(int(row[0]), int(row[1])): row for row in rows}
    missing_order_rows = [
        rows_by_index[(0, 0)],
        rows_by_index[(1, 0)],
        rows_by_index[(1, 1)],
        rows_by_index[(2, 0)],
        rows_by_index[(2, 2)],
    ]
    temp_file = _write_temp_gravity_file(tmp_path, header, missing_order_rows)

    c_ref, s_ref, _, _, _, _ = _load_reference_coefficients(GGM03S_PATH, max_degree=2)
    c_list, s_list, _, _ = loadGravFromFileToList(str(temp_file), maxDeg=2)

    assert c_list[2][1] == 0.0
    assert s_list[2][1] == 0.0
    assert c_list[2][2] == c_ref[2][2]
    assert s_list[2][2] == s_ref[2][2]


def test_load_grav_from_file_to_list_rejects_degree_regression(tmp_path):
    """Verify decreasing degree rows are rejected."""
    header, rows = _load_header_and_rows(GGM03S_PATH, max_degree=2)
    rows_by_index = {(int(row[0]), int(row[1])): row for row in rows}
    regressed_rows = [
        rows_by_index[(0, 0)],
        rows_by_index[(1, 0)],
        rows_by_index[(2, 0)],
        rows_by_index[(1, 1)],
    ]
    temp_file = _write_temp_gravity_file(tmp_path, header, regressed_rows)

    with pytest.raises(ValueError, match="non-decreasing degree"):
        loadGravFromFileToList(str(temp_file), maxDeg=2)


def test_load_poly_from_file_to_list_rejects_empty_tab_file(tmp_path):
    """Verify empty .tab polyhedral files are rejected with a clear error."""
    empty_tab_file = tmp_path / "emptyShape.tab"
    empty_tab_file.write_text("")

    with pytest.raises(ValueError, match="polyhedral file is empty"):
        loadPolyFromFileToList(str(empty_tab_file))
