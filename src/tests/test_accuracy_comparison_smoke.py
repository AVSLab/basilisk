#
# ISC License
#
# Copyright (c) 2026, Suhas Beemineni
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
#

"""Run the accuracy-comparison pipeline with analytic circular-orbit fixtures.

The CSV files occupy the GMAT and Orekit input slots, but are synthetic test
data, not output from either tool. Their manifests explicitly identify them
as fixtures. Native Basilisk dynamics, reference validation, sample alignment,
and error reporting run normally. No external reference tools or support-data
downloads are needed. Run from the repository root with::

    python -m pytest src/tests/test_accuracy_comparison_smoke.py
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ACCURACY_DIR = Path(__file__).resolve().parents[2] / "benchmarks" / "accuracyComparison"
sys.path.insert(0, str(ACCURACY_DIR))

import comparisonCommon as common  # noqa: E402
import compare_with_basilisk as comparison  # noqa: E402

CASE_NAME = "leo_twobody"
DURATION_S = 600.0  # [s]
SAMPLE_PERIOD_S = 30.0  # [s]
POSITION_TOLERANCE_M = 0.01  # [m]
VELOCITY_TOLERANCE_M_S = 1.0e-4  # [m/s]
CSV_HEADER = "t_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s"


def _circular_reference(spec):
    """Return exact inertial states for a circular orbit, independently of Basilisk.

    :param spec: Comparison configuration with a circular initial state.
    :return: Array of columns ``(t, r_xyz, v_xyz)`` in seconds, meters, and
        meters per second, including the initial and final sample.
    """
    case = spec["cases"][CASE_NAME]
    initial_position = np.asarray(case["r0_m"])
    initial_velocity = np.asarray(case["v0_m_s"])
    radius = np.linalg.norm(initial_position)
    angular_rate = np.sqrt(spec["mu_m3_s2"]/radius**3)
    times = np.arange(0.0, DURATION_S + SAMPLE_PERIOD_S, SAMPLE_PERIOD_S)  # [s]
    phase = angular_rate*times
    cosine = np.cos(phase)[:, None]
    sine = np.sin(phase)[:, None]
    positions = cosine*initial_position + sine*initial_velocity/angular_rate
    velocities = -sine*initial_position*angular_rate + cosine*initial_velocity
    return np.column_stack((times, positions, velocities))


def _write_reference(folder, spec, tool, states):
    """Write a synthetic CSV and valid fixture manifest for a reference input slot.

    :param folder: Temporary directory for test inputs.
    :param spec: Comparison configuration used to generate the fixture.
    :param tool: Reference slot, ``gmat`` or ``orekit``.
    :param states: Reference rows with columns ``(t, r_xyz, v_xyz)`` in SI units.
    """
    csv_path = folder / f"{tool}_{CASE_NAME}.csv"
    np.savetxt(csv_path, states, delimiter=",", header=CSV_HEADER, comments="")
    # These placeholder records are confined to the temporary fixture. No
    # GMAT/Orekit data were used, and no production validation is bypassed.
    external_data = {
        key: {"id": "synthetic circular-orbit fixture", "sha256": "0"*64}
        for key in common.REQUIRED_EXTERNAL_DATA[tool]
    }
    external_data["gravity_coefficients"] = common.gravityCoefficientsRecord(spec)
    common.writeManifestEntry(
        folder, tool, spec, CASE_NAME, csv_path, spec["inertial_frame"],
        "synthetic analytic fixture (not GMAT/Orekit output)", {}, external_data,
    )


@pytest.fixture
def circular_case(tmp_path, monkeypatch):
    """Provide a short circular case and fixtures without changing ``cases.json``."""
    spec = common.loadSpec()
    case = spec["cases"][CASE_NAME]
    # Keep the existing point-mass force settings; use an exactly circular
    # initial velocity so the independent reference needs no numerical solver.
    radius = np.linalg.norm(case["r0_m"])
    direction = np.asarray(case["v0_m_s"])/np.linalg.norm(case["v0_m_s"])
    case["v0_m_s"] = (direction*np.sqrt(spec["mu_m3_s2"]/radius)).tolist()
    spec["cases"] = {CASE_NAME: case}
    spec["duration_s"] = DURATION_S
    spec["sample_period_s"] = SAMPLE_PERIOD_S
    states = _circular_reference(spec)
    for tool in common.TOOLS:
        _write_reference(tmp_path, spec, tool, states)
    monkeypatch.setattr(comparison, "loadSpec", lambda: spec)
    return spec, tmp_path, states


@pytest.mark.parametrize("tools", [("gmat", "orekit"), ("orekit",)], ids=["both-tools", "orekit-only"])
def test_accuracy_comparison_runs_native_dynamics(circular_case, capsys, tools):
    """Validate the full comparison against independent circular-orbit states.

    Validation Test Description
    ---------------------------
    Run ten simulated minutes through native Basilisk propagation and the
    comparison's normal manifest, CSV, timestamp, and error-reporting paths.

    Test Parameter Discussion
    -------------------------
    ``tools`` selects both reference slots or only Orekit, exercising the
    supported reference-selection branches without invoking external tools.

    Description of Variables Being Tested
    -------------------------------------
    Each returned maximum position and velocity difference must be finite
    and below 1 cm and 0.1 mm/s respectively. The reported keys and fixture
    provenance must match the selected reference slots.
    """
    spec, folder, _ = circular_case
    spec["cases"][CASE_NAME]["references"] = list(tools)
    results = comparison.run([CASE_NAME], dataDir=folder)
    expected_keys = {"bsk_vs_orekit", "variants"}
    if "gmat" in tools:
        expected_keys.update(("bsk_vs_gmat", "gmat_vs_orekit"))
    assert set(results) == {CASE_NAME}
    assert set(results[CASE_NAME]) == expected_keys
    assert results[CASE_NAME]["variants"] == {}
    for key in expected_keys - {"variants"}:
        position_error, velocity_error = results[CASE_NAME][key]
        assert np.isfinite((position_error, velocity_error)).all()
        assert position_error < POSITION_TOLERANCE_M
        assert velocity_error < VELOCITY_TOLERANCE_M_S
    output = capsys.readouterr().out
    for tool in tools:
        assert f"{tool} references: synthetic analytic fixture" in output


def test_accuracy_comparison_reports_inaccurate_reference(circular_case):
    """Ensure errors remain observable when a valid reference contains a bad state."""
    spec, folder, states = circular_case
    changed = states.copy()
    position_offset = np.array([10.0, 20.0, 30.0])  # [m]
    velocity_offset = np.array([0.01, 0.02, 0.03])  # [m/s]
    changed[-1, 1:4] += position_offset
    changed[-1, 4:7] += velocity_offset
    _write_reference(folder, spec, "orekit", changed)
    results = comparison.run([CASE_NAME], dataDir=folder)[CASE_NAME]
    for key in ("bsk_vs_orekit", "gmat_vs_orekit"):
        position_error, velocity_error = results[key]
        assert position_error == pytest.approx(np.linalg.norm(position_offset), abs=POSITION_TOLERANCE_M)
        assert velocity_error == pytest.approx(np.linalg.norm(velocity_offset), abs=VELOCITY_TOLERANCE_M_S)
    assert results["bsk_vs_gmat"][0] < POSITION_TOLERANCE_M


def test_accuracy_comparison_rejects_misaligned_epochs(circular_case):
    """Reject reference rows from different epochs even with valid provenance."""
    spec, folder, states = circular_case
    changed = states.copy()
    epoch_offset_s = 1.0  # [s]
    changed[1:, 0] += epoch_offset_s
    _write_reference(folder, spec, "orekit", changed)
    with pytest.raises(ValueError, match="sample times.*differ"):
        comparison.run([CASE_NAME], dataDir=folder)
