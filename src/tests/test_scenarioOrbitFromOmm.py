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

from copy import copy
from pathlib import Path
import sys

import numpy as np
import pytest

from Basilisk.utilities import orbitalMotion, simHelpers

EXAMPLES_PATH = Path(__file__).resolve().parents[2] / "examples"
sys.path.append(str(EXAMPLES_PATH))
import scenarioOrbitFromOmm


@pytest.mark.scenarioTest
def test_scenario_orbit_from_omm(show_plots, tmp_path, monkeypatch):
    """Verify import, epoch alignment, and propagation against the Kepler solution.

    Run from an unrelated directory to exercise portable sample-data loading.
    Check Cartesian initialization, compare final states to analytic two-body
    propagation, and save the tutorial figures for the Sphinx documentation.
    """
    monkeypatch.chdir(tmp_path)
    results, figures = scenarioOrbitFromOmm.run(show_plots)
    satellites = results["satellites"]
    assert [satellite["record"].noradID for satellite in satellites] == ["999000001", "999000002"]
    for satellite in satellites:
        record = satellite["record"]
        assert record.ommEpoch == results["epoch"]
        assert satellite["time_s"][0] == 0.0
        assert satellite["time_s"][-1] >= 2.0*satellite["period_s"]
        assert np.all(np.isfinite(satellite["position_m"]))
        assert np.all(np.isfinite(satellite["velocity_m_s"]))
        initial_r, initial_v = orbitalMotion.elem2rv(results["mu"], record.oe)
        np.testing.assert_allclose(satellite["position_m"][0], initial_r, rtol=0, atol=1e-8)  # [m]
        np.testing.assert_allclose(satellite["velocity_m_s"][0], initial_v, rtol=0, atol=1e-11)  # [m/s]

        # Solve Kepler's equation independently of the spacecraft numerical integrator.
        elements = copy(record.oe)
        initial_E = orbitalMotion.f2E(elements.f, elements.e)  # [rad]
        initial_M = orbitalMotion.E2M(initial_E, elements.e)  # [rad]
        mean_motion = np.sqrt(results["mu"]/elements.a**3)  # [rad/s]
        final_M = (initial_M+mean_motion*satellite["time_s"][-1]) % (2.0*np.pi)  # [rad]
        elements.f = orbitalMotion.E2f(orbitalMotion.M2E(final_M, elements.e), elements.e)
        expected_r, expected_v = orbitalMotion.elem2rv(results["mu"], elements)
        np.testing.assert_allclose(satellite["position_m"][-1], expected_r, rtol=0, atol=0.2)  # [m]
        np.testing.assert_allclose(satellite["velocity_m_s"][-1], expected_v, rtol=0, atol=2e-4)  # [m/s]

    assert set(figures) == {"scenarioOrbitFromOmmOrbits", "scenarioOrbitFromOmmAltitudeSpeed"}
    for name, figure in figures.items():
        simHelpers.saveScenarioFigure(name, figure, str(Path(__file__).resolve().parent))


@pytest.mark.scenarioTest
def test_scenario_rejects_mixed_epochs(tmp_path):
    """Prevent catalog states at different epochs from being initialized simultaneously."""
    text = (EXAMPLES_PATH / "dataForExamples" / "ommExample.kvn").read_text(encoding="utf-8")
    text = text.replace("EPOCH = 2026-09-24", "EPOCH = 2026-09-25", 1)
    path = tmp_path / "mixed-epochs.kvn"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="common epoch"):
        scenarioOrbitFromOmm.run(False, path)
