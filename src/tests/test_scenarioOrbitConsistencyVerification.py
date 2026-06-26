
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

import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioOrbitConsistencyVerification


@pytest.mark.scenarioTest
def test_scenarioOrbitConsistencyVerification(show_plots):
    """Verify the spherical-harmonic propagation consistency scenario.

    A short, low-degree, coarse-step run is used so the test is fast (~1.5 s,
    versus ~7 s for a 20-day/degree-10/10-s run). The physical check is that
    connecting Earth rotation keeps the eccentricity bounded while the
    non-rotating case drifts, i.e. the non-rotating eccentricity excursion is
    several times larger than the rotating one (issue #1352)."""
    testFailCount = 0
    testMessages = []

    try:
        driftRatio, figureList = scenarioOrbitConsistencyVerification.run(
            show_plots, simDays=12.0, gravDeg=8, stepSeconds=30.0
        )
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)
    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioOrbitConsistencyVerification test failed to run: " + str(err))
        driftRatio = 0.0

    # The non-rotating eccentricity drift should dwarf the rotating one. A modest
    # threshold keeps this robust to integrator/sampling details while still
    # capturing the qualitative result.
    if not driftRatio > 3.0:
        testFailCount += 1
        testMessages.append(
            "FAILED: non-rotating eccentricity drift not dominant (ratio=%g)" % driftRatio
        )

    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_scenarioOrbitConsistencyVerification(False)
