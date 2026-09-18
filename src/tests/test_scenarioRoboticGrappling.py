#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

#
# Basilisk Scenario Integrated Test
#
# Purpose:  Integrated test for scenarioRoboticGrappling. The scenario default
#           uses dt = 1e-4 s for paper-quality results; the test passes
#           dt = 0.01 s and 25% linear constraint damping to save all four
#           documentation plots quickly. The test also shrinks the
#           free-flight approach gap and trims the post-retraction berth
#           duration to keep wall time small.
#

import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioRoboticGrappling


@pytest.mark.scenarioTest
def test_scenarioRoboticGrappling(show_plots):
    """Run the grappling scenario at coarsened dt and persist its figures."""
    testFailCount = 0
    testMessages = []

    try:
        figureList = scenarioRoboticGrappling.run(
            show_plots,
            gain=1e4,
            dynRateSeconds=0.01,  # [s]
            approachGapMeters=0.2,    # 50 s of drift → ~10 s
            berthDurationSeconds=45.0,  # retraction + brief hold (was 90 s)
            constraintLinearDampingScale=0.25,  # [-]
        )
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)
    except OSError:
        testFailCount += 1
        testMessages.append("scenarioRoboticGrappling test failed.")

    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    assert testFailCount < 1, testMessages
