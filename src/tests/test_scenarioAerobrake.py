#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Integrated test for scenarioAerobrake. The scenario default runs
#           four cases (drag model x attitude control) over two full orbits
#           at dt = 0.2 s for paper-quality results. The test exercises a
#           stripped-down configuration: just the two controlled cases at
#           dt = 0.5 s over 1.2 orbits, enough to capture the first
#           periapsis aerobraking step and the modeling-error opening up
#           between the branched and legacy drag models.
#

import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioAerobrake


@pytest.mark.scenarioTest
def test_scenarioAerobrake(show_plots):
    """Run the aerobrake scenario at coarsened dt with two cases and persist its figures."""
    testFailCount = 0
    testMessages = []

    testCases = (
        ("branched + ctrl", True, "branched"),
        ("legacy + ctrl",   True, "legacy"),
    )

    try:
        figureList = scenarioAerobrake.run(
            show_plots,
            dynRateSeconds=0.5,
            nOrbitsToRun=1.2,
            cases=testCases,
        )
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)
    except OSError:
        testFailCount += 1
        testMessages.append("scenarioAerobrake test failed.")

    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    assert testFailCount < 1, testMessages
