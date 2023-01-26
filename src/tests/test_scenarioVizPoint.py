#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules.  Illustrates
# how to setup an orbital simulation that uses multiple gravitational bodies.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 01, 2018
#


import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pytestmark = pytest.mark.skip(reason="viz interface not built without required libraries")


sys.path.append(path + '/../../examples')
import scenarioVizPoint

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Scott's brain no-worky\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("missionType", [
    ('dscovr'),
    ('marsOrbit')]
)
@pytest.mark.scenarioTest


def test_scenarioViz(show_plots, missionType):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    try:
        figureList = scenarioVizPoint.run(show_plots, missionType, False)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioVizPoint  test are failed.")

        #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
        assert testFailCount < 1, testMessages

