
# ISC License
#
# Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test for the albedo module
# Author:   Demet Cilden-Guler
# Creation Date:  May 27, 2020
#

import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioAlbedo

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("albedoData, multipleInstrument, multiplePlanet, useEclipse", [
    (True, False, False, False),
    (False, True, False, False),
    (False, True, False, True),
    (False, True, True, False)
])
@pytest.mark.scenarioTest

def test_scenarioAlbedo(show_plots, albedoData, multipleInstrument, multiplePlanet, useEclipse):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioAlbedo.run(show_plots, albedoData, multipleInstrument, multiplePlanet, useEclipse, 25)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("Albedo scenario test are failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

