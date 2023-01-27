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
# Purpose:  Integrated test of the spacecraft() and gravity modules illustrating
#           a four body system, for a Patched Conics analysis of an interplanetary transfer
#           between Earth and Jupiter
#
# Author:   Divinaa Burder
# Creation Date:  Feb 4, 2019
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

import scenarioPatchedConics

@pytest.mark.scenarioTest
@pytest.mark.slowtest



# provide a unique test method name, starting with test_
def test_scenarioPatchedConics(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        dataPos, figureList = scenarioPatchedConics.run(show_plots)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioPatchedConics test are failed.")

    # setup truth data for unit test
    truePos = [
        [19503460698.246426, 1948347074.142926, 0.0]
    ]


    # compare the results to the truth values
    accuracy = 1000000.0 # meters
    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPos, accuracy, "r_BN_N Vector",testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("testFailCount: " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
