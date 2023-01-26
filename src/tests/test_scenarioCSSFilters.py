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
# Purpose:  Demonstrates how to setup and use sun heading filters
# Author:   Thibaud Teil
# Creation Date:  November 20, 2017
#



import inspect
import os
import sys

import pytest

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioCSSFilters

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("FilterType, simTime", [
      ('uKF', 400)
    , ('EKF', 400)
    , ('OEKF', 400)
    , ('SEKF', 400)
    , ('SuKF', 400)
])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_Filters(show_plots, FilterType, simTime):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    try:
        scenarioCSSFilters.run(True, show_plots, FilterType, simTime)
    # except:
    #     print("Unexpected error:", sys.exc_info()[0])
    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("FAILDED:".format(err))
        # print("OS error: {0}".format(err))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages


