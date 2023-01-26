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
# Basilisk Integrated Test
#
# Purpose:  Integrated test of the MonteCarlo module with Spice usage.
#

import inspect
import os
import platform
import sys

import pytest

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')
import scenarioMonteCarloSpice

@pytest.mark.skipif(sys.version_info < (3, 9)  and platform.system() == 'Darwin',
                    reason="Test has issues with Controller class and older python.")
@pytest.mark.scenarioTest
def test_MonteCarloSimulationDatashader(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # each test method requires a single assert method to be called
    try:
        scenarioMonteCarloSpice.run()
    except OSError as err:
        testFailCount += 1
        testMessages.append("MC Spice tutorial failed.")

    assert testFailCount < 1, testMessages

    return


