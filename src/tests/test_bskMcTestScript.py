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
# Monte Carlo tests
#
# Purpose:  This script calls a series of bskSim Monte Carlo related simulations to ensure
# that they complete properly.
# Creation Date:  Nov 18, 2019
#


import importlib
import inspect
import os
import platform
import shutil
import sys

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/MonteCarloExamples')


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# @pytest.mark.skip(reason="MC can have issues completing.")

@pytest.mark.skipif(sys.version_info < (3, 9) and platform.system() == 'Darwin',
                    reason="Test has issues with Controller class and older python.")

@pytest.mark.slowtest
@pytest.mark.scenarioTest
def test_scenarioBskMcScenarios(show_plots):
    # These need to be run in serial such that the data is produced for analysis
    scenarios = ['scenario_AttFeedbackMC',
                 'scenarioAnalyzeMonteCarlo',
                 'scenarioRerunMonteCarlo']

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    for bskSimCase in scenarios:
        # import the bskSim script to be tested
        scene_plt = importlib.import_module(bskSimCase)

        try:
            figureList = scene_plt.run(False)

        except OSError as err:
            testFailCount = testFailCount + 1
            testMessages.append("OS error: {0}".format(err))


    print(path+ "/../../examples/MonteCarloExamples/scenario_AttFeedbackMC/")
    if os.path.exists(path+ "/../../examples/MonteCarloExamples/scenario_AttFeedbackMC/"):
        shutil.rmtree(path+ "/../../examples/MonteCarloExamples/scenario_AttFeedbackMC/")
    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

