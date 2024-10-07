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

from bokeh.io import output_file, save
from bokeh.server.server import Server
from bokeh.application import Application
from bokeh.application.handlers.function import FunctionHandler
from tornado.ioloop import IOLoop

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/MonteCarloExamples')


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# @pytest.mark.skip(reason="MC can have issues completing.")

@pytest.mark.skipif(sys.version_info < (3, 9),
                    reason="Test has issues with Controller class and older python.")

@pytest.mark.slowtest
@pytest.mark.scenarioTest

def test_scenarioBskMcScenarios(show_plots):
    # These need to be run in serial such that the data is produced for analysis
    scenarios = ['scenario_AttFeedbackMC',
                 'scenarioAnalyzeMonteCarlo']

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    for i, bskSimCase in enumerate(scenarios):
        # import the bskSim script to be tested
        scene_plt = importlib.import_module(bskSimCase)

        try:
            if i == 0:
                figureList = scene_plt.run(False)
            else:
                scene_plt.run()
                # Check if plots were generated
                plot_dir = os.path.join(path, "../../examples/MonteCarloExamples/saved_plots")
                doc_dir = os.path.join(path, "../../examples/MonteCarloExamples/docs")
                if not os.path.exists(plot_dir) or not os.listdir(plot_dir):
                    testFailCount += 1
                    testMessages.append("No plots were generated or saved.")
                if not os.path.exists(doc_dir) or not os.listdir(doc_dir):
                    testFailCount += 1
                    testMessages.append("No RST files were generated.")

        except Exception as err:
            testFailCount += 1
            testMessages.append(f"Error in {bskSimCase}: {str(err)}")

    # Clean up
    if os.path.exists(path + "/../../examples/MonteCarloExamples/scenario_AttFeedbackMC/"):
        shutil.rmtree(path + "/../../examples/MonteCarloExamples/scenario_AttFeedbackMC/")

    assert testFailCount < 1, testMessages
