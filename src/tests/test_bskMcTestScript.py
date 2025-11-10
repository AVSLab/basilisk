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
# Creation Date: Nov 18, 2019
# Recenlty updated: Nov 4, 2024
#


import importlib
import inspect
import os
import shutil
import sys
import pytest
import json
import ast
import numpy as np

# Check if Bokeh is available
bokeh_spec = importlib.util.find_spec("bokeh")
bokeh_available = bokeh_spec is not None

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/MonteCarloExamples')

# Skip test if Python version is less than 3.9
@pytest.mark.skipif(sys.version_info < (3, 9),
                    reason="Test has issues with Controller class and older python.")

# Skip test if Bokeh is not available
@pytest.mark.skipif(not bokeh_available,
                    reason="Bokeh is not available. Skipping test.")
@pytest.mark.slowtest
@pytest.mark.scenarioTest

def test_scenarioBskMcScenarios(show_plots):
    # These need to be run in serial such that the data is produced for analysis
    scenarios = ['scenarioBskSimAttFeedbackMC',
                 'scenarioVisualizeMonteCarlo']

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

        except Exception as err:
            testFailCount += 1
            testMessages.append(f"Error in {bskSimCase}: {str(err)}")

    # Clean up
    if os.path.exists(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/"):
        shutil.rmtree(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/")

    assert testFailCount < 1, testMessages

def test_dispersionApplicationMc():
    testMessages = []

    scene_plt_dispersions = importlib.import_module('scenarioBskSimAttFeedbackMC')

    try:
        figureList = scene_plt_dispersions.run(False, recordSimParams=True)

    except Exception as err:
        testFailCount += 1
        testMessages.append(f"Error in {'scenarioBskSimAttFeedbackMC'}: {str(err)}")

    # check path existence
    assert os.path.exists(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/")

    # define file paths
    dispPath = os.path.join(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/run0.json")
    attrPath = os.path.join(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/run0attributes.json")

    with open(dispPath, 'r') as f:
        dispData = json.load(f)
    with open(attrPath, 'r') as f:
        attrData = json.load(f)

    # check if keys are identical
    dispDataKeys = set(dispData.keys())
    attrDataKeys = set(attrData.keys())

    assert dispDataKeys == attrDataKeys, "Key sets differ"

    for key in dispDataKeys:

        dispVal = ast.literal_eval(dispData[key])

        if type(dispVal) == list:
            arrayDisp = np.array(dispVal).flatten()
            attrVal = ast.literal_eval(attrData[key])
            arrayAttr = np.array(attrVal).flatten()

            np.testing.assert_allclose(
                arrayAttr,
                arrayDisp,
                atol=1e-12,
                err_msg=f"Numerical mismatch for parameter: {key} in the first simulation run"
            )

        else:
            dispVal = dispData[key]
            attrVal = attrData[key]

            assert dispVal == attrVal, (
                f"Mismatch for parameter: {key} in the first simulation run. "
                f"Expected: '{dispVal}', Found: '{attrVal}'"
            )

    # Clean up
    if os.path.exists(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/"):
        shutil.rmtree(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/")
