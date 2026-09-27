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
# Integrated tests
#
# Purpose:  This script calls a series of quick start guide demonstration scripts to ensure
# that they complete properly.
# Author:   Hanspeter Schaub
# Creation Date:  Feb. 3, 2021
#


import fnmatch
import importlib
import inspect
import os
import sys

import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../docs/source/codeSamples')
files = fnmatch.filter(os.listdir(path + '/../../docs/source/codeSamples'), "*.py")

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("bskScript", files)
@pytest.mark.scenarioTest
def test_scenarioBskPrinciples(show_plots, bskScript):
    """Run each documentation code sample included in the BSK principles set."""

    if bskScript == "making-numbaModules.py":
        pytest.importorskip("numba")

    bskLogging.setDefaultLogLevel(bskLogging.WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    # import the bskSim script to be tested
    scene_plt = importlib.import_module(os.path.splitext(bskScript)[0])

    try:
        figureList = scene_plt.run()

        # save the figures to the RST scenario images folder
        if(figureList and figureList != {}):
            for pltName, plt in list(figureList.items()):
                simHelpers.saveScenarioFigure(pltName, plt, path)
    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages


def test_effector_state_naming_sample():
    """Repeated builds preserve custom names and retrieve the recorded states."""
    sample = importlib.import_module("bsk-multiSim")
    results = sample.run_cases()
    automatic_first, automatic_second, custom_first, custom_second = results

    automatic_names = [
        {name for panel_names in result["names"] for name in panel_names}
        for result in (automatic_first, automatic_second)
    ]
    assert automatic_names[0].isdisjoint(automatic_names[1])
    expected = (("leftPanelAngle", "leftPanelRate"), ("rightPanelAngle", "rightPanelRate"))
    assert custom_first["names"] == custom_second["names"] == expected
    for result in results:
        assert result["names_before"] == result["names"]
        assert len({name for pair in result["names"] for name in pair}) == 4
        assert np.all(np.isfinite(result["states"]))
        np.testing.assert_allclose(result["states"], result["messages"], rtol=0.0, atol=1e-13)
        np.testing.assert_allclose(result["states"], automatic_first["states"], rtol=0.0, atol=1e-13)
        initial_angles = np.array([0.1, 0.2])  # [rad]
        assert np.all(np.abs(result["states"][:, 0] - initial_angles) > 1e-4)  # [rad]


if __name__ == "__main__":
    test_scenarioBskPrinciples(
        False,        # show_plots
        'bsk-4'
    )
