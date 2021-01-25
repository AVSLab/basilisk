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
# Purpose:  This script calls a series of opNav related simulation tutorials to ensure
# that they complete properly.
# Author:   Thibaud Teil
# Creation Date:  Feb 7, 2020
#


import sys, os, inspect
import pytest
import importlib
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../examples/OpNavScenarios/scenariosOpNav')
sys.path.append(path + '/../examples/OpNavScenarios/')

r"""
Skip the following tests if all necessary modules do not exist
Requirements:
    - Vizard downloaded and app path set properly (in src/examples/OpNavScenarios/BSK_masters)
    - Basilisk built with ZMQ, protobuffers, and OpenCV 
"""

import BSK_OpNav
SimBase = BSK_OpNav.BSKSim(1, 1)
if os.path.exists(SimBase.vizPath) == False:
    pytestmark = pytest.mark.skip(reason="Vizard App not found: modify app in examples/OpNavScenarios/BSK_masters")

testScripts = [
      'scenario_faultDetOpNav'
    , 'scenario_OpNavAttOD'
    , 'scenario_OpNavAttODLimb'
    , 'scenario_OpNavHeading'
    , 'scenario_OpNavOD'
    , 'scenario_OpNavODLimb'
    , 'scenario_OpNavPoint'
    , 'scenario_OpNavPointLimb'
]


try:
    from Basilisk.simulation import vizInterface, camera
    from Basilisk.fswAlgorithms import houghCircles, limbFinding
except ImportError:
    pytestmark = pytest.mark.skip(reason="OpNav Algorithms not built: use opNav behavior in build")

try:
    from Basilisk.fswAlgorithms import centerRadiusCNN
    testScripts.append('scenario_CNNAttOD')
except ImportError:
    pass



# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("bskSimCase", testScripts)

@pytest.mark.scenarioTest

# if this script is run, it requires Vizard to be installed, and the pytest must be run single-threaded
# not with "-n 8" etc.
@pytest.mark.skip(reason="This test has special requirements to run.  See note in the script.")


def test_opnavBskScenarios(show_plots, bskSimCase):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    # import the bskSim script to be tested
    scene_plt = importlib.import_module(bskSimCase)

    try:
        figureList = scene_plt.run(False, 10)

        # save the figures to the Doxygen scenario images folder

        if(figureList != {}):
            for pltName, plt in list(figureList.items()):
                unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

