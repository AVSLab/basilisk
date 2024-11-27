#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import inspect
import os
import sys
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioGaussMarkovRandomWalk

# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("processNoiseLevel, walkBounds", [
    (0.5, 3.0),  # Default case matching documentation and example
])
@pytest.mark.scenarioTest
def test_scenarioGaussMarkovRandomWalk(show_plots, processNoiseLevel, walkBounds):
    """This function is called by the py.test environment."""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioGaussMarkovRandomWalk.run(show_plots, processNoiseLevel, walkBounds)

        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

        # Also save the static layout diagram if it exists
        if hasattr(scenarioGaussMarkovRandomWalk, 'getStaticDiagram'):
            staticFig = scenarioGaussMarkovRandomWalk.getStaticDiagram()
            unitTestSupport.saveScenarioFigure('test_scenarioGaussMarkovRandomWalk', staticFig, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioGaussMarkovRandomWalk test failed")
        testMessages.append(str(err))

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages

if __name__ == "__main__":
    test_scenarioGaussMarkovRandomWalk(
        False,       # show_plots
        0.5,        # processNoiseLevel
        3.0         # walkBounds
    )
