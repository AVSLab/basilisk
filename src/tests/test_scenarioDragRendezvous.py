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

import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioDragRendezvous as sdr

@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_scenarioDragRendezvous(show_plots):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = sdr.run(False, # show_plots
                            0.0, #   altitude offset (m)
                            0.1, #  True anomaly offset (deg)
                            1, #    Density multiplier (nondimensional)
                            ctrlType='lqr',
                            useJ2=False)

        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("Drag rendezvous scenario failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages


@pytest.mark.scenarioTest
def test_scenario_drag_rendezvous_j2_no_wind(capfd):
    """Verify the J2/no-wind configuration runs without a planet-orientation warning."""
    results = sdr.drag_simulator(
        0.0,  # [m] altitude offset
        0.1,  # [deg] true anomaly offset
        1,  # [-] density multiplier
        ctrlType='lqr',
        useJ2=True,
        useWind=False,
    )

    out, _ = capfd.readouterr()
    assert len(results["dynTimeData"]) > 0
    assert "no planet-orientation message" not in out
