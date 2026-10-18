
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.




#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer that reorbits a debris using electrostatic forces.
# Author:   Julian Hammerl
# Creation Date:  May 20, 2021
#

import inspect
import os
import sys
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.utilities import orbitalMotion, simHelpers

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioDebrisReorbitET


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.scenarioTest

def test_scenarioDebrisReorbitET(show_plots):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioDebrisReorbitET.run(False)

        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("Basic Formation Flying tutorial failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages


def test_debris_orbit_plot_with_rv2elem_axis():
    """Preserve plotting with NumPy scalar elements returned by ``rv2elem``."""
    mu = 3.986004415e14  # [m^3/s^2]
    initial_axis = 7.0e6  # [m]
    axes = initial_axis + np.array([0.0, 100.0, 250.0])  # [m]
    positions = np.zeros((len(axes), 3))
    velocities = np.zeros_like(positions)
    positions[:, 0] = axes
    velocities[:, 1] = np.sqrt(mu / axes)
    # The scenario obtains its reference elements from the initial Cartesian state.
    elements = orbitalMotion.rv2elem(mu, positions[0], velocities[0])
    assert type(elements.a) is np.float64
    time = np.array([0, 1000000000, 2000000000], dtype=np.uint64)  # [ns]
    period = 2.0 * np.pi * np.sqrt(elements.a**3 / mu)  # [s]
    planet = SimpleNamespace(radEquator=6378136.6)  # [m]
    try:
        figures = scenarioDebrisReorbitET.plotOrbits(
            time, positions, velocities, positions, velocities, elements, mu, period, planet
        )
        axis_plot = figures[scenarioDebrisReorbitET.fileName + "3"].axes[0].lines[0]
        np.testing.assert_allclose(axis_plot.get_ydata(), [0.0, 0.1, 0.25], rtol=0.0, atol=5e-12)  # [km]
    finally:
        plt.close("all")


if __name__ == "__main__":
    test_scenarioDebrisReorbitET(
        False        # show_plots
    )
