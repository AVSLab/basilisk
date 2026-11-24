#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Integrated test of the spacecraft(), RWs, simpleNav() and mrpFeedback()
#           and boreAngCalc() modules.  Illustrates the compliance of several keep-in and
#           keep out constraints while performing a slew maneuver with RWs as actuators.
# Author:   Riccardo Calaon
# Creation Date:  Apr. 18, 2022
#


import inspect
import os
import sys

import numpy as np
import pytest
from Basilisk.utilities import simHelpers

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioAttitudeConstrainedManeuver


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parameterize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.


@pytest.mark.parametrize("use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase", [(False, 20, 70, 0),
                                                                                           (True,  20, 70, 1),
																						   (True,  20, 70, 2)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_bskAttitudeConstrainedManeuver(show_plots, use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioAttitudeConstrainedManeuver.run(show_plots, use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase)
        assert isinstance(figureList, dict)
        assert len(figureList) == 6
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            simHelpers.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioAttitudeConstrainedManeuver tests have failed.")

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
def test_distance_search_comparison(show_plots):
    """Run the alternate start and verify the path metrics and comparison figures."""
    figures, comparison = scenarioAttitudeConstrainedManeuver.run(
        show_plots, False, 20, 70, 4, compareSearchModes=True)  # Cone half-angles [deg]
    assert len(figures) == 8
    default = comparison["default"]
    shortest = comparison["shortest"]
    # These grid costs are independently validated against Dijkstra in the native tests.
    assert default["edgeDistance"] == pytest.approx(1.19731, abs=5e-6)  # [-]
    assert shortest["edgeDistance"] == pytest.approx(1.13077, abs=5e-6)  # [-]
    assert shortest["edgeDistance"] < 0.95 * default["edgeDistance"]
    assert default["expandedNodeCount"] == 12
    assert shortest["expandedNodeCount"] > default["expandedNodeCount"]
    expected_start = [-3.0 / 7.0, -2.0 / 7.0, 0.0]  # MRPs [-]
    expected_goal = [0.342, 0.223, -0.432]  # MRPs [-]
    for result in comparison.values():
        np.testing.assert_allclose(result["waypoints"][0], expected_start, atol=1e-14)
        np.testing.assert_allclose(result["waypoints"][-1], expected_goal, atol=1e-14)

    overlay = figures["scenarioAttitudeConstrainedManeuverSearchPaths020704"]
    path_lines = overlay.axes[0].lines
    assert len(path_lines) == 3  # The shortest path has a shadow switch and two plotted segments.
    for line in path_lines:
        points = np.array(line.get_data_3d()).T
        assert np.all(np.linalg.norm(points, axis=1) <= 1.0 + 1e-12)
        assert np.all(np.linalg.norm(np.diff(points, axis=0), axis=1) < 0.5)  # [-] No antipodal jump
    table_figure = figures["scenarioAttitudeConstrainedManeuverSearchComparison020704"]
    cells = table_figure.axes[0].tables[0].get_celld()
    for row, result in enumerate((default, shortest), start=1):
        assert cells[row, 1].get_text().get_text() == f'{result["edgeDistance"]:.5f}'
        assert cells[row, 2].get_text().get_text() == str(result["expandedNodeCount"])
    for name, figure in figures.items():
        simHelpers.saveScenarioFigure(name, figure, path)


@pytest.mark.scenarioTest
def test_search_comparison_figure_names():
    """Keep both modes' figures when saving results for the same attitude case."""
    default_figures = scenarioAttitudeConstrainedManeuver.run(
        False, False, 20, 70, 4)  # Cone half-angles [deg]
    comparison_figures, _ = scenarioAttitudeConstrainedManeuver.run(
        False, False, 20, 70, 4, compareSearchModes=True)  # Cone half-angles [deg]
    expected_default_names = {f"scenarioAttitudeConstrainedManeuver{index}020704"
                              for index in range(1, 7)}
    assert set(default_figures) == expected_default_names
    assert set(default_figures).isdisjoint(comparison_figures)
    assert len(default_figures | comparison_figures) == 14


def test_mrp_overlay_shadow_switch():
    """Map an exterior waypoint into the unit sphere without drawing a shadow jump."""
    waypoints = np.array([[0.0, 0.0, 0.9], [0.0, 0.0, 1.05], [0.0, 0.0, -0.9]])  # MRPs [-]
    original = waypoints.copy()
    principal, segments, switches = scenarioAttitudeConstrainedManeuver._principal_path_segments(waypoints)
    expected = np.array([[0.0, 0.0, 0.9], [0.0, 0.0, -1.0 / 1.05], [0.0, 0.0, -0.9]])  # MRPs [-]
    np.testing.assert_allclose(principal, expected, atol=1e-14)
    assert [len(segment) for segment in segments] == [1, 2]
    np.testing.assert_allclose(switches, expected[:2], atol=1e-14)
    np.testing.assert_array_equal(waypoints, original)
