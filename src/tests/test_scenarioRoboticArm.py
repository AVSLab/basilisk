# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Integrated test of the scenarioRoboticArm. Verifies the robotic arm
#           joints reach their commanded angles. This guards against issue #1107,
#           where the standalone reference messages were garbage collected and all
#           joint angles stayed at zero.
#


import inspect
import os
import sys

import numpy as np
import pytest
from Basilisk.utilities import macros

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioRoboticArm


@pytest.mark.scenarioTest
def test_scenarioRoboticArm(show_plots):
    """Run the scenario and assert each joint reaches its commanded angle."""

    testFailCount = 0
    testMessages = []

    # commanded joint angles set in the scenario (deg)
    commandedDeg = [-30.0, 45.0, 90.0, -20.0]  # deg

    # capture the joint-angle recorders without rendering plots
    captured = {}

    def capturePlotting(showPlots, scLog, thetaLog):
        captured["thetaLog"] = thetaLog
        return {}

    originalPlotting = scenarioRoboticArm.plotting
    scenarioRoboticArm.plotting = capturePlotting
    try:
        scenarioRoboticArm.run(False)
    finally:
        scenarioRoboticArm.plotting = originalPlotting

    thetaLog = captured.get("thetaLog", [])
    if len(thetaLog) != len(commandedDeg):
        testFailCount += 1
        testMessages.append(
            f"Expected {len(commandedDeg)} joint logs, got {len(thetaLog)}."
        )

    for idx, recorder in enumerate(thetaLog):
        finalDeg = np.array(recorder.theta)[-1] * macros.R2D
        if not np.isfinite(finalDeg) or abs(finalDeg - commandedDeg[idx]) > 0.5:
            testFailCount += 1
            testMessages.append(
                f"Joint {idx + 1} final angle {finalDeg:.3f} deg did not reach "
                f"commanded {commandedDeg[idx]:.1f} deg."
            )

    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_scenarioRoboticArm(False)
