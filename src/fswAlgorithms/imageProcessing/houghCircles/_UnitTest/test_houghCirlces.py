''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
#
#   Unit Test Script
#   Module Name:        HoughCirlces
#   Author:             Thibaud Teil
#   Creation Date:      March 13, 2019
#
from PIL import Image, ImageDraw
from math import sqrt, pi, cos, sin
from cannyDetection import canny_edge_detector
from collections import defaultdict

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass, unitTestSupport
from Basilisk.utilities import macros
from Basilisk.simulation import sim_model

try:
    from Basilisk.fswAlgorithms import houghCircles
except ImportError:
    pytest.skip("Hough Circles not built---check OpenCV option")

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.


# update "module" in this function name to reflect the module name
def test_module(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = houghCirclesTest(show_plots)
    assert testResults < 1, testMessage


def houghCirclesTest(show_plots):


    # Truth values from python
    input_image = Image.open("circles.png")
    #################################################

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    bitmapArray = []

    # # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = houghCircles.HoughCircles()
    moduleConfig.ModelTag = "houghCircles"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    moduleConfig.imageInMsgName = "sample_image"
    moduleConfig.opnavCirclesOutMsgName = "circles"
    pointerToStuff = sim_model.ConstCharVector()
     # = sim_model.new_cByteArray(len(hex(int(id(input_image)))))
    pointerLength = len(hex(int(id(input_image))))
    print hex(int(id(input_image)))
    for i in range(pointerLength):
        pointerToStuff.append(hex(int(id(input_image)))[i])
        # sim_model.cByteArray_setitem(pointerToStuff, i, int(hex(int(id(input_image)))[i], 16))
    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = houghCircles.CameraImageMsg()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inputMessageData.imagePointer = pointerToStuff
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.imageInMsgName,
                               inputMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.imageInMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    pointer = unitTestSim.pullMessageLogData(moduleConfig.imageInMsgName + ".imagePointer", range(pointerLength))
    centers = unitTestSim.pullMessageLogData(moduleConfig.opnavCirclesOutMsgName + ".circlesCenters", range(3*2))
    radii = unitTestSim.pullMessageLogData(moduleConfig.opnavCirclesOutMsgName + ".circlesRadii", range(3 * 2))

    # Output image:
    output_image = Image.new("RGB", input_image.size)
    output_image.paste(input_image)
    draw_result = ImageDraw.Draw(output_image)

    # Find circles
    rmin = 5
    rmax = int(input_image.size[0]/2)
    steps = 100
    threshold = 0.4

    points = []
    for r in range(rmin, rmax + 1):
        for t in range(steps):
            points.append((r, int(r * cos(2 * pi * t / steps)), int(r * sin(2 * pi * t / steps))))

    acc = defaultdict(int)
    for x, y in canny_edge_detector(input_image):
        for r, dx, dy in points:
            a = x - dx
            b = y - dy
            acc[(a, b, r)] += 1

    circles = []
    for k, v in sorted(acc.items(), key=lambda i: -i[1]):
        x, y, r = k
        if float(v) / steps >= threshold and all((x - xc) ** 2 + (y - yc) ** 2 > rc ** 2 for xc, yc, rc in circles):
            circles.append((x, y, r))

    for x, y, r in circles:
        draw_result.ellipse((x - r, y - r, x + r, y + r), outline=(255, 0, 0, 0))

    # Save output image
    output_image.save("result.png")


    #   print out success message if no error were found
    # snippentName = "passFail"
    # if testFailCount == 0:
    #     colorText = 'ForestGreen'
    #     print "PASSED: " + moduleWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    # else:
    #     colorText = 'Red'
    #     print "Failed: " + moduleWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    # unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    houghCirclesTest(
                 False
               )
