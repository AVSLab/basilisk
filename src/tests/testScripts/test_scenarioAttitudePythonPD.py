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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           a Python implementation of the MRP_PD module.  Illustrates
#           a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.  The main
#           purpose of this module is to illustrate how to use python processes.
# Author:   Scott Piggott
# Creation Date:  Jun. 28, 2017
#

import pytest
import os, sys
import inspect
import numpy as np

from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudePythonPD


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useJitterSimple, useRWVoltageIO", [(False, False)])
def test_bskAttitudeFeedbackPD(show_plots, useJitterSimple, useRWVoltageIO):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataSigmaBR, dataUsReq, dataSigmaBRBase, dataUsReqBase, figureList = \
        scenarioAttitudePythonPD.runRegularTask(show_plots, useJitterSimple, useRWVoltageIO)



    dataSigDiff = dataSigmaBR[:, 1:4] - dataSigmaBRBase[:, 1:4]
    if np.linalg.norm(dataSigDiff) > 1.0E-10:
        testFailCount += 1
        testMessages.append("Failed to get accurate agreement between attitude variables")

    dataUsDiff = dataUsReq[:, 1:4] - dataUsReqBase[:, 1:4]
    if np.linalg.norm(dataUsDiff) > 1.0E-10:
        testFailCount += 1
        testMessages.append("Failed to get accurate agreement between torque command variables")

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in figureList.items():
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages

