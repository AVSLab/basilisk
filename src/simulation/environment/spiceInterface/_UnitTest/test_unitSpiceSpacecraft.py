
# ISC License
#
# Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import inspect
import os

import numpy as np

#
# Spice Unit Test
#
# Purpose:  Test the proper function of the Spice Ephemeris module for spacecraft information.
#           Proper function is tested by comparing Spice Ephemeris to
#           JPL Horizons Database for different planets and times of year
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 17, 2021
#

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import macros

# provide a unique test method name, starting with test_
def test_unitSpiceSc(show_plots):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSpiceSc(show_plots)
    assert testResults < 1, testMessage


# Run unit test
def unitSpiceSc(show_plots):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))
    dateSpice = "2015 February 10, 00:00:00.0 TDB"

    # Initialize the spice modules that we are using.
    spiceObject = spiceInterface.SpiceInterface()
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    scNames = ["HUBBLE SPACE TELESCOPE"]
    spiceObject.addSpacecraftNames(spiceInterface.StringVector(scNames))
    spiceObject.UTCCalInit = dateSpice
    spiceObject.zeroBase = "earth"
    spiceObject.loadSpiceKernel("hst_edited.bsp", bskPath + '/supportData/EphemerisData/')

    TotalSim.AddModelToTask(unitTaskName, spiceObject)

    # Configure simulation
    TotalSim.ConfigureStopTime(macros.sec2nano(0.1))

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    # unload spice kernel
    spiceObject.unloadSpiceKernel("hst_edited.bsp", bskPath + '/supportData/EphemerisData/')

    # set truth
    truthPosition = np.array([-5855529.540348052, 1986110.860522791, -3116764.7117067943])
    truthVelocity = np.array([-1848.9038338503085, -7268.515626753905, -1155.3578832725618])
    truthAtt = np.array([0., 0., 0.])
    truthZero = np.array([0., 0., 0.])

    scStateMsg = spiceObject.scStateOutMsgs[0].read()
    # print(scStateMsg.r_BN_N)
    # print(scStateMsg.v_BN_N)
    # print(scStateMsg.sigma_BN)
    accuracy = 0.01
    testFailCount, testMessages = unitTestSupport.compareVector(truthPosition,
                                                                scStateMsg.r_BN_N,
                                                                accuracy, "scState-r_BN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthPosition,
                                                                scStateMsg.r_CN_N,
                                                                accuracy, "scState-r_CN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthVelocity,
                                                                scStateMsg.v_BN_N,
                                                                accuracy, "scState-v_BN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthVelocity,
                                                                scStateMsg.v_CN_N,
                                                                accuracy, "scState-v_CN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthAtt,
                                                                scStateMsg.sigma_BN,
                                                                accuracy, "scState-sigma_BN",
                                                                testFailCount, testMessages)
    attStateMsg = spiceObject.attRefStateOutMsgs[0].read()
    testFailCount, testMessages = unitTestSupport.compareVector(truthAtt,
                                                                attStateMsg.sigma_RN,
                                                                accuracy, "scState-sigma_RN",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthZero,
                                                                attStateMsg.omega_RN_N,
                                                                accuracy, "scState-omega_RN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthZero,
                                                                attStateMsg.domega_RN_N,
                                                                accuracy, "scState-domega_RN_N",
                                                                testFailCount, testMessages)

    transStateMsg = spiceObject.transRefStateOutMsgs[0].read()
    testFailCount, testMessages = unitTestSupport.compareVector(truthPosition,
                                                                transStateMsg.r_RN_N,
                                                                accuracy, "scState-r_RN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthVelocity,
                                                                transStateMsg.v_RN_N,
                                                                accuracy, "scState-v_RN_N",
                                                                testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(truthZero,
                                                                transStateMsg.a_RN_N,
                                                                accuracy, "scState-a_RN_N",
                                                                testFailCount, testMessages)


    # print out success message if no error were found
    if testFailCount == 0:
        print(" \n PASSED ")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSpiceSc(
                     False  # show_plots
                     )
