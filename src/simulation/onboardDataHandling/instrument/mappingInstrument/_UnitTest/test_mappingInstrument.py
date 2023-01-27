# 
#  ISC License
# 
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import mappingInstrument
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_mappingInstrument():
    r"""
    This test checks that both the name of the data and the baudRate are correctly set in the mapping instrument module.

    In this test, two accessMsgs are instantiated to test the module. In the first accessMsg, hasAccess = 1, meaning
    that the point is accessible. This point is named '1.' In the second accessMsg, hasAccess = 0, meaning that the
    point is not accessible. The point is named 'data2' to test longer dataNames. The mappingInstrument should output
    a baudRate of 1 for point one, and a baudRate of 0 for point two. The dataNames in the output message are also
    checked.
    """
    [testResults, testMessage] = mappingInstrumentTestFunction()
    assert testResults < 1, testMessage


def mappingInstrumentTestFunction():
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = mappingInstrument.MappingInstrument()
    module.ModelTag = "mappingInstrumentTag"
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.nodeBaudRate = 1.

    # Configure blank module input messages
    accessInMsgData1 = messaging.AccessMsgPayload()
    accessInMsgData1.hasAccess = 1
    accessInMsg1 = messaging.AccessMsg().write(accessInMsgData1)

    accessInMsgData2 = messaging.AccessMsgPayload()
    accessInMsgData2.hasAccess = 0
    accessInMsg2 = messaging.AccessMsg().write(accessInMsgData2)

    # subscribe input messages to module
    module.addMappingPoint(accessInMsg1, '1')
    module.addMappingPoint(accessInMsg2, 'data2')

    # setup output message recorder objects
    dataLogs = []
    for idx in range(0, 2):
        dataLogs.append(module.dataNodeOutMsgs[idx].recorder())
        unitTestSim.AddModelToTask(unitTaskName, dataLogs[idx])

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    dataAmt = []
    dataNames = []
    for idx in range(0, 2):
        dataNames.append(dataLogs[idx].dataName)
        dataAmt.append(dataLogs[idx].baudRate)

    dataAmt = np.array(dataAmt)
    dataNames = np.array(dataNames)

    if not np.array_equal(dataAmt[0,:], np.array([1., 1., 1.])):
        testFailCount += 1

    if not np.array_equal(dataAmt[1,:], np.array([0., 0., 0.])):
        testFailCount += 1

    if not np.array_equal(dataNames[0,:], np.array(['1', '1', '1'])):
        testFailCount += 1

    if not np.array_equal(dataNames[1,:], np.array(['data2', 'data2', 'data2'])):
        testFailCount += 1

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_mappingInstrument()


