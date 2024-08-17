#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Test if a C-wrapped input message can be logged with a recorder module and that a C++ module can access its data using non-swig modules
# Author: Scott Piggott
# Creation Date:  Aug. 17 2024
#


from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.architecture import sim_model
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport as uts


def test_RecordingInputMessages():
    """
    testing recording a C-wrapped input message with the recorder module
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplate()
    mod1.ModelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)

    # Write input data
    inputData = messaging.CModuleTemplateMsgPayload()
    inputData.dataVector = [1, 2, 3]
    inputDataMsg = messaging.CModuleTemplateMsg().write(inputData)

    # Subscribe input message to stand alone message
    inputAddr = sim_model.getObjectAddress(inputDataMsg)
    mod1.dataInMsg.subscribeTo(inputAddr)

    # Create recorders tied to IO messages
    dataInRec = mod1.dataInMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataInRec)
    dataOutRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataOutRec)

    attGuidMsg = messaging.CModuleTemplateMsg_C()
    attGuidMsgPayload = messaging.CModuleTemplateMsgPayload()
    attGuidMsg.write(attGuidMsgPayload)
    messaging.CModuleTemplateMsg_C_addAuthor(mod1.dataOutMsg, attGuidMsg)
    dataOut2Rec = attGuidMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataOut2Rec)

    # create modules
    mod1CPP = cppModuleTemplate.CppModuleTemplate()
    mod1CPP.ModelTag = "cppModule1"
    scSim.AddModelToTask("dynamicsTask", mod1CPP)

    # Subscribe input message to C-module output message address
    cOutAddr = sim_model.getObjectAddress(mod1.dataOutMsg)
    mod1CPP.dataInMsg.subscribeToCAddr(cOutAddr)

    # Create recorders tied to IO messages
    dataInRecCPP = mod1CPP.dataInMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataInRecCPP)
    dataOutRecCPP = mod1CPP.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataOutRecCPP)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    # reading the module output message show not change the earlier redirection
    # further, we are testing that the read() command copies the payload from
    # the stand alone msg to the module output module
    tempSet = mod1.dataOutMsg.read().dataVector
    scSim.ConfigureStopTime(macros.sec2nano(2.0))
    scSim.ExecuteSimulation()

    # print(dataInRec.dataVector)
    # print(dataOutRec.dataVector)
    # print(dataOut2Rec.dataVector)

    testFailCount, testMessages = uts.compareArray([inputData.dataVector]*3
                                                   , dataInRec.dataVector
                                                   , 0.01
                                                   , "recorded input message was not correct."
                                                   , testFailCount
                                                   , testMessages)

    testFailCount, testMessages = uts.compareArray([[0, 0, 0], [0, 0, 0], [3, 2, 3]]
                                                   , dataOutRec.dataVector
                                                   , 0.01
                                                   , "recorded module output message was not correct."
                                                   , testFailCount
                                                   , testMessages)

    testFailCount, testMessages = uts.compareArray([[2, 2, 3], [3, 2, 3], [4, 2, 3]]
                                                   , dataOut2Rec.dataVector
                                                   , 0.01
                                                   , "recorded redirected module output message was not correct."
                                                   , testFailCount
                                                   , testMessages)

    testFailCount, testMessages = uts.compareArray([[4., 2., 3.]]
                                                   , [mod1.dataOutMsg.read().dataVector]
                                                   , 0.01
                                                   , "read of module output message was not correct."
                                                   , testFailCount
                                                   , testMessages)

    testFailCount, testMessages = uts.compareArray([[4, 2, 3]]
                                                   , [attGuidMsg.read().dataVector]
                                                   , 0.01
                                                   , "read of module redirected output message was not correct."
                                                   , testFailCount
                                                   , testMessages)
    #Check that the input message subscribed with address reflects data right
    testFailCount, testMessages = uts.compareArray(dataOutRec.dataVector
                                                   , dataInRecCPP.dataVector
                                                   , 0.01
                                                   , "recorded input message from addr was not correct."
                                                   , testFailCount
                                                   , testMessages)
    dataRecExpectation = dataInRecCPP.dataVector
    dummyVal = 1
    for i in range(dataRecExpectation.shape[0]):
        dataRecExpectation[i, 0] += dummyVal
        dummyVal += 1

    #Check that the output message using input subscribed with address is right
    testFailCount, testMessages = uts.compareArray(dataRecExpectation
                                                   , dataOutRecCPP.dataVector
                                                   , 0.01
                                                   , "recorded output message from addr was not correct."
                                                   , testFailCount
                                                   , testMessages)

    if testFailCount:
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_RecordingInputMessages()
