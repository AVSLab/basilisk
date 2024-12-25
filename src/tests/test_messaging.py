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
# Integrated tests
#
# Purpose:  This script runs a series of test to ensure that the messaging
# interface is properly set up for C and Cpp messages
# Author:   Benjamin Bercovici
# Creation Date:  June 4th, 2021
#


from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.moduleTemplates import cModuleTemplate
import numpy as np
import gc
import weakref


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# @pytest.mark.parametrize("bskScript", files)


def messaging_unit_tests():
    test_cpp_msg_subscription_check()
    test_c_msg_subscription_check()
    test_cpp_2_c_msg_subscription_check()
    test_c_2_cpp_msg_subscription_check()
    test_standalone_message_scope()
    test_standalone_message_multiple_readers()
    test_standalone_message_cpp_reader_cleanup()
    test_standalone_message_c_reader_cleanup()
    test_standalone_message_mixed_readers_cleanup()
    test_standalone_message_python_combinations()



def test_cpp_msg_subscription_check():
    # Check whether python-wrapped C++ messages are properly checked for subscription

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Try out all the existing cpp messages
    cppMessages = [ el for el in dir(messaging) if el.endswith("Msg")]

    for el in cppMessages:

        # Create three messages
        msgA = eval("messaging." + el + "()")
        msgB = eval("messaging." + el + "()")
        msgC = eval("messaging." + el + "()")

        # Create subscribers to pair messages
        msgB_subscriber = msgB.addSubscriber()
        msgC_subscriber = msgB.addSubscriber()

        # Subscribe
        msgB_subscriber.subscribeTo(msgA) # Subscribe B to A
        msgC_subscriber.subscribeTo(msgB) # Subscribe C to B

        # Check
        if msgB_subscriber.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgB_subscriber.isSubscribedTo(msgA) should be True")
        if msgC_subscriber.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgB) should be True")
        if msgC_subscriber.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgA) should be False")


        # Change subscription pattern
        msgB_subscriber.subscribeTo(msgC) # Subscribe B to C
        msgC_subscriber.subscribeTo(msgA) # Subscribe C to A

        # Check
        if msgB_subscriber.isSubscribedTo(msgC) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgB_subscriber.isSubscribedTo(msgC) should be True")
        if msgC_subscriber.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgA) should be True")
        if msgC_subscriber.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgB) should be False")



    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert(testFailCount == 0)






def test_c_msg_subscription_check():
    # Check whether python-wrapped C++ messages are properly checked for subscription

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Try out all the existing c messages
    cMessages = [ el for el in dir(messaging) if el.endswith("Msg_C")]


    for el in cMessages:

        # Create three messages
        msgA = eval("messaging." + el + "()")
        msgB = eval("messaging." + el + "()")
        msgC = eval("messaging." + el + "()")

        # Subscribe
        msgB.subscribeTo(msgA) # Subscribe B to A
        msgC.subscribeTo(msgB) # Subscribe C to B

        # Check
        if msgB.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgB.isSubscribedTo(msgA) should be True")
        if msgC.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgB) should be True")
        if msgC.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgA) should be False")


        # Change subscription pattern
        msgB.subscribeTo(msgC) # Subscribe B to C
        msgC.subscribeTo(msgA) # Subscribe C to A

        # Check
        if msgB.isSubscribedTo(msgC) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgB.isSubscribedTo(msgC) should be True")
        if msgC.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgA) should be True")
        if msgC.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgB) should be False")



    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert(testFailCount == 0)


def test_c_2_cpp_msg_subscription_check():

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Try out all the existing messages
    cppMessages = [ el for el in dir(messaging) if el.endswith("Msg")]
    cMessages = [ el for el in dir(messaging) if el.endswith("Msg_C")]

    # Find common messages
    common_messages = [el for el in cppMessages if el + "_C" in cMessages]


    for el in common_messages:

        # Create c and cpp messages
        msgA = eval("messaging." + el + "()")
        msgB = eval("messaging." + el + "()")

        msgC = eval("messaging." + el + "_C()")
        msgD = eval("messaging." + el + "_C()")


        # Subscribe
        msgC.subscribeTo(msgA) # Subscribe C to A
        msgD.subscribeTo(msgB) # Subscribe D to B

        # Check
        if msgC.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgA) should be True")
        if msgD.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgD.isSubscribedTo(msgB) should be True")
        if msgD.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgD.isSubscribedTo(msgA) should be False")
        if msgC.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgB) should be False")

        # Change subscription pattern
        msgC.subscribeTo(msgB) # Subscribe C to B
        msgD.subscribeTo(msgA) # Subscribe D to A

        # Check
        if msgC.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgB) should be True")
        if msgD.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgD.isSubscribedTo(msgA) should be True")
        if msgC.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC.isSubscribedTo(msgA) should be False")
        if msgD.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgD.isSubscribedTo(msgB) should be False")



    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert(testFailCount == 0)


def test_cpp_2_c_msg_subscription_check():
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Try out all the existing messages
    cppMessages = [ el for el in dir(messaging) if el.endswith("Msg")]
    cMessages = [ el for el in dir(messaging) if el.endswith("Msg_C")]

    # Find common messages
    common_messages = [el for el in cppMessages if el + "_C" in cMessages]


    for el in common_messages:

        # Create c and cpp messages
        msgA = eval("messaging." + el + "_C()")
        msgB = eval("messaging." + el + "_C()")

        msgC = eval("messaging." + el + "()")
        msgD = eval("messaging." + el + "()")

        # Create subscribers to pair messages
        msgC_subscriber = msgC.addSubscriber()
        msgD_subscriber = msgD.addSubscriber()

        # Subscribe
        msgC_subscriber.subscribeTo(msgA) # Subscribe C to A
        msgD_subscriber.subscribeTo(msgB) # Subscribe D to B

        # Check
        if msgC_subscriber.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgA) should be True")
        if msgD_subscriber.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgD_subscriber.isSubscribedTo(msgB) should be True")
        if msgD_subscriber.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgD_subscriber.isSubscribedTo(msgA) should be False")
        if msgC_subscriber.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgB) should be False")

        # Change subscription pattern
        msgC_subscriber.subscribeTo(msgB) # Subscribe C to B
        msgD_subscriber.subscribeTo(msgA) # Subscribe D to A

        # Check
        if msgC_subscriber.isSubscribedTo(msgB) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgB) should be True")
        if msgD_subscriber.isSubscribedTo(msgA) != 1:
            testFailCount += 1
            testMessages.append(el + ": msgD_subscriber.isSubscribedTo(msgA) should be True")
        if msgC_subscriber.isSubscribedTo(msgA) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgC_subscriber.isSubscribedTo(msgA) should be False")
        if msgD_subscriber.isSubscribedTo(msgB) != 0:
            testFailCount += 1
            testMessages.append(el + ": msgD_subscriber.isSubscribedTo(msgB) should be False")



    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert(testFailCount == 0)


def test_standalone_message_scope():
    """
    Test that subscribed messages don't get garbage collected when going out of scope
    """
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.ModelTag = "cppModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)

    # setup message recording
    msgRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", msgRec)

    # Create a local scope to test message lifetime
    def addLocalStandaloneMessage(module):
        """Create a stand-alone message in local scope and subscribe to it"""
        msgData = messaging.CModuleTemplateMsgPayload()
        msgData.dataVector = [10., 20., 30.]
        msg = messaging.CModuleTemplateMsg().write(msgData)
        module.dataInMsg.subscribeTo(msg)

    # Subscribe to the input message in a local scope
    addLocalStandaloneMessage(mod1)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(3.0))
    scSim.ExecuteSimulation()

    # Verify output matches expected values
    expected = np.array([
        [11., 20., 30.],
        [12., 20., 30.],
        [13., 20., 30.],
        [14., 20., 30.]
    ])

    if not (msgRec.dataVector == expected).all():
        testFailCount += 1
        testMessages.append("Output data does not match expected values")

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages


def test_standalone_message_multiple_readers():
    """Test standalone message with multiple readers - verify message stays alive until all readers gone"""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0
    testMessages = []

    # Create standalone message with data
    msgData = messaging.CModuleTemplateMsgPayload()
    msgData.dataVector = [1., 2., 3.]
    standalone_msg = messaging.CModuleTemplateMsg().write(msgData)

    # Create multiple modules to act as readers
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.ModelTag = "cppModule1"
    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"
    mod3 = cppModuleTemplate.CppModuleTemplate()
    mod3.ModelTag = "cppModule3"

    # Subscribe all modules to standalone message
    mod1.dataInMsg.subscribeTo(standalone_msg)
    mod2.dataInMsg.subscribeTo(standalone_msg)
    mod3.dataInMsg.subscribeTo(standalone_msg)

    # Verify all modules can read the message
    if not np.array_equal(mod1.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod1 failed to read standalone message")
    if not np.array_equal(mod2.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod2 failed to read standalone message")
    if not np.array_equal(mod3.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod3 failed to read standalone message")

    # Remove references one by one and verify remaining readers still work
    del mod1
    gc.collect()

    if not np.array_equal(mod2.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod2 failed after mod1 deletion")
    if not np.array_equal(mod3.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod3 failed after mod1 deletion")

    del mod2
    gc.collect()

    if not np.array_equal(mod3.dataInMsg().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("mod3 failed after mod2 deletion")

    # Final cleanup
    del mod3
    gc.collect()

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages

def test_standalone_message_cpp_reader_cleanup():
    """Test cleanup when both message and C++ reader go out of scope"""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0
    testMessages = []

    def create_and_subscribe():
        # Create message in local scope
        msgData = messaging.CModuleTemplateMsgPayload()
        msgData.dataVector = [1., 2., 3.]
        msg = messaging.CModuleTemplateMsg().write(msgData)
        msg_ref = weakref.ref(msg)

        # Create reader in local scope
        mod = cppModuleTemplate.CppModuleTemplate()
        mod.ModelTag = "cppModule"
        mod.dataInMsg.subscribeTo(msg)

        # Verify initial read
        if not np.array_equal(mod.dataInMsg().dataVector, msgData.dataVector):
            return False, None, None
        return True, msg_ref, mod  # Return the module too

    # Test in local scope
    success, msg_ref, mod = create_and_subscribe()
    if not success:
        testFailCount += 1
        testMessages.append("Failed to read message in local scope")

    # Delete message first, then module
    del msg_ref
    gc.collect()
    del mod
    gc.collect()

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages

def test_standalone_message_c_reader_cleanup():
    """Test cleanup when both message and C reader go out of scope"""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0
    testMessages = []

    def create_and_subscribe():
        # Create message in local scope
        msgData = messaging.CModuleTemplateMsgPayload()
        msgData.dataVector = [1., 2., 3.]
        msg = messaging.CModuleTemplateMsg_C().write(msgData)

        # Store weak reference to check cleanup
        msg_ref = weakref.ref(msg)

        # Create C reader in local scope
        mod = cModuleTemplate.cModuleTemplate()
        mod.ModelTag = "cModule"
        mod.dataInMsg.subscribeTo(msg)

        # Verify initial read
        readData = mod.dataInMsg.read()
        if not np.array_equal(readData.dataVector, msgData.dataVector):
            return False, None
        return True, msg_ref

    # Test in local scope
    success, msg_ref = create_and_subscribe()
    if not success:
        testFailCount += 1
        testMessages.append("Failed to read message in local scope")

    # Force cleanup
    gc.collect()

    # Verify message was actually cleaned up
    if msg_ref() is not None:
        testFailCount += 1
        testMessages.append("Message was not properly cleaned up")

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages

def test_standalone_message_mixed_readers_cleanup():
    """Test cleanup with mix of C++ and C readers"""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0
    testMessages = []

    def create_and_subscribe():
        # Create standalone message (using C-style message)
        msgData = messaging.CModuleTemplateMsgPayload()
        msgData.dataVector = [1., 2., 3.]
        standalone_msg = messaging.CModuleTemplateMsg_C().write(msgData)
        msg_ref = weakref.ref(standalone_msg)

        # Create mix of C++ and C readers
        cpp_mod = cppModuleTemplate.CppModuleTemplate()
        cpp_mod.ModelTag = "cppModule"
        c_mod = cModuleTemplate.cModuleTemplate()
        c_mod.ModelTag = "cModule"

        # Subscribe both types
        cpp_mod.dataInMsg.subscribeTo(standalone_msg)
        c_mod.dataInMsg.subscribeTo(standalone_msg)
        gc.collect()

        # Verify both can read
        if not np.array_equal(cpp_mod.dataInMsg().dataVector, msgData.dataVector):
            return False, None, None, None, None
        readData = c_mod.dataInMsg.read()
        if not np.array_equal(readData.dataVector, msgData.dataVector):
            return False, None, None, None, None

        return True, msg_ref, cpp_mod, c_mod, standalone_msg

    # Test in local scope
    success, msg_ref, cpp_mod, c_mod, standalone_msg = create_and_subscribe()
    if not success:
        testFailCount += 1
        testMessages.append("Failed to read message in local scope")

    # Cleanup in correct order (following pattern from test_standalone_message_python_combinations)
    del cpp_mod  # Delete C++ module first
    gc.collect()
    del c_mod    # Then delete C module
    gc.collect()
    del standalone_msg  # Delete message explicitly
    gc.collect()
    del msg_ref  # Finally delete reference
    gc.collect()

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages

def test_standalone_message_python_combinations():
    """Test combinations involving Python modules and standalone readers"""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    testFailCount = 0
    testMessages = []

    # Test C++ message with Python reader
    msgData = messaging.CModuleTemplateMsgPayload()
    msgData.dataVector = [1., 2., 3.]
    cpp_msg = messaging.CModuleTemplateMsg()
    cpp_msg.write(msgData)
    cpp_reader = cpp_msg.addSubscriber()
    msg_ref = weakref.ref(cpp_msg)
    gc.collect()

    # Verify C++ reader works
    if not np.array_equal(cpp_reader().dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("Python C++ reader failed initial read")

    # Keep message alive while reader is active
    readData = cpp_reader()
    if not np.array_equal(readData.dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("Python C++ reader failed after message deletion")

    # Cleanup in correct order
    del cpp_reader
    gc.collect()
    del cpp_msg
    gc.collect()

    # Test C message with Python reader
    c_msg = messaging.CModuleTemplateMsg_C()
    c_msg.write(msgData)
    c_reader = c_msg  # C messages are their own readers
    c_msg_ref = weakref.ref(c_msg)
    gc.collect()

    # Keep C message alive while testing reader
    readData = c_reader.read()
    if not np.array_equal(readData.dataVector, msgData.dataVector):
        testFailCount += 1
        testMessages.append("Python C reader failed initial read")

    # Cleanup C message (no need to delete cpp_reader again)
    del c_msg
    del c_reader
    gc.collect()

    # Verify C message cleanup
    if c_msg_ref() is not None:
        testFailCount += 1
        testMessages.append("C message was not properly cleaned up")

    if testFailCount == 0:
        print("PASSED")
    else:
        [print(msg) for msg in testMessages]
    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    messaging_unit_tests()
