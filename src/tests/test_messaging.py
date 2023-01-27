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



if __name__ == "__main__":
    messaging_unit_tests()
