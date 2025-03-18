import pytest
import numpy as np
import zmq, json
from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport # general support file with common unit test functions
import ROS2Handler

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["test_ROS2Handler"
                                    #   , "extForceInertialAndTorque"
                                      ])

def test_ROS2HandlerAllTest(function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage
    
def test_ROS2Handler(test_rate = 0.1, sim_time = 20.):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(test_rate)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Include test module:
    module = ROS2Handler.ROS2Handler()
    module.ModelTag = "ros2Handler"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    
    scStateInData = messaging.SCStatesMsgPayload()
    scStateInData.r_BN_N = [1000., 100000., -10000.]
    scStateInData.v_BN_N = [200., 300., -400.]
    scStateInData.sigma_BN = [0.,0.,0.]
    scStateInData.omega_BN_B = [1.,-2.,3.]
    
    scStateInMsg = messaging.SCStatesMsg().write(scStateInData)
    
    module.scStateInMsg.subscribeTo(scStateInMsg)

    cmdMsgData = {
        "time": 0.,
        "Fcmd": [0.1, 2.0, -4.0],
        "lrCmd": [2.5, -3.0, 0.05],
    }

    cmdMsgDataJSON = json.dumps(cmdMsgData)

    # TODO modify in test
    context, sub_socket, pub_socket = __set_fake_bridge_send_receive_sockets(module)
    # 1) Sub socket listens to ROS2Handler.send_socket for fake JSON data
    try: 
        sub_socket.recv_string(flags=zmq.NOBLOCK)
    except zmq.Again:
        # time.sleep(0.1)  # Avoid busy-waiting
        pass # No new messages, continue loop    
    # 2) Pub socket publish fake JSON data to ROS2Handler.receive_socket
    pub_socket.send_string(cmdMsgDataJSON) # TODO make this to publish data during the BSK sim.
    
    # Start simulation:
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()
    
    # Test pass/fail conditions:
    if testFailCount == 0:
        print("PASSED: " + " External Body Force and Torque Inegrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]
    
def __set_fake_bridge_send_receive_sockets(module):
    context = zmq.Context()
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://localhost:{module.send_port}")  # Adjust as needed
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    sub_socket.setsockopt(zmq.CONFLATE, 1)  # Always latest data    

    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"tcp://*:{module.receive_port}")  # Adjust as needed
    
    return context, sub_socket, pub_socket   
    
if __name__ == "__main__":
    test_ROS2Handler(
        test_rate=0.1, # Set custom test rate for testing ROS2-BSK synchronization/time delay.
        sim_time = 20. # Set custom test sim. time.
    )