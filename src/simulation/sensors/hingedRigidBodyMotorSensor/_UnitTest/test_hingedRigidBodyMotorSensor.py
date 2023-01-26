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

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import hingedRigidBodyMotorSensor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("thetaNoiseStd, thetaDotNoiseStd, accuracy", [(0.0, 0.0, 1.0e-12)])
@pytest.mark.parametrize("thetaBias, thetaDotBias",[(0,0), (-.1,.1), (.2,-.01)])
@pytest.mark.parametrize("thetaLSB, thetaDotLSB",[(-1,-1), (.1,.05), (.01,.005)])
@pytest.mark.parametrize("trueTheta, trueThetaDot",[(1.01,-0.23), (-.229,.112)])

def test_hingedRigidBodyMotorSensor(show_plots, thetaNoiseStd, thetaDotNoiseStd, thetaBias, thetaDotBias, thetaLSB, thetaDotLSB, trueTheta, trueThetaDot, accuracy):
    r"""
    **Validation Test Description**

    This test checks the sensor's addition of a bias and discretization to the input value.

    **Test Parameters**

    Args:
        thetaNoiseStd (double): standard deviation of the added noise to theta
        thetaDotNoiseStd (double): standard deviation of the added noise to thetaDot
        thetaBias (double): bias added to theta
        thetaDotBias (double): bias added to thetaDot
        thetaLSB (double): least significant bit for discretizing theta. Negative means no discretization.
        thetaDotLSB (double): least significant bit for discretizing thetaDot. Negative means no discretization.
        trueTheta (double): true value of theta
        trueThetaDot (double): true value of thetaDot
        accuracy (double): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**
    
    The python evaluated sensed value is compared against the module output.

    """
    [testResults, testMessage] = hingedRigidBodyMotorSensorTestFunction(show_plots, thetaNoiseStd, thetaDotNoiseStd, thetaBias, thetaDotBias, thetaLSB, thetaDotLSB, trueTheta, trueThetaDot, accuracy)
    assert testResults < 1, testMessage


def hingedRigidBodyMotorSensorTestFunction(show_plots, thetaNoiseStd, thetaDotNoiseStd, thetaBias, thetaDotBias, thetaLSB, thetaDotLSB, trueTheta, trueThetaDot, accuracy):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    
    timeStep = 0.5
    totalTime = 10.0

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(timeStep)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = hingedRigidBodyMotorSensor.HingedRigidBodyMotorSensor()
    module.ModelTag = "hingedRigidBodyMotorSensorTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    hingedRigidBodyMotorSensorInMsgData = messaging.HingedRigidBodyMsgPayload()
    
    # set up fake input message
    hingedRigidBodyMotorSensorInMsgData.theta = trueTheta;
    hingedRigidBodyMotorSensorInMsgData.thetaDot = trueThetaDot;

    hingedRigidBodyMotorSensorInMsg = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMotorSensorInMsgData)

    # subscribe input messages to module
    module.hingedRigidBodyMotorSensorInMsg.subscribeTo(hingedRigidBodyMotorSensorInMsg)

    # set up output message recorder objects
    dataLog = module.hingedRigidBodyMotorSensorOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    
    # set up variables in sensor
    module.thetaNoiseStd = thetaNoiseStd
    module.thetaDotNoiseStd = thetaDotNoiseStd
    module.thetaBias = thetaBias
    module.thetaDotBias = thetaDotBias
    module.thetaLSB = thetaLSB
    module.thetaDotLSB = thetaDotLSB
    module.setRNGSeed(2) # change RNG seed here


    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(totalTime))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    sensedTheta = dataLog.theta[-1]
    sensedThetaDot = dataLog.thetaDot[-1]
    
    # add bias to test values
    biasTheta = trueTheta+thetaBias
    biasThetaDot = trueThetaDot+thetaDotBias
    
    # discretize test values
    if thetaLSB > 0:
        discTheta = round(biasTheta/thetaLSB)*thetaLSB
    else:
        discTheta = biasTheta
    if thetaDotLSB > 0:
        discThetaDot = round(biasThetaDot/thetaDotLSB)*thetaDotLSB
    else:
        discThetaDot = biasThetaDot
        
    print(sensedTheta)
    print(sensedThetaDot)
    print(trueTheta)
    print(trueThetaDot)

    # check adding bias
    if abs(thetaBias) > accuracy:
        if not unitTestSupport.isDoubleEqual(sensedTheta, discTheta, accuracy):
            testMessages.append("Failed theta bias.")
            testFailCount += 1
        if not unitTestSupport.isDoubleEqual(sensedThetaDot, discThetaDot, accuracy):
            testMessages.append("Failed thetaDot bias.")
            testFailCount += 1
    
    # check discretization
    if abs(thetaLSB) > accuracy:
        if not unitTestSupport.isDoubleEqual(sensedTheta, discTheta, accuracy):
            testMessages.append("Failed theta discretization.")
            testFailCount += 1
        if not unitTestSupport.isDoubleEqual(sensedThetaDot, discThetaDot, accuracy):
            testMessages.append("Failed thetaDot discretization.")
            testFailCount += 1

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)
        
    if show_plots:
        thetaVals = trueTheta*np.ones(int(totalTime/timeStep)+1)
        thetaDotVals = trueThetaDot*np.ones(int(totalTime/timeStep)+1)
        timeAxis = dataLog.times() * macros.NANO2SEC

        plt.figure(1)
        plt.plot(timeAxis, thetaVals,
                 label=r'Theta Truth')
        plt.plot(timeAxis, dataLog.theta,
                 label=r'Theta Sensed')
        plt.legend(loc='lower right')
        plt.xlabel('Time [s]')
        plt.ylabel('Theta [rad]')
        plt.figure(2)
        plt.plot(timeAxis, thetaDotVals,
                 label=r'ThetaDot Truth')
        plt.plot(timeAxis, dataLog.thetaDot,
                 label=r'ThetaDot Sensed')
        plt.legend(loc='lower right')
        plt.xlabel('Time [s]')
        plt.ylabel('ThetaDot [rad]')
        plt.show()
        plt.close("all")

    return [testFailCount, "".join(testMessages)]
    

if __name__ == "__main__":
    test_hingedRigidBodyMotorSensor(
        True,
        0.1, 0.05, # noise: note this makes bias/disc tests FAIL
        0.0, 0.0, # bias
        -1, -1, # discretization
        1.01, -0.23 , # true values
        1.0e-12) # accuracy
