
#   3rd party / std lib imports
import pytest
import os, inspect
import numpy as np

#   Utilities/macros
from Basilisk.utilities import SimulationBaseClass as sbc 
from Basilisk.utilities import macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk 
from Basilisk.utilities import orbitalMotion as om 

#   Modules to test
from Basilisk.fswAlgorithms import hillToAttRef
from Basilisk.architecture import messaging
#from Basilisk.simulation import simFswInterfaceMessages

@pytest.mark.xfail(reason="test does not use correct truth values")
@pytest.mark.parametrize("msg_type", ['AttRefMsg','NavAttMsg'])
@pytest.mark.parametrize("use_limits", [True, False])
def test_hillToAttRef(show_plots, use_limits, msg_type):
        """
    **Validation Test Description**

    Unit test for hillToAttRef. The unit test specifically covers:

    1. Input message types: Does hillToAttRef accept either a NavAttMsg or an AttRefMsg and produce identical behavior with either one?

    2. Limit enforcement: When set, does the module correctly use the limits specified by the user?

    """
        runner(show_plots, use_limits, msg_type)

def runner(show_plots, use_limits, msg_type):
    sim = sbc.SimBaseClass()
    procName = 'process'
    taskName = 'task'
    proc = sim.CreateNewProcess(procName)
    task =  sim.CreateNewTask(taskName, macros.sec2nano(1.0))
    proc.addTask(task)

    #   Set up a test relative state vector
    relative_state = [1000, 0, 0, 
                      0,    5, 0] # m / m/s
    #   Set up a dummy gain matrix
    lqr_gain_set = np.array([[0,1,0],
                             [0,0,0],
                             [0,0,0],
                             [0,0,0],
                             [0,0,0.25],
                             [0,0,0],       ]).T
    hillStateMsgData = messaging.HillRelStateMsgPayload()
    hillStateMsgData.r_DC_H = relative_state[0:3]
    hillStateMsgData.v_DC_H = relative_state[3:]
    hillStateMsg = messaging.HillRelStateMsg().write(hillStateMsgData)
    

    #   Set up the hillStateConverter
    depAttRefData = hillToAttRef.HillToAttRefConfig()
    depAttRefWrap = sim.setModelDataWrap(depAttRefData)
    depAttRefWrap.ModelTag = "dep_hillControl"
    depAttRefData.gainMatrix = hillToAttRef.MultiArray(lqr_gain_set)
    depAttRefData.hillStateInMsg.subscribeTo(hillStateMsg)
    if msg_type == 'NavAttMsg':
        attRefMsgData = messaging.NavAttMsgPayload()
        attRefMsgData.sigma_BN = [0.5, 0.5, 0.5]
        attRefMsg = messaging.NavAttMsg().write(attRefMsgData)
        depAttRefData.attNavInMsg.subscribeTo(attRefMsg)
    else:
        attRefMsgData = messaging.AttRefMsgPayload()
        attRefMsgData.sigma_RN = [0.2, 0.2, 0.2]
        attRefMsg = messaging.AttRefMsg().write(attRefMsgData)
        depAttRefData.attRefInMsg.subscribeTo(attRefMsg)
    
    if use_limits:
        depAttRefData.relMRPMin = -0.2 #    Configure minimum MRP
        depAttRefData.relMRPMax = 0.2  #    Configure maximum MRP

        ref_vals = [0.2, -0.2, 0.2]
    else:
        ref_vals = lqr_gain_set.dot(relative_state)
    
    #   Store the output att ref message
    depAttRecorder = depAttRefData.attRefOutMsg.recorder()
    
    sim.AddModelToTask(taskName, depAttRefWrap, depAttRefData)
    sim.AddModelToTask(taskName, depAttRecorder)

    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

    hill_positions = depAttRecorder.sigma_RN
    #   Test the attitude calculation:
    # Note, this unit test is broken as it does not provide correct truth values.  This was not
    # caught earlier as the manner of testing was not correct either.
    for val1, val2 in zip(hill_positions[-1], ref_vals):
        assert val1 == pytest.approx(val2)

if __name__=="__main__":
    test_hillToAttRef(False, True, 'AttRefMsg')
