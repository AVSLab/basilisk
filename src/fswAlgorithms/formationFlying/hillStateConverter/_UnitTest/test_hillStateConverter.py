
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
from Basilisk.fswAlgorithms import hillStateConverter
from Basilisk.architecture import messaging
#from Basilisk.simulation import simFswInterfaceMessages

def test_hillStateConverter(show_plots):
    """
    Tests the hillStateConverter module for the following:
    1. Accepts both a hill and deputy message;
    2. Correctly converts those messages into the hill frame.
    """
    sim = sbc.SimBaseClass()
    procName = 'process'
    taskName = 'task'
    proc = sim.CreateNewProcess(procName)
    task =  sim.CreateNewTask(taskName, macros.sec2nano(1.0))
    proc.addTask(task)

    #   Set up two spacecraft position messages
    chief_r = [7100,0,0]
    chief_v = [0,7.000,0]
    dep_r = [7101, 0, 0]
    dep_v = [0,7.010,0]

    chiefNavMsgData = messaging.NavTransMsgPayload()
    chiefNavMsgData.r_BN_N = chief_r
    chiefNavMsgData.v_BN_N = chief_v
    chiefNavMsg = messaging.NavTransMsg().write(chiefNavMsgData)

    depNavMsgData = messaging.NavTransMsgPayload()
    depNavMsgData.r_BN_N = dep_r 
    depNavMsgData.v_BN_N = dep_v
    depNavMsg = messaging.NavTransMsg().write(depNavMsgData)

    #   Set up the hillStateConverter
    hillStateNavData = hillStateConverter.HillStateConverterConfig()
    hillStateNavWrap = sim.setModelDataWrap(hillStateNavData)
    hillStateNavWrap.ModelTag = "dep_hillStateNav"
    hillStateNavData.chiefStateInMsg.subscribeTo(chiefNavMsg)
    hillStateNavData.depStateInMsg.subscribeTo(depNavMsg)
    hillRecorder = hillStateNavData.hillStateOutMsg.recorder()
    
    sim.AddModelToTask(taskName, hillStateNavWrap, hillStateNavData)
    sim.AddModelToTask(taskName, hillRecorder)

    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

    hill_positions = hillRecorder.r_DC_H
    hill_velocities = hillRecorder.v_DC_H

    ref_pos = [1,0,0]
    ref_vel = [0,0.00901408,0]
    #   Test the position calculation:
    for val1, val2 in zip(hill_positions, ref_pos):
        assert pytest.approx(val1, val2)

    for val1, val2  in zip(hill_velocities, ref_vel):
        assert pytest.approx(val1, val2)

if __name__=="__main__":
    test_hillStateConverter(False)
