import pytest
import numpy as np
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.architecture import messaging
import gc
import psutil
import os

def getMemoryUsage():
    """Get memory usage of current process in MB"""
    process = psutil.Process(os.getpid())
    gc.collect()  # Force collection before measurement
    memory_info = process.memory_info()
    return memory_info.rss / 1024 / 1024  # RSS in MB

def create_and_run_simulation():
    """Create and run a simulation with RW setup"""
    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create simulation
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Create spacecraft and RW objects
    scObject = spacecraft.Spacecraft()
    rwFactory = simIncludeRW.rwFactory()

    # Create 3 reaction wheels
    for i in range(3):
        rwFactory.create('Honeywell_HR16',
                        [1, 0, 0] if i == 0 else [0, 1, 0] if i == 1 else [0, 0, 1],
                        Omega=500.,
                        maxMomentum=50.)

    # Create RW state effector
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "ReactionWheel"
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # Add models to task
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add logging
    rwLogs = []
    for item in range(rwFactory.getNumOfDevices()):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder())
        unitTestSim.AddModelToTask(unitTaskName, rwLogs[item])

    # Run simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitTestSim.ExecuteSimulation()

    # Cleanup in reverse order of creation
    # Clear logs first
    for log in rwLogs:
        del log

    # Clear message subscriptions
    for msg in rwStateEffector.rwOutMsgs:
        if hasattr(msg, 'unsubscribeAll'):
            msg.unsubscribeAll()

    # Clear references in reverse order
    del rwStateEffector
    del rwFactory
    del scObject
    del testProc
    del unitTestSim

    gc.collect()

@pytest.mark.parametrize("num_iterations,max_allowed_growth", [
    (25, 3.0),   # Reduced from 50 to 25 iterations
    (50, 3.0),   # Reduced from 100 to 50 iterations
])
def test_rw_memory_leak(num_iterations, max_allowed_growth):
    """Test for memory leaks in reaction wheel implementation"""
    initial_memory = getMemoryUsage()
    memory_measurements = []

    # Run multiple iterations
    for i in range(num_iterations):
        create_and_run_simulation()

        # Take measurement after forced GC
        gc.collect()
        current_memory = getMemoryUsage()
        memory_measurements.append(current_memory)

        if (i + 1) % 10 == 0:
            print(f"Iteration {i+1}/{num_iterations}, Memory: {current_memory:.2f} MB")
            print(f"Delta from start: {current_memory - initial_memory:.2f} MB")

    # Calculate memory statistics
    memory_growth = memory_measurements[-1] - initial_memory
    memory_trend = np.polyfit(range(len(memory_measurements)), memory_measurements, 1)[0]

    # More detailed failure messages
    if memory_growth >= max_allowed_growth:
        pytest.fail(f"Memory growth ({memory_growth:.2f} MB) exceeds maximum allowed "
                   f"({max_allowed_growth:.2f} MB)\nTrend: {memory_trend:.4f} MB/iteration")

    if memory_trend >= 0.05:
        pytest.fail(f"Memory growth trend ({memory_trend:.4f} MB/iteration) indicates "
                   f"potential leak\nTotal growth: {memory_growth:.2f} MB")

if __name__ == "__main__":
    test_rw_memory_leak(50, 3.0)
