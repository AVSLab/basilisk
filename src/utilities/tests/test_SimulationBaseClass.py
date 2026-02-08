from unittest.mock import MagicMock

import numpy as np
import pytest
from Basilisk.utilities import SimulationBaseClass, macros


@pytest.mark.parametrize("stopTime1", [0.0, 8.0, 10.0, 12.0])
@pytest.mark.parametrize("stopTime2", [0.0, 8.0, 10.0, 12.0, 95.0, 100.0, 105.0])
@pytest.mark.parametrize("StopCondition", ["<=", ">="])
def test_ConfigureStopTime(stopTime1, stopTime2, StopCondition):
    sim_rate = 10.0

    # Skip nonsense tests
    if stopTime1 > stopTime2:
        return

    # Create a simulation
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("TestProcess")
    simulationTimeStep = macros.sec2nano(sim_rate)
    dynProcess.addTask(scSim.CreateNewTask("TestTask", simulationTimeStep))
    scSim.InitializeSimulation()

    for stopTime in [stopTime1, stopTime2]:
        scSim.ConfigureStopTime(macros.sec2nano(stopTime), StopCondition)
        scSim.ExecuteSimulation()

        if StopCondition == "<=":
            assert scSim.TotalSim.CurrentNanos == macros.sec2nano(
                np.floor(stopTime / sim_rate) * sim_rate
            )
        elif StopCondition == ">=":
            assert scSim.TotalSim.CurrentNanos == macros.sec2nano(
                np.ceil(stopTime / sim_rate) * sim_rate
            )


@pytest.mark.parametrize(
    "creationTime,eventRate,expectedCallsExactRate,expectedCallsInexactRate",
    [
        (0.0, 10.0, 10, 10),
        (10.0, 10.0, 9, 9),
        (0.0, 40.0, 3, 3),
        (10.0, 40.0, 2, 3),
        (0.0, np.pi, 1, 10),
        (0.0, 5.0, 10, 10),
        (0.0, 7.5, 4, 10),
        (0.0, 12.5, 2, 5),
        (0.0, 15.0, 4, 5),
    ],
)
@pytest.mark.parametrize("exactRateMatch", [True, False])
def test_Events(
    creationTime,
    eventRate,
    expectedCallsExactRate,
    expectedCallsInexactRate,
    exactRateMatch,
):
    sim_rate = 10.0
    sim_duration = 100.0

    # Create a simulation
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("TestProcess")
    simulationTimeStep = macros.sec2nano(sim_rate)
    dynProcess.addTask(scSim.CreateNewTask("TestTask", simulationTimeStep))
    scSim.InitializeSimulation()

    conditionFunction = MagicMock(return_value=False)

    # Step to event creation time
    scSim.ConfigureStopTime(macros.sec2nano(creationTime))
    scSim.ExecuteSimulation()

    # Create event and step through the rest of the sim
    scSim.createNewEvent(
        "TestEvent",
        conditionFunction=conditionFunction,
        eventRate=macros.sec2nano(eventRate),
        eventActive=True,
        exactRateMatch=exactRateMatch,
    )
    scSim.ConfigureStopTime(macros.sec2nano(sim_duration))
    scSim.ExecuteSimulation()

    if exactRateMatch:
        assert conditionFunction.call_count == expectedCallsExactRate
    else:
        assert conditionFunction.call_count == expectedCallsInexactRate
