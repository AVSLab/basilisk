#
#  ISC License
#
#  Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway
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

import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.simulation import antennaPower

# Antenna state constants (from AntennaDefinitions.h)
ANTENNA_OFF = 0
ANTENNA_RX = 1
ANTENNA_TX = 2
ANTENNA_RXTX = 3


@pytest.mark.parametrize("antennaState", [ANTENNA_OFF, ANTENNA_RX, ANTENNA_TX, ANTENNA_RXTX])
@pytest.mark.parametrize("basePowerNeed", [0.0, 5.0])
def test_antennaPower(antennaState, basePowerNeed):
    """Test antenna power consumption for all antenna states with and without base power."""
    testFailCount = 0
    testMessages = []

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", testProcessRate))

    # Set up module
    antennaPowerModule = antennaPower.AntennaPower()
    antennaPowerModule.ModelTag = "antennaPower"
    antennaPowerModule.basePowerNeed = basePowerNeed
    unitTestSim.AddModelToTask("unitTask", antennaPowerModule)

    # Define test power values
    P_Rx = 2.0
    P_Tx = 10.0

    # Create and write antenna state message
    antennaMsgPayload = messaging.AntennaLogMsgPayload()
    antennaMsgPayload.antennaState = antennaState
    antennaMsgPayload.P_Rx = P_Rx
    antennaMsgPayload.P_Tx = P_Tx
    antennaMsg = messaging.AntennaLogMsg().write(antennaMsgPayload)
    antennaPowerModule.antennaSetStateInMsg.subscribeTo(antennaMsg)

    # Set up data logging
    dataLog = antennaPowerModule.nodePowerOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", dataLog)

    # Run simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Compute expected power
    if antennaState == ANTENNA_OFF:
        truePower = -basePowerNeed
    elif antennaState == ANTENNA_RX:
        truePower = -(P_Rx + basePowerNeed)
    elif antennaState == ANTENNA_TX:
        truePower = -(P_Tx + basePowerNeed)
    elif antennaState == ANTENNA_RXTX:
        truePower = -(P_Rx + P_Tx + basePowerNeed)

    # Check results
    for i, val in enumerate(dataLog.netPower):
        if not unitTestSupport.isDoubleEqual(float(val), truePower, 1e-12):
            testFailCount += 1
            testMessages.append(f"netPower[{i}]: got {val}, expected {truePower}")

    if testFailCount == 0:
        print("PASSED: " + antennaPowerModule.ModelTag)
    else:
        print(testMessages)

    assert testFailCount < 1, "".join(testMessages)

if __name__ == "__main__":
    for state in [ANTENNA_OFF, ANTENNA_RX, ANTENNA_TX, ANTENNA_RXTX]:
        for basePower in [0.0, 5.0]:
            test_antennaPower(state, basePower)
