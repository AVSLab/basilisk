# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import MtbEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
@pytest.mark.parametrize("missingInput", ["mtbCmdInMsg", "magInMsg", "mtbParamsInMsg"])
def test_mtbEffector_validation(validationPath, missingInput):
    """Attachment initialization and direct Reset() must reject missing required inputs."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    mtbEffector = MtbEffector.MtbEffector()
    inputMessages = {
        "mtbCmdInMsg": messaging.MTBCmdMsg().write(messaging.MTBCmdMsgPayload()),
        "magInMsg": messaging.MagneticFieldMsg().write(messaging.MagneticFieldMsgPayload()),
        "mtbParamsInMsg": messaging.MTBArrayConfigMsg().write(messaging.MTBArrayConfigMsgPayload())
    }
    for inputName, inputMessage in inputMessages.items():
        if inputName != missingInput:
            getattr(mtbEffector, inputName).subscribeTo(inputMessage)
    spacecraftObject.addDynamicEffector(mtbEffector)
    simulation.AddModelToTask("task", spacecraftObject)

    with pytest.raises(BasiliskError, match=missingInput):
        if validationPath == "reset":
            mtbEffector.Reset(0)
        else:
            simulation.InitializeSimulation()
