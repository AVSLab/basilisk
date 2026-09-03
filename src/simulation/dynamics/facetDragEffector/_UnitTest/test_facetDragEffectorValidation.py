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

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import facetDragDynamicEffector, spacecraft, spinningBodyOneDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


@pytest.mark.parametrize("attachToBranch", [False, True], ids=["hub", "branch"])
@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
def test_facetDragEffector_validation(attachToBranch, validationPath):
    """Hub, branch, and direct Reset paths must reject a missing density input."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()
    if attachToBranch:
        spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
        spinningBody.mass = 1.0  # [kg]
        spinningBody.sHat_S = [[1.0], [0.0], [0.0]]  # [-]
        spinningBody.dcm_S0B = [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]]  # [-]
        spinningBody.IPntSc_S = [[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]]  # [kg m^2]
        spinningBody.addDynamicEffector(dragEffector)
        spacecraftObject.addStateEffector(spinningBody)
    else:
        spacecraftObject.addDynamicEffector(dragEffector)
    simulation.AddModelToTask("task", spacecraftObject)

    with pytest.raises(BasiliskError, match="atmoDensInMsg"):
        if validationPath == "reset":
            dragEffector.Reset(0)
        else:
            simulation.InitializeSimulation()
