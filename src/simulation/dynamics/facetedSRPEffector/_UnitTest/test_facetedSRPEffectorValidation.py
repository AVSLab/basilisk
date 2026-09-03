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

import numpy as np
import pytest

from Basilisk.architecture import astroConstants, messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import facetedSRPEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
@pytest.mark.parametrize(
    ("missingInput", "expectedMessage"),
    [("sun", "sunStateInMsg"),
     ("facet", "facetElementBodyInMsgs"),
     ("area", "facetProjectedAreaInMsgs")])
def test_facetedSRPEffector_validation(validationPath, missingInput, expectedMessage):
    """Attachment initialization and direct Reset() must reject every missing required input."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    srpEffector = facetedSRPEffector.FacetedSRPEffector()
    srpEffector.setNumFacets(1)
    sunMessage = messaging.SpicePlanetStateMsg().write(messaging.SpicePlanetStateMsgPayload())
    facetMessage = messaging.FacetElementBodyMsg().write(messaging.FacetElementBodyMsgPayload())
    areaMessage = messaging.ProjectedAreaMsg().write(messaging.ProjectedAreaMsgPayload())
    if missingInput != "sun":
        srpEffector.sunStateInMsg.subscribeTo(sunMessage)
    if missingInput != "facet":
        srpEffector.facetElementBodyInMsgs[0].subscribeTo(facetMessage)
    if missingInput != "area":
        srpEffector.facetProjectedAreaInMsgs[0].subscribeTo(areaMessage)
    spacecraftObject.addDynamicEffector(srpEffector)
    simulation.AddModelToTask("task", spacecraftObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        if validationPath == "reset":
            srpEffector.Reset(0)
        else:
            simulation.InitializeSimulation()


def test_facetedSRPEffector_spacecraftOnlyComputesForce():
    """An attached-only effector must initialize its caches and compute SRP force."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = [astroConstants.AU * 1000.0, 0.0, 0.0]  # [m]
    sunMessage = messaging.SpicePlanetStateMsg().write(sunPayload)

    facetPayload = messaging.FacetElementBodyMsgPayload()
    facetPayload.area = 1.0  # [m^2]
    facetPayload.nHat_B = [1.0, 0.0, 0.0]  # [-]
    facetPayload.c_diffuse = 0.0  # [-]
    facetPayload.c_specular = 0.0  # [-]
    facetMessage = messaging.FacetElementBodyMsg().write(facetPayload)

    projectedAreaPayload = messaging.ProjectedAreaMsgPayload()
    projectedAreaPayload.area = 1.0  # [m^2]
    projectedAreaMessage = messaging.ProjectedAreaMsg().write(projectedAreaPayload)

    srpEffector = facetedSRPEffector.FacetedSRPEffector()
    srpEffector.setNumFacets(1)
    srpEffector.sunStateInMsg.subscribeTo(sunMessage)
    srpEffector.facetElementBodyInMsgs[0].subscribeTo(facetMessage)
    srpEffector.facetProjectedAreaInMsgs[0].subscribeTo(projectedAreaMessage)
    spacecraftObject.addDynamicEffector(srpEffector)
    simulation.AddModelToTask("task", spacecraftObject)

    simulation.InitializeSimulation()

    expectedForce = np.array([-astroConstants.SOLAR_FLUX_EARTH / astroConstants.SPEED_LIGHT,
                              0.0,
                              0.0])  # [N]
    np.testing.assert_allclose(np.array(srpEffector.forceExternal_B).flatten(), expectedForce,
                               rtol=1e-12, atol=1e-15)
    assert srpEffector.CallCounts == 0
