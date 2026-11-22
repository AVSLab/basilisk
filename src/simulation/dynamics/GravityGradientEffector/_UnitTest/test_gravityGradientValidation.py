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

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import GravityGradientEffector, gravityEffector, spacecraft, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros


@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
def test_gravityGradient_validation(validationPath):
    """Attachment initialization and direct Reset() must reject an empty planet list."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    gravityGradient = GravityGradientEffector.GravityGradientEffector()
    spacecraftObject.addDynamicEffector(gravityGradient)
    simulation.AddModelToTask("task", spacecraftObject)

    with pytest.raises(BasiliskError, match="planetPropertyNames"):
        if validationPath == "reset":
            gravityGradient.Reset(0)
        else:
            simulation.InitializeSimulation()


def test_shared_body_gravity_gradient_torques():
    """Each spacecraft uses its own planet properties when both share an ephemeris."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = macros.sec2nano(1.0)  # [ns]
    body = gravityEffector.GravBodyData()
    body.planetName = "planet"
    body.mu = 1e5  # [m^3/s^2]
    sample = messaging.SpicePlanetStateMsgPayload()
    source = messaging.SpicePlanetStateMsg().write(sample)
    body.planetBodyInMsg.subscribeTo(source)
    inertia = np.diag([1.0, 2.0, 3.0])  # [kg*m^2]
    positions = np.array([[100.0, 0.0, 0.0], [-100.0, 0.0, 0.0]])  # [m]
    vehicles, gradients = [], []
    for i, position in enumerate(positions):
        task = f"task{i}"
        process.addTask(sim.CreateNewTask(task, (i + 1) * step))
        sc = spacecraft.Spacecraft()
        sc.hub.mHub = 10.0  # [kg]
        sc.hub.IHubPntBc_B = inertia.tolist()
        sc.hub.r_CN_NInit = position.tolist()
        sc.gravField.addGravBody(body)
        gg = GravityGradientEffector.GravityGradientEffector()
        gg.addPlanetName(body.planetName)
        sc.addDynamicEffector(gg)
        sim.AddModelToTask(task, sc, 2)
        sim.AddModelToTask(task, gg, 1)
        vehicles.append(sc)
        gradients.append(gg)

    sim.InitializeSimulation()
    sample.PositionVector = [0.0, 100.0, 0.0]  # [m]
    source.write(sample, 0)
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()
    # Identity spacecraft attitudes make the analytic torques equal and opposite about z.
    relative = positions - sample.PositionVector
    radii = np.linalg.norm(relative, axis=1)[:, None]  # [m]
    expected = 3 * body.mu * np.cross(relative, relative @ inertia) / radii**5  # [N*m]
    measured = [gg.gravityGradientOutMsg.read().gravityGradientTorque_B for gg in gradients]
    np.testing.assert_allclose(measured, expected, rtol=1e-12, atol=1e-15)  # [N*m]

    # Only the faster spacecraft should publish the second ephemeris to its state manager.
    sample.VelocityVector = [0.0, 2.0, 0.0]  # [m/s]
    sample.J20002Pfix = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]  # [-]
    sample.J20002Pfix_dot = np.diag([-0.1, -0.1, 0.0]).tolist()  # [1/s]
    source.write(sample, step)
    sim.ConfigureStopTime(step)
    sim.ExecuteSimulation()
    for name, updated, previous in [
        ("planet.v_PN_N", sample.VelocityVector, np.zeros(3)),
        ("planet.J20002Pfix", sample.J20002Pfix, np.eye(3)),
        ("planet.J20002Pfix_dot", sample.J20002Pfix_dot, np.zeros((3, 3))),
    ]:
        for sc, value in zip(vehicles, (updated, previous)):
            np.testing.assert_array_equal(np.squeeze(sc.dynManager.getPropertyReference(name)), value)


@pytest.mark.parametrize("mode", ["hub", "offset", "translation-only"])
def test_shared_central_body_torques_are_translation_invariant(mode):
    """Both spacecraft retain their analytic torque when the central ephemeris is translated and boosted."""
    sim = SimulationBaseClass.SimBaseClass()
    step = macros.sec2nano(1.0)  # [ns]
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", step))
    center = gravityEffector.GravBodyData()
    center.planetName = "center"
    center.isCentralBody = True
    center.mu = 1e5  # [m^3/s^2]
    sample = messaging.SpicePlanetStateMsgPayload()
    source = messaging.SpicePlanetStateMsg().write(sample)
    center.planetBodyInMsg.subscribeTo(source)
    inertia = np.diag([1.0, 2.0, 3.0])  # [kg*m^2]
    relative = np.array([[100.0, 100.0, 0.0], [-100.0, 100.0, 0.0]])  # [m]
    velocity = np.array([0.0, -200.0, 0.0])  # [m/s]
    readers = []
    for position in relative:
        sc = spacecraft.Spacecraft()
        sc.pointMassTranslationalOnly = mode == "translation-only"
        if mode == "offset":
            sc.hub.r_BcB_B = [2.0, 1.0, 0.0]  # [m]
        sc.hub.mHub = 10.0  # [kg]
        sc.hub.IHubPntBc_B = inertia.tolist()
        sc.hub.r_CN_NInit = position.tolist()
        sc.hub.v_CN_NInit = velocity.tolist()
        sc.gravField.addGravBody(center)
        sc.setIntegrator(svIntegrators.svIntegratorEuler(sc))
        gg = GravityGradientEffector.GravityGradientEffector()
        gg.addPlanetName(center.planetName)
        sc.addDynamicEffector(gg)
        sim.AddModelToTask("task", sc, 2)
        sim.AddModelToTask("task", gg, 1)
        readers.append(gg.gravityGradientOutMsg.addSubscriber())

    sim.InitializeSimulation()
    # Euler samples the initial geometry twice, then the advanced geometry with the opposite torque.
    advanced = relative + velocity * (step * macros.NANO2SEC)  # [m]
    for epoch, positions in ((0, relative), (step, relative), (2 * step, advanced)):
        if epoch == step:
            sample.PositionVector = [1000.0, 0.0, 0.0]  # [m] at t=1 s
            sample.VelocityVector = [0.0, 25.0, 0.0]  # [m/s]
            source.write(sample, epoch)
        sim.ConfigureStopTime(epoch)
        sim.ExecuteSimulation()
        assert all(reader.isWritten() and reader.timeWritten() == epoch for reader in readers)
        radii = np.linalg.norm(positions, axis=1)[:, None]  # [m]
        expected = 3 * center.mu * np.cross(positions, positions @ inertia) / radii**5  # [N*m]
        measured = [reader().gravityGradientTorque_B for reader in readers]
        np.testing.assert_allclose(measured, expected, rtol=1e-12, atol=1e-15)  # [N*m]
