#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
"""Regression checks for ordered stochastic increments on MuJoCo quaternions."""
import numpy as np
import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture import sysModel
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled, reason="Requires Basilisk built with --mujoco True"
)
if mujocoEnabled:
    from Basilisk.simulation import mujoco


@pytest.mark.parametrize("method", [
    "Mayurama", "EulerHeun", "RKMil", "SRIW1", "SOSRI", "SRA1", "SOSRA"
])
@pytest.mark.parametrize("highOrder", [False, True])
def test_noiseQuaternionPropagation(highOrder, method):
    """Apply two body-axis noise rotations in source order in both attitude modes.

    The position state has seven components, while each diffusion has six.
    For constant diffusion and zero drift, compare two steps against independent
    ordered quaternion products. This checks propagation plumbing, not the
    convergence order of these methods on a manifold.
    """
    # Sphere radius [m], mass [kg], and zero gravitational acceleration [m/s^2].
    xml = """<mujoco><option gravity="0 0 0"/><worldbody>
      <body name="hub"><freejoint/><geom type="sphere" size="1" mass="10"/></body>
    </worldbody></mujoco>"""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step = 0.125  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step)))
    scene = mujoco.MJScene(xml)
    scene.highOrderAttitudeIntegration = highOrder
    simulation.AddModelToTask("task", scene)
    angularDiffusion = [0.4, 0.7]  # [rad/sqrt(s)], body x and y axes

    class FixedDiffusion(sysModel.SysModel):
        """Republish the physical diffusion after each stage changes live storage."""

        def UpdateState(self, currentSimNanos):
            """Restore both constant diffusion columns for the current evaluation."""
            for source, amplitude in enumerate(angularDiffusion):
                diffusion = np.zeros((6, 1))
                diffusion[3 + source, 0] = amplitude
                position.setDiffusion(diffusion.tolist(), source)

    writer = FixedDiffusion()
    scene.AddModelToDiffusionDynamicsTask(writer)
    integrator = getattr(svIntegrators, "svStochasticIntegrator" + method)(scene)
    scene.setIntegrator(integrator)
    simulation.InitializeSimulation()

    position = scene.dynManager.getStateObject("mujocoQpos")
    position.setNumNoiseSources(2)

    noise = svIntegrators.PrescribedGaussianNoiseGenerator()
    increments = [[0.3, -0.2], [-0.15, 0.4]]  # [sqrt(s)]
    for sample in increments:
        noise.pushStep(sample)
    integrator.setNoiseGenerator(noise)
    expected = np.array([1.0, 0.0, 0.0, 0.0])
    integrator.integrate(0.0, 0.0)
    assert noise.remaining() == len(increments)

    for index, sample in enumerate(increments):
        integrator.integrate(index * step, step)
        for source, amplitude in enumerate(angularDiffusion):
            angle = amplitude * sample[source]
            rotation = np.zeros(4)
            rotation[0] = np.cos(angle / 2.0)
            rotation[source + 1] = np.sin(angle / 2.0)
            scalar = expected[0] * rotation[0] - np.dot(expected[1:], rotation[1:])
            vector = (expected[0] * rotation[1:] + rotation[0] * expected[1:]
                      + np.cross(expected[1:], rotation[1:]))
            expected = np.concatenate(([scalar], vector))
        actual = np.asarray(position.getState()).reshape(-1)
        np.testing.assert_allclose(actual[:3], 0.0, rtol=0.0, atol=1e-14)
        np.testing.assert_allclose(actual[3:], expected, rtol=0.0, atol=1e-14)
        assert np.linalg.norm(actual[3:]) == pytest.approx(1.0, abs=1e-14)
    assert noise.remaining() == 0
