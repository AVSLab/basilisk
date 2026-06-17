#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
import pytest

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import svIntegrators

    couldImportMujoco = True
except:
    couldImportMujoco = False

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk

import numpy as np
import matplotlib.pyplot as plt

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat.xml"
REFERENCE_DATA = f"{TEST_FOLDER}/test_sat_qpos_reference.txt"


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_integration(showPlots: bool = False):
    """Tests that integration with RK4 in Basilisk is equal to integration
    with RK4 in MuJoCo.

    The simulation is simple. The multi-body is made of a central cube
    with two panels attached with one rotational degree of freedom (see
    ``test_sat.xml`` for details). No forces are exerted on the system,
    but the system is given an initial angular rate and the
    panels' joints are initialized to some velocity.

    The native mujoco toolbox was used to integrate this dynamic
    environment, and the results were saved to ``test_sat_qpos_reference.txt``.
    Basilisk is also used to integrate these dynamics, using the
    same integrator, thus the results should be numerically identical.
    """

    # The index of the state to check
    check = -1  # last state

    ref = np.loadtxt(REFERENCE_DATA)

    dt = ref[1, 0] - ref[0, 0] # s
    tf = ref[check, 0] # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create mujoco scene
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Create scene recorder
    mujocoStateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", mujocoStateRecorder)

    # initialize the simulation and set the initial rates
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scene.getBody("cube").setAttitudeRate([0.1, 0.2, 0.3]) # rad/s
    scene.getBody("panel_1").getScalarJoint("panel_1_elevation").setVelocity(0.1) # rad/s
    scene.getBody("panel_2").getScalarJoint("panel_2_elevation").setVelocity(0.1) # rad/s

    # Run the sim
    scSim.ExecuteSimulation()

    result = np.column_stack(
        [
            mujocoStateRecorder.times() * macros.NANO2SEC,
            np.squeeze(mujocoStateRecorder.qpos),
        ]
    )

    if showPlots:
        plt.figure()
        for i in range(1, ref.shape[1]):
            scale = np.max(np.abs(ref[:, i]))
            plt.plot(result[:, 0], result[:, i] / scale, label="Basilisk")
            plt.plot(ref[:, 0], ref[:, i] / scale, "k--", label="MuJoCo")
        plt.xlabel("Time [s]")
        plt.ylabel("Scaled State Value [-]")
        plt.show()

        fig, axs = plt.subplots(ref.shape[1] - 1)
        for i in range(1, ref.shape[1]):
            diff = ref[:, i] - result[:, i]
            axs[i - 1].plot(result[:, 0], diff)

        fig.supxlabel("Time [s]")
        fig.supylabel("Difference Basilisk vs. MuJoCo [-]")
        fig.tight_layout()

        plt.show()

    # Assert that the final position absolute error is < 1e-14
    # Which is essentially floating point noise
    assert result[check, 1:] == pytest.approx(ref[check, 1:], 0, 1e-14)

    # Check the same as above, but use the message cube origin message
    cubeState = scene.getBody("cube").getOrigin().stateOutMsg.read()
    assert ref[check, 1:4] == pytest.approx(cubeState.r_BN_N), (
        ref[check, 1:4],
        cubeState.r_BN_N,
    )
    quat = rbk.MRP2EP(cubeState.sigma_BN)
    assert ref[check, 4:8] == pytest.approx(quat)


# A torque-free single rigid body. With no external moment and a symmetric
# inertia the body rate is conserved, so a high-accuracy integration provides a
# reference attitude for the convergence and normalization checks below.
SPHERE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="1" mass="10"/>
    </body>
  </worldbody>
</mujoco>
"""


def _runFreeSpin(highOrder, dt, tf, omega):
    """Integrate the torque-free sphere from an initial body rate and return the
    final orientation quaternion (w,x,y,z) from the hub origin message."""
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(SPHERE_XML)
    scene.extraEoMCall = True
    scene.highOrderAttitudeIntegration = highOrder
    scSim.AddModelToTask("test", scene)

    integ = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integ)
    _hold = [integ]

    rec = scene.getBody("hub").getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
    scSim.AddModelToTask("test", rec)

    scSim.InitializeSimulation()
    scene.getBody("hub").setAttitudeRate(list(omega))
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    return rbk.MRP2EP(np.array(rec.sigma_BN)[-1])  # (w,x,y,z)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_highOrderQuaternionConvergesFasterThanDefault():
    """High-order attitude integration converges at a higher order than the
    default exponential-map path.

    Both are compared against a high-accuracy reference (very small step). When
    the step is halved, the high-order (RK4) quaternion error falls by roughly
    2^4 = 16, while the default path is only second order on SO(3) and falls by
    roughly 2^2 = 4.
    """
    omega = [0.4, -0.3, 0.8]  # rad/s, constant for a torque-free symmetric body
    tf = 5.0

    def attError(highOrder, dt, ref):
        q = _runFreeSpin(highOrder, dt, tf, omega)
        dcm = rbk.EP2C(q).dot(rbk.EP2C(ref).T)
        return 4.0 * np.arctan(np.linalg.norm(rbk.C2MRP(dcm)))

    refHi = _runFreeSpin(True, 0.005, tf, omega)
    refLo = _runFreeSpin(False, 0.005, tf, omega)

    ratioHi = attError(True, 0.10, refHi) / attError(True, 0.05, refHi)
    ratioLo = attError(False, 0.10, refLo) / attError(False, 0.05, refLo)

    assert ratioHi > 8.0  # close to 16 (fourth order)
    assert ratioLo < 6.0  # close to 4 (second order)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("highOrder", [False, True])
def test_quaternionStaysNormalized(highOrder):
    """The orientation quaternion stays unit-norm after many integration steps in
    both the default and high-order attitude modes."""
    w, x, y, z = _runFreeSpin(highOrder, dt=0.1, tf=20.0, omega=[0.3, -0.2, 0.5])
    assert np.sqrt(w * w + x * x + y * y + z * z) == pytest.approx(1.0, abs=1e-10)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_repeatedResetSucceeds():
    """A scene can be reset more than once. ``test_sat.xml`` has nq != nv (free
    joint plus hinges), so the bulk states must be registered at their compiled
    sizes for a second reset to succeed."""
    scene = mujoco.MJScene.fromFile(XML_PATH)
    integ = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integ)
    _hold = [integ]

    scene.Reset(0)
    scene.Reset(0)  # must not raise


if __name__ == "__main__":
    if True:
        import tqdm

        test_integration(True)

        N = 0
        # Run a bunch of times to ensure reproducibility
        for i in tqdm.tqdm(range(N)) if N > 0 else []:
            test_integration(False)
    else:
        pytest.main([__file__])
