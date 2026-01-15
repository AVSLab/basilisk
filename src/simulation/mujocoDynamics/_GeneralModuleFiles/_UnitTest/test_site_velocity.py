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

import os

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

import numpy as np
import pytest

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/sat_hub_only.xml"


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_siteVelocity():
    """Test the velocity output of sites."""

    dt = 1 # s
    tf = 30 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create scene
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set initial conditions
    initialPosition = [1.5,-1,0.75]
    initialVelocity = [5e-3,-5e-3,5e-3]
    initialAttitude = [0.15,-0.1,0.1]
    initialAttitudeRate = [0.01,-0.01,0.005]

    # Save body state through recorder
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Set initial states
    scene.getBody("hub").setPosition(initialPosition)
    scene.getBody("hub").setVelocity(initialVelocity)
    scene.getBody("hub").setAttitude(initialAttitude)
    scene.getBody("hub").setAttitudeRate(initialAttitudeRate)

    # run sim
    scSim.ExecuteSimulation()

    # validate the simulation output values
    v_BN_N = bodyStateRecorder.v_BN_N[-1]
    omega_BN_B = bodyStateRecorder.omega_BN_B[-1]

    np.testing.assert_allclose(v_BN_N, np.array(initialVelocity), rtol=1e-8,
                               err_msg=f"Linear velocity {v_BN_N} not close to expected {initialVelocity}")
    np.testing.assert_allclose(omega_BN_B, np.array(initialAttitudeRate), rtol=1e-8,
                               err_msg=f"Angular velocity {omega_BN_B} not close to expected {initialAttitudeRate}")


if __name__ == "__main__":
    test_siteVelocity()
