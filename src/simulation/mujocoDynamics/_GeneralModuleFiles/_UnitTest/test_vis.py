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

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators

import numpy as np

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat_2.xml"


# A manual test, since pytest can't check visualization
def vis(initialSpin=True):
    """A script to run a simple MJScene-based simulation and
    then use the visualization tool.

    Args:
        initialSpin (bool, optional): If True, the satellite
        is given an initial angular velocity. Defaults to True.
    """

    dt = 0.01 # s
    tf = 13 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create scene
    scene = mujoco.MJScene.fromFile(XML_PATH)
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)
    scSim.AddModelToTask("test", scene)

    # Save scene state through recorder
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # Initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Set attitude rate if needed
    if initialSpin:
        scene.getBody("cube").setAttitudeRate([0.1, 0.2, 0.3]) # rad/s

    # Create a force on the joints of the panels
    elevationControllerMsgPayload = messaging.SingleActuatorMsgPayload()
    elevationControllerMsgPayload.input = 5 # N*m
    elevationControllerMsg = messaging.SingleActuatorMsg()
    elevationControllerMsg.write(elevationControllerMsgPayload)

    scene.getSingleActuator("panel_1_elevation").actuatorInMsg.subscribeTo(
        elevationControllerMsg
    )
    scene.getSingleActuator("panel_2_elevation").actuatorInMsg.subscribeTo(
        elevationControllerMsg
    )

    # run sim
    scSim.ExecuteSimulation()

    # run visualization
    mujoco.visualize(stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene)


if __name__ == "__main__":
    vis(True)
