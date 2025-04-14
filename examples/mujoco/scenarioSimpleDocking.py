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

"""
It's recommended to review the following scenario(s) first (and any
recommended scenario(s) that they may have):

#. ``examples/mujoco/scenarioArmWithThrusters.py``

This scripts shows how one can simulate two independent spacecraft
docking and constraining their movement with respect to each other.
This illustrates the use of MuJoCo ``equality`` in :ref:`MJScene<MJScene>`.

The multi-body system is defined in the XML file ``sats_dock.xml``.
The system defined in this XML file is simple, but it illustrates some
of the tools MuJoCo offers for our simulation needs. First, this shows
how contact physics can be disabled through ``flag contact="disable"``.
Second, it illustrates how two free-floating bodies can be defined.
Third, it shows how to declare an ``equality``.

Equalities are a way to add dynamical constraints to a system. In this
case, we use a 'weld' constraint between two sites, which tells MuJoCo
to enforce that the frames associated with the sites are aligned. This
prevents any relative movement between them, and thus between the bodies
that they are attached to. We use this to simulate the dynamics after
docking. MuJoCo will automatically compute the force and torques that
are produced at this point to ensure the equality constraint is met.

For more information about constraints, see
https://mujoco.readthedocs.io/en/stable
"""

import os
from typing import Sequence

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators

import numpy as np
import matplotlib.pyplot as plt

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sats_dock.xml"


def setThrusterForce(
    msgs: Sequence[messaging.SingleActuatorMsg], thrust: Sequence[float]
):
    """Writes the ``input`` argument of the given messages with
    the values in ``thrust``."""
    for msg, val in zip(msgs, thrust):
        forceMsgPayload = messaging.SingleActuatorMsgPayload()
        forceMsgPayload.input = val
        msg.write(forceMsgPayload)


def run(showPlots: bool = False, visualize: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        visualize (bool, optional): If True, the ``MJScene`` visualization tool is
            run on the simulation results. Defaults to False.
    """

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(0.1)))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)

    # Create two stand-alone messages to control the thrusters
    # of each Hub
    hub1ThrusterMsg = messaging.SingleActuatorMsg()
    hub2ThrusterMsg = messaging.SingleActuatorMsg()
    scene.getSingleActuator("hub_1_thruster").actuatorInMsg.subscribeTo(hub1ThrusterMsg)
    scene.getSingleActuator("hub_2_thruster").actuatorInMsg.subscribeTo(hub2ThrusterMsg)

    # Record the position of the contact point on both CubeSats
    hub1ContantRecorder = (
        scene.getBody("hub_1").getSite("hub_1_contact_point").stateOutMsg.recorder()
    )
    hub2ContantRecorder = (
        scene.getBody("hub_2").getSite("hub_2_contact_point").stateOutMsg.recorder()
    )
    scSim.AddModelToTask("test", hub1ContantRecorder)
    scSim.AddModelToTask("test", hub2ContantRecorder)

    # Record the minimal coordinates of the entire scene for visualization
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Set the initial position of both CubeSats
    scene.getBody("hub_1").setPosition([-1, 0, 0]) # m
    scene.getBody("hub_2").setPosition([0, 0, -2]) # m

    # Thrust so that the spacecraft get close to each other
    force = 1.0
    thrusterMsgs = [hub1ThrusterMsg, hub2ThrusterMsg]
    setThrusterForce(thrusterMsgs, [force, force])
    scSim.ConfigureStopTime(macros.sec2nano(1))
    scSim.ExecuteSimulation()

    # Use opposite thrust to slow down
    setThrusterForce(thrusterMsgs, [-force, -force])
    scSim.ConfigureStopTime(macros.sec2nano(2))
    scSim.ExecuteSimulation()

    # Now, the cubesats should be together and be stationary.
    # Turn off thrusters, and keep like this for .5 secods.
    setThrusterForce(thrusterMsgs, [0, 0])
    scSim.ConfigureStopTime(macros.sec2nano(2.5))
    scSim.ExecuteSimulation()

    # Activate the 'dlock' equality, which forces the frames of
    # hub_1_contact_point and hub_2_contact_point to be aligned.
    scene.getEquality("dock").setActive(True)

    # Turn on the thruster of hub_1 only. Because of the active
    # constrain, hub_2 should be dragged along.
    setThrusterForce(thrusterMsgs, [force, 0])
    scSim.ConfigureStopTime(macros.sec2nano(4))
    scSim.ExecuteSimulation()

    # Disable the 'dock' equality, which should allow both bodies
    # to separate again
    scene.getEquality("dock").setActive(False)

    # Run for some extra time, the hubs should separate
    setThrusterForce(thrusterMsgs, [-force, 0])
    scSim.ConfigureStopTime(macros.sec2nano(6))
    scSim.ExecuteSimulation()

    if showPlots:
        rContact1_N = np.squeeze(hub1ContantRecorder.r_BN_N)
        rContact2_N = np.squeeze(hub2ContantRecorder.r_BN_N)
        distance = np.linalg.norm(rContact1_N - rContact2_N, axis=1)

        plt.figure()
        plt.plot(hub1ContantRecorder.times() * macros.NANO2SEC, distance)
        plt.xlabel("Time [s]")
        plt.ylabel("Distance Between Hubs [m]")

        plt.show()

    if visualize:
        speedUp = 0.25
        mujoco.visualize(
            stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, speedUp
        )


if __name__ == "__main__":
    run(True, True)
