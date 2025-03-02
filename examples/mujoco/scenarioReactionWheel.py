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
Recommended first scenario to learn MuJoCo-based simulation!

This script shows how to simulate a CubeSat with a reaction wheel using the
Mujoco-based dynamic object in Basilisk.

Traditional Basilisk simulations (which use the :ref:`spacecraft` :ref:`DynamicObject<dynamicObject>`)
are '`hub`'-centric. This means that one usually considers a single body to be
the 'main' body of the simulation, where forces and torques are applied.
``DynamicEffectors`` and ``StateEffectors`` are attached to this :ref:`DynamicObject<dynamicObject>`, which
simulate these forces/torques or provide some (limited) multi-body behavior.
For example, the ``reactionWheelStateEffector`` can be used to simulate a reaction
wheel on a :ref:`spacecraft` :ref:`DynamicObject<dynamicObject>`.

On the other hand, the Mujoco-based :ref:`DynamicObject<dynamicObject>` (:ref:`MJScene<MJScene>`) is more general
and allows for more complex multi-body dynamics. Under this formulation, there
is no 'main' body, and all bodies are treated equally. Forces and torques can
be applied at any point of any body. Two bodies can be connected by joints (like
the revolute joint of a wheel, or a prismatic joint of a linear actuator). Through
these joints, forces and torques from one body can be transmitted to another.
This allows to model a plethora of multi-body systems, without the need to
explicitly model the dynamics of each body. The same system can model a CubeSat,
a set of CubeSats, a robotic arm, reaction wheels, etc. Moreover, :ref:`MJScene<MJScene>` (a
:ref:`DynamicObject<dynamicObject>`) does not use ``DynamicEffectors`` and ``StateEffectors``. Instead,
:ref:`MJScene<MJScene>` has an internal "dynamics task", similar to other tasks in Basilisk.
This task can hold arbitrary :ref:`SysModel<sys_model>`, some of which can output forces and
torques to be applied at specific points of the multi-body system in :ref:`MJScene<MJScene>`.
Unlike regular tasks in Basilisk, which run at specific time intervals, the
dynamics task in :ref:`MJScene<MJScene>` runs at every time step of the integrator.

In this example, we show how to simulate a CubeSat with a reaction wheel using
the Mujoco-based dynamic object. The multi-body system consists of a 'hub' body
and a 'wheel' body. The wheel is connected to the hub by a revolute joint, which
allows the wheel to rotate around the hub. The wheel is actuated by a torque
applied at the center of mass of the wheel. The hub is allowed to move and rotate
freely. This system definition must be translated into the XML format that Mujoco
understands, which is done in the ``sat_w_wheel.xml`` file. The following is a
simplified version of this file, line numbers have been added on the left:

::

    1  <mujoco>
    2      <worldbody>
    3          <body name="hub">
    4              <freejoint/>
    5              <geom name="hub_box" type="box" size="1 1 1" density="200"/>
    6
    7              <body name="wheel_1" pos="0 0 0" xyaxes="1 0 0 0 0 1">
    8                  <joint name="wheel_1" type="hinge" axis="0 0 1"/>
    9                  <geom name="wheel_1" type="cylinder" size="0.4 0.2" density="2000"/>
    10             </body>
    11         </body>
    12     </worldbody>
    13
    14     <actuator>
    15         <motor name="wheel_1" joint="wheel_1" />
    16     </actuator>
    17
    18 </mujoco>


Every MuJoCo XML file must have a `<mujoco>` root element (line 1 and 18). The ``<worldbody>``
element (line 2) contains the bodies of the system. In this case, there are two bodies:
the 'hub' body (line 3) and the 'wheel_1' body (line 7). The 'hub' body is connected to
the world by a ``freejoint`` (line 4), which allows it to move and rotate freely. The 'hub'
body has a box geometry (line 5), which is used to define the visual representation,
mass properties, and collision properties of the body.

The 'wheel_1' body is connected to the 'hub' body by a revolute joint (line 8). This joint
allows the 'wheel_1' body to rotate around the 'hub' body. For revolute joints, the ``axis``
attribute defines the axis of rotation. The 'wheel_1' body has a cylinder geometry (line 9).
The 'wheel_1' body is defined inside the 'hub' body, which means that the position of the
'wheel_1' body is defined with respect to the 'hub' body. We say "hub" is the parent body,
while "wheel_1" is a sub-body or child body of the "hub" body. Bodies can have multiple
sub-bodies, but only one parent, which means that multi-body systems are (topologically)
trees. Closed-loop topologies can be simulated, but this is not covered in this example.

Each body has an 'origin' reference frame, which is the frame in which all sub-elements of
the body are defined. In the definition of the 'wheel_1' body, we define the position
and orientation of the 'origin' frame of the 'wheel_1' body with respect to the 'origin'
frame of the 'hub' body using the ``pos`` and ``xyaxes`` attributes (line 7). Geometries
of bodies are defined with respect to the 'origin' frame of said body. Similarly, the
``axis`` attribute of the joint is defined with respect to the 'origin' frame of the body.

The ``<actuator>`` element (line 14) contains the actuators of the system. In this case, there
is a single actuator called 'wheel_1' (line 15), which is linked to the 'wheel_1' joint.
This actuator can apply a torque to the 'wheel_1' joint. This actuator simulates a motor
applying the torque on the reaction wheel. The torque value can be controlled through
the Basilisk messaging system, as shown in the script.

This is a brief introduction to the MuJoCo XML format. For more information, please refer
to the MuJoCo documentation: https://mujoco.readthedocs.io/en/stable/XMLreference.html

The system is simulated for 120 seconds, with a torque of 2 Nm applied to
the wheel for the first 60 seconds, and a torque of -2 Nm applied for the
next 60 seconds. Based on conservation of momentum, the system should accelerate
its rotation in the first 60 seconds and then decelerate in the next 60 seconds.
The final angular velocity should be zero.

The system states are recorded and optionally plotted and/or visualized in a 3D
geometry visualization environment using ``mujoco.visualize``. It's possible that
this function is not available in your system, depending on whether you chose to
build the additional 'replay' utility when building Basilisk.
"""

import os

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators

import numpy as np
import matplotlib.pyplot as plt

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_wheel.xml"


def run(showPlots: bool = False, visualize: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        visualize (bool, optional): If True, the ``MJScene`` visualization tool is
            run on the simulation results. Defaults to False.
    """
    dt = 1 # s
    tf = 60 # s

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    # Note how all Basilisk integrators are supported for
    # this integrator object
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)

    # Define a stand-alone ``SingleActuatorMsg`` which defines
    # the torque that a single actuator should apply. 'Single
    # actuators' are those actuators that apply a single scalar
    # input to the system. In this case, the actuator is a motor
    # that applies a torque to the 'wheel_1' joint.
    torque = 2 # N*m
    TorqueAtSiteMsgPayload = messaging.SingleActuatorMsgPayload()
    TorqueAtSiteMsgPayload.input = torque
    torqueMsg = messaging.SingleActuatorMsg()
    torqueMsg.write(TorqueAtSiteMsgPayload)

    # Subscribe the actuator to the torque message.
    scene.getSingleActuator("wheel_1").actuatorInMsg.subscribeTo(torqueMsg)

    # Add the state recorder to the task
    # This recorder will print all the states of the system,
    # which may not very useful for users, since this is an array
    # containing the minimal-coordinates of the multi-body system.
    # However, the data is used in ``mujoco.visualize``.
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # The ``bodyStateRecorder`` is perhaps are more useful recorder
    # for users. The message being recorded is of type ``SCStatesMsgPayload``,
    # which contains the (inertial) position, velocity, attitude, and angular
    # velocity of the "origin" site of the "hub" body.
    # In this system, a 'body' is a 3D rigid object. 'Sites' are essentially
    # frames fixed on a 'body'. Each site has a ``stateOutMsg``, which reports
    # the state of this frame. More than one site can be attached to a body.
    # All bodies come with two sites by default: the 'origin' site (accesible
    # with ``getBody("body").getOrigin()``) and the 'center of mass' site
    # (accesible with ``getBody("body").getCenterOfMass()``).
    # The 'origin' frame is the frame in which all sub-elements of the body
    # are defined (for example, sub-bodies, joints, geometries, etc.).
    # The 'center of mass', as the name suggests, is the frame located at the
    # center of mass of the body.
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Run the simulation for 60 seconds
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Change the torque to -2 N*m
    TorqueAtSiteMsgPayload = messaging.SingleActuatorMsgPayload()
    TorqueAtSiteMsgPayload.input = -torque
    torqueMsg.write(TorqueAtSiteMsgPayload)

    # Run the simulation for another 60 seconds
    scSim.ConfigureStopTime(macros.sec2nano(2 * tf))
    scSim.ExecuteSimulation()

    # Plot the attitude of the hub body
    if showPlots:
        plt.figure()
        att = np.squeeze(bodyStateRecorder.sigma_BN)
        for i in range(3):
            plt.plot(
                bodyStateRecorder.times() * macros.NANO2SEC,
                att[:, i],
                "-",
                label=f"Hub sigma [{i}]",
            )
        plt.xlabel("Time [s]")
        plt.ylabel(r"$\sigma_{BN}$ [rad]")
        plt.legend()
        plt.show()

    # Visualize the system in a 3D environment
    if visualize:
        speedUp = 5
        mujoco.visualize(
            stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, speedUp
        )


if __name__ == "__main__":
    run(True, True)
