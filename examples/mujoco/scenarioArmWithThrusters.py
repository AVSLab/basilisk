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

#. ``examples/mujoco/scenarioReactionWheel.py``

This scripts shows how to simulate a spacecraft that deploys two thrusters
at the end of a 4-link branching deployable arm. This uses the MuJoCo-based
:ref:`DynamicObject<dynamicObject>` :ref:`MJScene<MJScene>`.

The multi-body system is defined in the XML file ``sat_w_deployable_thruster.xml``.
This XML file defines a 'hub' body and a branching deployable arm with 4 links.
The first link ('arm_1') is attached to the 'hub' body, and the second link ('arm_2')
is attached to the first link. The third link ('arm_3') AND the fourth link ('arm_4')
are both attached to the second link. Both of the last two links have a site attached
to them ('thruster_1' and 'thruster_2', respectively), which defines the point at
which the thrust vector is applied. The joints between the links are revolute joints.

The thrusters are turned on and off at specific times. The thrust direction
and application point on the link body ('arm_3' or 'arm_4') are fixed throughout
the simulation. However, because the 'arm_3' and 'arm_4' will move with respect
the inertial frame (the 'hub' body is free-floating, the arm joints move), the effective
thrust points and directions will change.

Note that the only actuators defined are the ones used to model the thrusters ('thruster_1'
and 'thruster_2'). We could opt to define actuators for the joints of the arm, so that
deploying the arm is modeled as an actuation. In this case, the script would have to
define the torque to be applied at these joints to deploy the arm. This is complex, and
generally requires a control system to be implemented (see ``mujoco/scenarioDeployPanels.py``).
Instead, we opt for a simpler approach: we constrain the motion of the joints.

Prescribing the motion of joints can be useful to model a system that we can
assume is very accurately controlled by some unknown system. Essentially, we let
the dynamic object know what the motion of the joint should be, and the dynamic
engine will compute the forces necessary to achieve that motion. This implies that
the joint will not have exactly the motion we constrain, since we are numerically
solving the inverse dynamic problem (i.e., given the motion, what are the forces?).
However, the motion will be very close to what we constrain. It should be noted that
this comes at a considerable computational cost.

To constrain the motion of the joints, we connect to the ``constrainedStateInMsg`` of
the joint object.

This script also showcases the use of the ``XXXInterpolator`` models, which
are utility models that interpolate a set of points to generate a profile
between them. In this script these are used in two occasions:

#. To constrain the motion of the joints of the deployable arm.
#. To define the thrust of the thrusters.

For the first case, we use a linear interpolation, so that the joints move
at a constant rate. For the second case, we use a piecewise interpolation,
so that the thrusters are turned on and off at specific times.

The datapoints for each interpolator are defined in the dictionaries
``JOINT_INTERPOLATION_POINTS`` and ``ACTUATOR_INTERPOLATION_POINTS``. Each
interpolator takes a numpy matrix with >=2 columns, where the first column
is the time at which the point is defined, and the rest of the columns
are the values of the points. In this script we demonstrate the use
of ``mujoco.SingleActuatorInterpolator`` and ``mujoco.ScalarJointStateInterpolator``,
both of which need only 2 columns.

Note that the interpolator models are added to the dynamics task, so that
the interpolation happens at every integrator step. All models in the dynamics
task are updated at every integrator step. If we had added the models to the
regular task, the interpolation would only happen at the task rate, which means
that the thrust/joint values would only be updated at the fixed rate of 1 Hz.
Whether you add a model to the dynamics task or the regular task depends on
the specific use case.
"""

import os

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.simulation import svIntegrators

import numpy as np
import matplotlib.pyplot as plt

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_deployable_thruster.xml"

# These are the instant at which each action ends. For example,
# ARM_3_DEPLOY ends at t=20s. FIRST_THRUST ends at t=52.5 s, etc.
START = 0 # s
ARM_1_DEPLOY = 5 # s
ARM_2_DEPLOY = ARM_1_DEPLOY + 10 # s
ARM_3_DEPLOY = ARM_2_DEPLOY + 5 # s
FIRST_THRUST = ARM_3_DEPLOY + 32.5 # s
FIRST_ROLL = FIRST_THRUST + 10 # s
SECOND_THRUST = FIRST_ROLL + 30 # s
SECOND_ROLL = SECOND_THRUST + 5 # s
ARM_4_DEPLOY = SECOND_ROLL + 5 # s
THIRD_THRUST = ARM_4_DEPLOY + 30 # s
END = THIRD_THRUST # s

# This dictionary maps the joint name to the body it is attached to.
BODY_OF_JOINT = {
    "arm_1_elev": "arm_1",
    "arm_2_elev": "arm_2",
    "arm_3_elev": "arm_3",
    "arm_4_elev": "arm_4",
    "arm_1_roll": "arm_1",
}

# These are the points that define the interpolation for the thrusters.
ACTUATOR_INTERPOLATION_POINTS = {
    "thruster_1": [
        [macros.sec2nano(0), 0],
        [macros.sec2nano(ARM_3_DEPLOY), 2],
        [macros.sec2nano(FIRST_THRUST), 0],
        [macros.sec2nano(FIRST_ROLL), 2],
        [macros.sec2nano(SECOND_THRUST), 0],
        [macros.sec2nano(SECOND_ROLL), 2],
    ],
    "thruster_2": [
        [macros.sec2nano(0), 0],
        [macros.sec2nano(ARM_4_DEPLOY), 2],
    ],
}

# These are the points that define the interpolation for the joints.
JOINT_INTERPOLATION_POINTS = {
    "arm_1_elev": [
        [macros.sec2nano(START), np.deg2rad(0)],
        [macros.sec2nano(ARM_1_DEPLOY), np.deg2rad(45)],
    ],
    "arm_2_elev": [
        [macros.sec2nano(ARM_1_DEPLOY), np.deg2rad(180)],
        [macros.sec2nano(ARM_2_DEPLOY), np.deg2rad(74.93)],
    ],
    "arm_3_elev": [
        [macros.sec2nano(ARM_2_DEPLOY), np.deg2rad(180)],
        [macros.sec2nano(ARM_3_DEPLOY), np.deg2rad(60.06941955)],
    ],
    "arm_1_roll": [
        [macros.sec2nano(FIRST_THRUST), np.deg2rad(0)],
        [macros.sec2nano(FIRST_ROLL), np.deg2rad(180)],
        [macros.sec2nano(SECOND_THRUST), np.deg2rad(180)],
        [macros.sec2nano(SECOND_ROLL), np.deg2rad(90)],
    ],
    "arm_4_elev": [
        [macros.sec2nano(SECOND_ROLL), np.deg2rad(180)],
        [macros.sec2nano(ARM_4_DEPLOY), np.deg2rad(90)],
    ],
}


def run(showPlots: bool = False, visualize: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        visualize (bool, optional): If True, the ``MJScene`` visualization tool is
            run on the simulation results. Defaults to False.
    """
    dt = 1 # s

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the MJScene (MuJoCo DynamicObject)
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)

    # Creat the thrust interpolators to define the piecewise thrust profile
    modelsActuator = []
    for thrusterName, interpPoint in ACTUATOR_INTERPOLATION_POINTS.items():

        act: mujoco.MJSingleActuator = scene.getSingleActuator(thrusterName)

        # mujoco.SingleActuatorInterpolator has a single output message,
        # which is interpolatedOutMsg and is of type SingleActuatorMsg.
        # The interpolated scalar is set on the 'input' field of the message.
        model = mujoco.SingleActuatorInterpolator()

        # We pass the data points to the interpolator model.
        # The second argument sets the order for the interpolation.
        # 0 indicates a piecewise interpolation.
        model.setDataPoints(np.array(interpPoint), 0)

        act.actuatorInMsg.subscribeTo(model.interpolatedOutMsg)

        # Note that the model, which is a regular ``SysModel``, is
        # added to the dynamics task of the dynamic object.
        # The optional second argument of this function is the priority
        # of the model in the task. This is similar to the behavior of the
        # ``AddModelToTask`` function of the simulation.
        scene.AddModelToDynamicsTask(model)
        modelsActuator.append(model)

    # Create the joint interpolators to define the motion of the joints
    modelsJoint = []
    for jointName, interpPoint in JOINT_INTERPOLATION_POINTS.items():

        # We can access the joint object by getting the body that
        # the joint is attached to, and then getting the joint object
        joint: mujoco.MJScalarJoint = scene.getBody(
            BODY_OF_JOINT[jointName]
        ).getScalarJoint(jointName)

        # mujoco.ScalarJointStateInterpolator has a single output message,
        # which is interpolatedOutMsg and is of type ScalarJointStateMsg.
        # The interpolated scalar is set on the 'state' field of the message.
        model = mujoco.ScalarJointStateInterpolator()
        model.setDataPoints(np.array(interpPoint), 1)

        joint.constrainedStateInMsg.subscribeTo(model.interpolatedOutMsg)

        # Note that the model, which is a regular ``SysModel``, is
        # added to the dynamics task of the dynamic object.
        # The optional second argument of this function is the priority
        # of the model in the task. This is similar to the behavior of the
        # ``AddModelToTask`` function of the simulation.
        scene.AddModelToDynamicsTask(model)
        modelsJoint.append(model)

    # Record the state of the 'hub' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Record the minimal coordinates of the entire scene for visualization
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # Initialize the simulation and set the initial angles of the joints.
    # NOTE: the simulation MUST be initialized before setting the initial
    # state of the joints. This includes the initial position, attitude,
    # velocity, or angular velocity of the freefloating bodies.
    scSim.InitializeSimulation()

    for jointName, interpPoint in JOINT_INTERPOLATION_POINTS.items():
        initialAngle = interpPoint[0][1]
        joint: mujoco.MJScalarJoint = scene.getBody(
            BODY_OF_JOINT[jointName]
        ).getScalarJoint(jointName)
        joint.setPosition(initialAngle)

    # Run the simulation
    scSim.ConfigureStopTime(macros.sec2nano(END))
    scSim.ExecuteSimulation()

    if showPlots:

        # Plot the attitude of the 'hub' body
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

    # Visualize the simulation
    if visualize:
        speedUp = 3  # Run the visualization at 3x speed
        mujoco.visualize(
            stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, speedUp
        )


if __name__ == "__main__":
    run(True, True)
