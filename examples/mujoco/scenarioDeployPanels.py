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

This script demonstrates how to simulate a spacecraft with solar panels deployed
using a Proportional-Integral-Derivative (PID) controller. This script uses the
MuJoCo-based :ref:`DynamicObject<dynamicObject>` :ref:`MJScene<MJScene>`.

In ``mujoco/scenarioArmWithThrusters.py``, we saw how we can constrain joints to
follow a specific angle by letting the dynamic engine figure out and apply
the necessary torques. In this script, we are controlling the joints using a
PID controller. This is a more adequate simulation setup when you want to simulate
or design the control system for these joints. It is also generally more
computationally efficient than letting the dynamic engine figure out the torques.

The multi-body system is defined in the XML file ``sat_w_deployable_panels.xml``.
This XML file defines a 'hub' body and six deployable solar panels. The panels
are attached to the 'hub' body via revolute joints, allowing them to rotate
from a stowed position to a deployed position.

This XML file (and this script) demonstrates the use of the ``range`` attribute
in the ``joint`` element. This attribute allows the joint to be limited to a
specific range of motion. For example, between 0 and 90 or 180 degrees. If the
joint tries to move beyond this range, the dynamic engine will apply a force to
prevent it from doing so (akin to a physical stop on the system). Note in the
plots and in the 3D visualization how the panels never get deployed over their
joint limit.

The deployment of the panels is controlled using an analog PID controller.
The desired position and velocity profiles for the joints are
generated using a trapezoidal/triangular velocity profile. These profiles are
then used as inputs to the PID controller, which computes the torque required to achieve
the desired motion. Note that the controller class extends ``StatefulSysModel``,
instead of ``SysModel``, since we need to register the integral error as a
continuous state.

The simulation is run for 80 minutes and the state of the system is recorded.
The desired and achieved joint angles, as well as the torque applied to each
joint, are plotted. The system can also be visualized in a 3D environment using
the ``mujoco.visualize`` function.
"""

import os
from typing import Any, List
from time import perf_counter
from contextlib import contextmanager

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.simulation import StatefulSysModel
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators

import numpy as np

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_deployable_panels.xml"

JOINT_START_END = [(np.pi / 2, 0), (np.pi, 0), (np.pi, 0)] # rad
MAX_PROFILE_VEL = np.deg2rad(0.05) # rad
MAX_PROFILE_ACCEL = np.deg2rad(0.0001) # rad


@contextmanager
def catchtime():
    tic = toc = perf_counter()
    yield lambda: toc - tic
    toc = perf_counter()


def generateProfiles(
    initialPoint: float, finalPoint: float, vMax: float, aMax: float
) -> List[mujoco.ScalarJointStateInterpolator]:
    """
    Generate a position and velocity profile for a point-to-point movement.

    This function calculates the position and velocity profiles for a point-to-point
    movement using either a trapezoidal or triangular velocity profile, depending on
    whether the maximum velocity is reached or not.

    Args:
        initialPoint (float): The starting position.
        finalPoint (float): The target position.
        vMax (float): The maximum velocity.
        aMax (float): The maximum acceleration.
    Returns:
        list: A list containing two mujoco.ScalarJointStateInterpolator objects,
              one for the position profile and one for the velocity profile.
    """

    distance = np.abs(finalPoint - initialPoint)

    # Time to reach vMax
    tAccel = vMax / aMax

    # Distance covered during acceleration (and deceleration, which is symmetrical)
    dAccel = 0.5 * aMax * tAccel**2

    # Check if we reach vMax or if we have a triangular profile
    if 2 * dAccel > distance:
        # Triangular profile: maximum velocity not reached
        tAccel = np.sqrt(distance / aMax)
        tConst = 0
    else:
        # Trapezoidal profile: maximum velocity reached
        dConst = distance - 2 * dAccel
        tConst = dConst / vMax

    # Total time
    tTotal = 2 * tAccel + tConst

    # Time points for plotting
    time = np.linspace(0, tTotal, 100)
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)

    # Compute velocity and position at each time point
    for i, t in enumerate(time):
        if t < tAccel:
            # Acceleration phase
            velocity[i] = aMax * t
            position[i] = 0.5 * aMax * t**2
        elif t < tAccel + tConst:
            # Constant velocity phase
            velocity[i] = vMax
            position[i] = dAccel + vMax * (t - tAccel)
        else:
            # Deceleration phase
            tDecel = t - tAccel - tConst
            velocity[i] = vMax - aMax * tDecel
            position[i] = (
                dAccel + vMax * tConst + (vMax * tDecel - 0.5 * aMax * tDecel**2)
            )

    if finalPoint > initialPoint:
        position = initialPoint + position
    else:
        position = initialPoint - position
        velocity = -velocity

    interpolators = []
    for values in (position, velocity):
        # The interpolatorPoints define the trapzoidal/triangular profile.
        # We pass this to a ``mujoco.ScalarJointStateInterpolator`` so that
        # the simulation time is mapped to the position/velocity by linearly
        # interpolating through the generated profile data points.
        interpolatorPoints = np.column_stack([time * 1e9, values])
        interpolator = mujoco.ScalarJointStateInterpolator()
        interpolator.setDataPoints(interpolatorPoints, 1)
        interpolators.append(interpolator)

    return interpolators


def run(initialSpin: bool = False, showPlots: bool = False, visualize: bool = False):
    """Main function, see scenario description.

    Args:
        initialSpin (bool, optional): If True, the satellite is given an initial
            angular velocity. Defaults to False.
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        visualize (bool, optional): If True, the ``MJScene`` visualization tool is
            run on the simulation results. Defaults to False.
    """

    dt = 1 # s
    tf = 80 * 60 # s

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)

    # Enable an additional call to all models in the dynamics task
    # once integration has finished. This will have no impact on the
    # dynamics, but it will give a chance to these models to get called
    # with the correct/final state of the multi-body. This is useful
    # when some of the models produce output messages that depend on this
    # state, and other models use these output messages.
    # In this case, we want recorders of the desired joint angle of each
    # panel (which is computed by an interpolator model in the dynamics task)
    # and the torque output of the controller (also computed in the dynamics task).
    scene.extraEoMCall = True

    # We have 6 panels: 3 along positive x-axis and 3 along negative x-axis
    # They are numbered from 1 to 3 (each side, from inside to outside)
    panelIds = [
        ("p", 1),
        ("p", 2),
        ("p", 3),
        ("n", 1),
        ("n", 2),
        ("n", 3),
    ]

    # We need to cache the models so they don't go out of scope
    controllerModels = {}
    positionInterpolatorModels = {}
    velocityInterpolatorModels = {}
    desiredPosRecorderModels = {}
    measuredPosRecorderModels = {}
    torqueRecorderModels = {}

    for side, i in panelIds:
        actuatorName = f"panel_{side}{i}_deploy"
        act: mujoco.MJSingleActuator = scene.getSingleActuator(actuatorName)

        bodyName = f"panel_{side}{i}"
        body: mujoco.MJBody = scene.getBody(bodyName)

        jointName = f"panel_{side}{i}_deploy"
        joint: mujoco.MJScalarJoint = body.getScalarJoint(jointName)

        # Generate the position and velocity profiles for the joint
        positionInterpolator, velocityInterpolator = generateProfiles(
            JOINT_START_END[i - 1][0],
            JOINT_START_END[i - 1][1],
            MAX_PROFILE_VEL,
            MAX_PROFILE_ACCEL,
        )

        # Create the PD controller for each joint. The inputs to this
        # controller are the desired position and velocity of the joint,
        # the measured position and velocity of the joint (in this case the
        # exact values are used, but in a real system these may be the product
        # of a sensor), and the output is the torque to be applied to the joint.
        pdController = PIDController()
        pdController.ModelTag = f"{actuatorName}_controller"

        # Connect the interpolators to the PD controller for the desired
        # position and velocity
        pdController.desiredInMsg.subscribeTo(positionInterpolator.interpolatedOutMsg)
        pdController.desiredDotInMsg.subscribeTo(
            velocityInterpolator.interpolatedOutMsg
        )

        # Connect the PD controller to the joint for the 'measured' position and velocity
        pdController.measuredInMsg.subscribeTo(joint.stateOutMsg)
        pdController.measuredDotInMsg.subscribeTo(joint.stateDotOutMsg)

        # Connect the PD controller output to the actuator torque input
        act.actuatorInMsg.subscribeTo(pdController.outputOutMsg)

        # Recprders for plotting
        desiredPosRecorder = pdController.desiredInMsg.recorder()
        measuredPosRecorder = pdController.measuredInMsg.recorder()
        torqueRecorder = pdController.outputOutMsg.recorder()

        # Add the models to the dynamics task
        # The priority is important because the PD controller needs to be updated
        # after the interpolators.
        scene.AddModelToDynamicsTask(positionInterpolator, 50)
        scene.AddModelToDynamicsTask(velocityInterpolator, 49)
        scene.AddModelToDynamicsTask(pdController, 25)

        # Add the recorders to the non-dynamics task!
        # We don't actually want to record these values at every integrator
        # step, since those values are 'unfinished' (integrators are free
        # to set arbitrary/incorrect states on which to evaluate the dynamics).
        # Instead, we want to record these numbers after the dynamics of the
        # MJScene have been resolved (the states are finished/correct).
        # This is why we set ``scene.extraEoMCall = True`` above.
        scSim.AddModelToTask("test", desiredPosRecorder)
        scSim.AddModelToTask("test", measuredPosRecorder)
        scSim.AddModelToTask("test", torqueRecorder)

        # Cache the models so they don't go out of scope
        controllerModels[(side, i)] = pdController
        positionInterpolatorModels[(side, i)] = positionInterpolator
        velocityInterpolatorModels[(side, i)] = velocityInterpolator
        desiredPosRecorderModels[(side, i)] = desiredPosRecorder
        measuredPosRecorderModels[(side, i)] = measuredPosRecorder
        torqueRecorderModels[(side, i)] = torqueRecorder

    # Record the minimal coordinates of the entire scene for visualization
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # Initialize the simulation and set the initial angles of the joints.
    # NOTE: the simulation MUST be initialized before setting the initial
    # joint angles and setting the initial attitude rate of the hub.
    scSim.InitializeSimulation()

    # Set the initial angles of the joints
    for side, i in panelIds:
        bodyName = f"panel_{side}{i}"
        jointName = f"panel_{side}{i}_deploy"
        scene.getBody(bodyName).getScalarJoint(jointName).setPosition(
            JOINT_START_END[i - 1][0]
        )

    # We can only set the attitude rate of the hub because it's
    # a free-floating body. You can also set the position, velocity,
    # and attitude of any free-floating bodies. Moreover, you can do
    # this at any time during the simulation.
    if initialSpin:
        scene.getBody("hub").setAttitudeRate([0, 0.8, 0]) # rad/s

    # Configure the stop time of the simulation
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Run the simulation
    with catchtime() as executeTime:
        scSim.ExecuteSimulation()
    print("Run sim in", executeTime(), " seconds")

    # Draw plots from recorded values
    if showPlots:
        import matplotlib.pyplot as plt

        # Plot the desired and achieved joint angle for each panel
        fig, axs = plt.subplots(2, 3)
        for ax, (side, i) in zip(axs.flat, panelIds):

            desiredPosRecorder = desiredPosRecorderModels[(side, i)]
            measuredPosRecorder = measuredPosRecorderModels[(side, i)]

            ax.plot(
                desiredPosRecorder.times() * macros.NANO2SEC,
                np.rad2deg(desiredPosRecorder.state),
                "-",
                label="Desired Profile" if ax is axs.flat[0] else None,
            )
            ax.plot(
                measuredPosRecorder.times() * macros.NANO2SEC,
                np.rad2deg(measuredPosRecorder.state),
                "--",
                label="Achieved" if ax is axs.flat[0] else None,
            )
            ax.set_title(f"Panel {side}{i}")
            ax.set_xticks([])
            ax.set_ylabel("Angle [deg]")
            ax.set_xlabel("Time [s]")

        fig.suptitle("Panel angle")
        fig.legend(loc="outside lower center", ncols=2)
        fig.tight_layout()

        # Plot the torque applied to each panel
        fig, axs = plt.subplots(2, 3)
        for ax, (side, i) in zip(axs.flat, panelIds):
            torqueRecorder = torqueRecorderModels[(side, i)]
            ax.plot(torqueRecorder.times() * macros.NANO2SEC, torqueRecorder.input)
            ax.set_title(f"Panel {side}{i}")
            ax.set_ylabel("Torque [Nm]")
            ax.set_xlabel("Time [s]")

        plt.show()

    # Visualize the simulation
    if visualize:
        speedUp = 120
        mujoco.visualize(
            stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, speedUp
        )


# The following is an example of a Python-based SysModel that
# can be added to the dynamics task of a MJScene.
class PIDController(StatefulSysModel.StatefulSysModel):
    """
    A Proportional-Integral-Derivative (PID) Controller class for controlling joint states.

    This models an analog PID controller, which means that its output evolves in continuous
    time, not discrete time. Thus, it should be used within the dynamics task of ``MJScene``.

    Attributes:
        K_p (float): Proportional gain.
        K_d (float): Derivative gain.
        K_i (float): Integral gain.

        measuredInMsg (messaging.ScalarJointStateMsgReader): Reader for the measured joint state.
        desiredInMsg (messaging.ScalarJointStateMsgReader): Reader for the desired joint state.
        measuredDotInMsg (messaging.ScalarJointStateMsgReader): Reader for the measured joint state derivative.
        desiredDotInMsg (messaging.ScalarJointStateMsgReader): Reader for the desired joint state derivative.

        outputOutMsg (messaging.SingleActuatorMsg): Output message, contains the torque for the connected actuator.
    """

    def __init__(self, *args: Any):
        """Initialize"""
        super().__init__(*args)
        self.K_p = 0.1
        self.K_d = 0.002
        self.K_i = 0.0001

        self.measuredInMsg = messaging.ScalarJointStateMsgReader()
        self.desiredInMsg = messaging.ScalarJointStateMsgReader()

        self.measuredDotInMsg = messaging.ScalarJointStateMsgReader()
        self.desiredDotInMsg = messaging.ScalarJointStateMsgReader()

        self.outputOutMsg = messaging.SingleActuatorMsg()

    def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
        self.integralErrorState = registerer.registerState(1, 1, "integralError")
        self.integralErrorState.setState([[0]]) # explicitely zero initialize

    def UpdateState(self, CurrentSimNanos: int):
        """Computes the control command from the measured and desired
        joint position and velocity."""
        # Compute the error in the state and its derivative
        stateError = self.desiredInMsg().state - self.measuredInMsg().state
        stateDotError = self.desiredDotInMsg().state - self.measuredDotInMsg().state
        stateIntegralError = self.integralErrorState.getState()[0][0]

        # Compute the control output
        control_output = self.K_p * stateError + self.K_d * stateDotError + self.K_i * stateIntegralError

        # Set the derivative of the integral error inner state
        self.integralErrorState.setDerivative([[stateError]])

        # Write the control output to the output message
        payload = messaging.SingleActuatorMsgPayload()
        payload.input = control_output
        self.outputOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(False, True, True)
