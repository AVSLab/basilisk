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

#. ``examples/mujoco/scenarioDeployPanels.py``

This script demonstrates how to simulate a spacecraft with a
*branching* solar panel deployment sequence using a Proportional-
Integral-Derivative (PID) controller. Like
``scenarioDeployPanels.py``, this scenario uses the MuJoCo-based
:ref:`DynamicObject<dynamicObject>` :ref:`MJScene<MJScene>`, but
extends the concept to a more complex, multi-stage deployment
process.

In ``mujoco/scenarioDeployPanels.py``, all solar panels were deployed
simultaneously using PID controllers and trapezoidal/triangular
velocity profiles. In this scenario, the panels are deployed in a
staged sequence, where the deployment of some panels happens after
the deployment of others. Moreover, this scenario uses the C++
implementation of the PID controller (`MJPIDControllers.JointPIDController`),
which provides improved performance compared to the previous
Python-based controller.

The multi-body system is defined in the XML file
``sat_w_branching_panels.xml``, which describes a spacecraft hub with
six deployable panels arranged in a branching configuration. Each
panel is attached via a revolute joint, and the deployment sequence is
controlled by locking and unlocking joints at specific times,
simulating mechanical latches.

The deployment profiles for each joint are generated using a
trapezoidal/triangular velocity profile, as in the previous scenario.
However, the timing of each deployment is offset to create the desired
branching sequence. The PID controllers for each joint receive these
profiles as their reference inputs and compute the required torques to
achieve the commanded motion.

During the simulation, joints are locked or unlocked at runtime to
enforce the staged deployment sequence. This is accomplished by
sending constraint messages to the relevant joints, effectively
freezing or releasing their motion as needed.

The simulation is run in three legs, corresponding to the sequential
deployment of different panel groups. The state of the system is
recorded throughout, and the desired and achieved joint angles are plotted
for each panel. The system can also be visualized in 3D using
the ``mujoco.visualize`` function.

This scenario illustrates how to model and control complex, staged
deployment mechanisms in a spacecraft simulation, and how to
coordinate multiple actuators and controllers in a branched sequence
using MuJoCo and Basilisk.
"""

import os
from typing import List
from time import perf_counter
from contextlib import contextmanager

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.simulation import StatefulSysModel
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import MJPIDControllers

import numpy as np

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_branching_panels.xml"

@contextmanager
def catchtime():
    tic = toc = perf_counter()
    yield lambda: toc - tic
    toc = perf_counter()

BLUE = "#004488"
YELLOW = "#DDAA33"
RED = "#BB5566"

def generateProfiles(
    initialPoint: float, finalPoint: float, vMax: float, aMax: float, timeOffset: int = 0
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
        timeOffset (int): The profile will start at this time (in nanoseconds).
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
        interpolatorPoints = np.column_stack([timeOffset + time * 1e9, values])
        interpolator = mujoco.ScalarJointStateInterpolator()
        interpolator.setDataPoints(interpolatorPoints, 1)
        interpolators.append(interpolator)

    return interpolators

def run(showPlots: bool = False, visualize: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        visualize (bool, optional): If True, the ``MJScene`` visualization tool is
            run on the simulation results. Defaults to False.
    """

    dt = macros.sec2nano(10)
    operationTime = macros.sec2nano(90*60)

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", dt))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    # integ.setRelativeTolerance(1e-5)
    # integ.setAbsoluteTolerance(1e-6)
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

    # Get all joints of interest, save them in a dictionary for easy access
    joints: dict[str, mujoco.MJScalarJoint] = {}
    for panelID in ["10", "1p", "1n", "20", "2p", "2n"]:
        bodyName = f"panel_{panelID}"
        body: mujoco.MJBody = scene.getBody(bodyName)

        jointName = f"panel_{panelID}_deploy"
        joints[panelID] = body.getScalarJoint(jointName)

    # We need to cache the models so they don't go out of scope
    controllerModels = {}
    positionInterpolatorModels = {}
    velocityInterpolatorModels = {}
    desiredPosRecorderModels = {}
    measuredPosRecorderModels = {}
    torqueRecorderModels = {}
    jointConstraintMessages = {}

    def addJointController(panelID: str, initialAngle: float, timeOffset: int):
        joint = joints[panelID]

        actuatorName = f"panel_{panelID}_deploy"
        act: mujoco.MJSingleActuator = scene.addJointSingleActuator(actuatorName, joint)

        # Generate the position and velocity profiles for the joint
        positionInterpolator, velocityInterpolator = generateProfiles(
            initialPoint=initialAngle, # rad
            finalPoint=0, # rad
            vMax=np.deg2rad(0.05),
            aMax=np.deg2rad(0.0001),
            timeOffset=timeOffset
        )

        # Use the C++ JointPIDController
        pidController = MJPIDControllers.JointPIDController()
        pidController.ModelTag = f"{actuatorName}_controller"
        pidController.setProportionalGain(0.1)
        pidController.setDerivativeGain(0.002)
        pidController.setIntegralGain(0.0001)

        # Connect the interpolators to the PID controller for the desired
        # position and velocity
        pidController.desiredPosInMsg.subscribeTo(positionInterpolator.interpolatedOutMsg)
        pidController.desiredVelInMsg.subscribeTo(velocityInterpolator.interpolatedOutMsg)

        # Connect the PID controller to the joint for the measured position and velocity
        pidController.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
        pidController.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)

        # Connect the PID controller output to the actuator torque input
        act.actuatorInMsg.subscribeTo(pidController.outputOutMsg)

        # Recprders for plotting
        desiredPosRecorder = pidController.desiredPosInMsg.recorder()
        measuredPosRecorder = pidController.measuredPosInMsg.recorder()
        torqueRecorder = pidController.outputOutMsg.recorder()

        # Add the models to the dynamics task
        # The priority is important because the PD controller needs to be updated
        # after the interpolators.
        scene.AddModelToDynamicsTask(positionInterpolator, 50)
        scene.AddModelToDynamicsTask(velocityInterpolator, 49)
        scene.AddModelToDynamicsTask(pidController, 25)

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
        controllerModels[panelID] = pidController
        positionInterpolatorModels[panelID] = positionInterpolator
        velocityInterpolatorModels[panelID] = velocityInterpolator
        desiredPosRecorderModels[panelID] = desiredPosRecorder
        measuredPosRecorderModels[panelID] = measuredPosRecorder
        torqueRecorderModels[panelID] = torqueRecorder

    def lockJoint(panelID: str, angle: float):
        # create a stand-alone message with the given joint angle
        jointConstraintMsg = messaging.ScalarJointStateMsg()
        jointConstraintMsgPayload = messaging.ScalarJointStateMsgPayload()
        jointConstraintMsgPayload.state = angle # rad
        jointConstraintMsg.write(jointConstraintMsgPayload, 0, -1)

        # connect the message to the `constrainedStateInMsg` input of
        # the joint, which will effectively "lock" the joint to the
        # angle set in `jointConstraintMsg`
        joint = joints[panelID]
        joint.constrainedStateInMsg.subscribeTo(jointConstraintMsg)

        # cache the msg
        jointConstraintMessages[panelID] = jointConstraintMsg

    def unlockJoint(panelID: str):
        # disconnecting `constrainedStateInMsg` is enough to remove
        # the constraint
        joint = joints[panelID]
        joint.constrainedStateInMsg.unsubscribe()

    addJointController("10", initialAngle=np.pi/2, timeOffset=0)
    addJointController("20", initialAngle=np.pi,   timeOffset=0)
    addJointController("1p", initialAngle=np.pi,   timeOffset=operationTime)
    addJointController("2p", initialAngle=np.pi,   timeOffset=operationTime)
    addJointController("1n", initialAngle=np.pi,   timeOffset=2*operationTime)
    addJointController("2n", initialAngle=np.pi,   timeOffset=2*operationTime)

    for panelID in ["1p", "1n", "2p", "2n"]:
        lockJoint(panelID, angle=np.pi)

    # Record the minimal coordinates of the entire scene for visualization
    stateRecorder = scene.stateOutMsg.recorder()
    scSim.AddModelToTask("test", stateRecorder)

    # Initialize the simulation and set the initial angles of the joints.
    # NOTE: the simulation MUST be initialized before setting the initial
    # joint angles.
    scSim.InitializeSimulation()

    # Increase the time constant of the constraint equality to 5 seconds
    # so that the panel locks are "softer". Also, lower the impedance.
    # We do this to make the problem less stiff and thus make the scenario run faster.
    for panelID in ["10", "1p", "1n", "20", "2p", "2n"]:
        eq: mujoco.MJSingleJointEquality = joints[panelID].getConstrainedEquality()
        eq.setSolref(5, 1)
        eq.setSolimp(0.8, 0.95, 0.001, 0.5, 2)

    # Set the initial angles of the joints (stowed configuration)
    for panelID in ["10", "1p", "1n", "20", "2p", "2n"]:
        initialAngle = np.pi/2 if panelID == "10" else np.pi # rad
        joints[panelID].setPosition(initialAngle)

    # Configure the stop time of the simulation
    scSim.ConfigureStopTime(operationTime)

    # Run the simulation
    with catchtime() as executeTime:
        scSim.ExecuteSimulation()
    print("Run leg 1 in", executeTime(), " seconds")

    lockJoint("10", angle=0)
    lockJoint("20", angle=0)
    unlockJoint("1p")
    unlockJoint("2p")

    # Configure the stop time of the simulation
    scSim.ConfigureStopTime(2*operationTime)

    # Run the simulation
    with catchtime() as executeTime:
        scSim.ExecuteSimulation()
    print("Run leg 2 in", executeTime(), " seconds")

    lockJoint("1p", angle=0)
    lockJoint("2p", angle=0)
    unlockJoint("1n")
    unlockJoint("2n")

    # Configure the stop time of the simulation
    scSim.ConfigureStopTime(3*operationTime)

    # Run the simulation
    with catchtime() as executeTime:
        scSim.ExecuteSimulation()
    print("Run leg 3 in", executeTime(), " seconds")

    # Draw plots from recorded values
    if showPlots:
        import matplotlib.pyplot as plt

        # Plot the desired and achieved joint angle for each panel
        fig, axs = plt.subplots(ncols=2, nrows=3, sharex="all", sharey="all", squeeze=False)
        for ax, panelID in zip(axs.flat, ["10", "20", "1p", "2p", "1n", "2n"]):

            desiredPosRecorder = desiredPosRecorderModels[panelID]
            measuredPosRecorder = measuredPosRecorderModels[panelID]

            ax.plot(
                desiredPosRecorder.times() * macros.NANO2MIN,
                np.rad2deg(desiredPosRecorder.state),
                "-",
                color=BLUE,
                lw = 2,
                label="Desired Profile" if ax is axs.flat[0] else None,
            )
            ax.plot(
                measuredPosRecorder.times() * macros.NANO2MIN,
                np.rad2deg(measuredPosRecorder.state),
                "--",
                color=RED,
                lw = 2,
                label="Achieved" if ax is axs.flat[0] else None,
            )
            for line in [1, 2]:
                ax.axvline(line * operationTime * macros.NANO2MIN, color="k", linestyle=":")
            ax.set_title(f"Panel {panelID}")
            ax.set_yticks([0, 45, 90, 135, 180])
            if ax in axs[:,0]:
                ax.set_ylabel("Angle [deg]")
            if ax in axs[-1,:]:
                ax.set_xlabel("Time [min]")

        fig.suptitle("Panel angle")
        fig.legend(loc="outside lower center", ncols=2)
        fig.tight_layout()

        plt.show()

    # Visualize the simulation
    if visualize:
        speedUp = 120
        qpos = np.squeeze(stateRecorder.qpos)
        mujoco.visualize(
            stateRecorder.times(), qpos, scene, speedUp
        )

if __name__ == "__main__":
    run(True, True)
