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

This scenario showcases how solar-radiation pressure (SRP) can be modeled
ona spacecraft with deployable panels. This script uses the MuJoCo-based
:ref:`DynamicObject<dynamicObject>` :ref:`MJScene<MJScene>`.

The multi-body system is defined in the XML file ``sat_w_deployable_panels.xml``.
In ``mujoco/scenarioDeployPanels.py`` we saw how to use a PD controller to compute
the torque necessary to deploy the panels of this same multi-body system. In
this script, we opt for constraining the panel joint angles to guarantee a smooth
deployment.

The formula for the magnitude of SRP acting on a flat surface can be written as:

::

    SRP_force_mag = C_SRP * cos(incidence_angle)**2

where ``C_SRP`` is a fixed* coefficient and `incidence_angle` is the angle between
the sunlight vector and the normal vector to the surface. The direction of this
force is along the normal to the surface, and in the direction of the sunlight (
imagine the sunrays 'pushing' the surface). The point of application is the
centroid of the surface (akin to regular pressure).

Modeling this with the tools we have considered in previous scenarios should be
straightforward! Since the thrust direction and point of application are fixed
on the body-fixed reference frame, we can use an single (scalar) actuator applied
on a 'site' in the body (recall how thrusters were modeled the same way). To compute
the ``incidence_angle``, we need the direction of sunlight in the inertial frame
(we assume this is constant during the simulation) and the direction of the normal
vector to the surface in the inertial frame. Fortunately, all sites (remember, these
are similar to frames fixed on a body) publish their state, which includes their
attitude with respect to the inertial frame. If we know the definition of the
surface normal in the site reference frame (which is constant since both the surface
normal and site frame are fixed in the same rigid body), then we can easily compute
the direction of the surface normal in the inertial frame.

The :ref:`SysModel<sys_model>` class ``SRPPanel`` performs this calculation for a single panel.
This accepts three class attributes: ``C_SRP``, the direction of sunlight in the
inertial frame, and the normal vector in the site-fixed reference frame.
It also takes an input message, which defines the state of said site frame (i.e.
it's attitude, among other information). Finally, it outputs a message with
the magnitude of the SRP force. This can, in turn, be tied to the control input
of a (single/scalar) actuator that acts on the same site, and with the ``gear``
attribute that corresponds to the frame-fixed surface normal defined in ``SRPPanel``.

We can create a ``SRPPanel`` for every single facet for which we want to model
the SRP force. In this script, we apply it to the sunlit sides of the 6 panels.
However, we could also apply the same model for the sunlit side of our CubeSat.
We could even implement a system to detect when a surface is in shadow or sunlit,
and then apply the ``SRPPanel`` to every surface of our spacecraft (all sides of
our CubeSat, both sides of the panels, etc.).

This script is a good example of how :ref:`MJScene<MJScene>`-based simulations are flexible
and require less modeling effort. The dynamic engine takes care of complicated
frame transformations, so we can retrieve the state of arbitrary frames defined
in the multi-body and apply forces/torques in local reference frames.
It also takes care of computing inter-body forces to meet the defined joint
types, joint limits, constraints, etc. Because all bodies, sites, joints, and
actuators are treated equally by the engine, we obtain a very modular architecture
that reduces development effort.

Note: * There are many reasons why ``C_SRP`` may not actually be constant in reality
(distance from sun, eclipses, material degradation...). This is a simplification!
"""

import os
from typing import Any
from time import perf_counter
from contextlib import contextmanager

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import sysModel

from Basilisk.utilities import RigidBodyKinematics as rbk


import numpy as np

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_deployable_panels.xml"

JOINT_PROFILES = (
    [
        [macros.sec2nano(0), np.pi / 2], # nanoseconds, rad
        [macros.sec2nano(60), 0], # nanoseconds, rad
    ],
    [
        [macros.sec2nano(0), np.pi], # nanoseconds, rad
        [macros.sec2nano(60), 0], # nanoseconds, rad
    ],
    [
        [macros.sec2nano(0), np.pi], # nanoseconds, rad
        [macros.sec2nano(60), 0], # nanoseconds, rad
    ],
)


@contextmanager
def catchtime():
    """Used to measure the time that the context takes to run."""
    tic = toc = perf_counter()
    yield lambda: toc - tic
    toc = perf_counter()


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

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    task = scSim.CreateNewTask("test", macros.sec2nano(1))
    process.addTask(task)

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integ)

    # We want to record the SRP force generated, which is computed
    # in a model in the dynamics task of the ``MJScene``. Thus, we
    # add an extra call to all models in this task to ensure that
    # the SRP model gets called with the finished dynamical states.
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

    # We constrain the joint angle of the panels to follow a linear
    # profile from stowed to deployed position.
    jointAngleInterpolators = {}
    for side, i in panelIds:
        jointAngleInterpolator = mujoco.ScalarJointStateInterpolator()
        jointAngleInterpolator.setDataPoints(np.array(JOINT_PROFILES[i - 1]), 1)

        bodyName = f"panel_{side}{i}"
        jointName = f"{bodyName}_deploy"
        joint: mujoco.MJScalarJoint = scene.getBody(bodyName).getScalarJoint(jointName)

        joint.constrainedStateInMsg.subscribeTo(
            jointAngleInterpolator.interpolatedOutMsg
        )

        scene.AddModelToDynamicsTask(jointAngleInterpolator)
        jointAngleInterpolators[(side, i)] = jointAngleInterpolator

    # We create a model to simulate solar radiation pressure force
    # on each of the panels.
    srpModels = {}
    srpForceRecorders = {}
    for side, i in panelIds:

        srpModel = SRPPanel(srpFactor=1)
        srpModel.ModelTag = f"SRP_{side}{i}"

        # The normal vector of the panel in the body-fixed frame
        # is along the positive z-axis.
        srpModel.normalVector_B = [0, 0, 1]

        # We assume that sunlight is comming from the sun positioned
        # in the infinite positive x-axis direction (so the sunlight
        # direction in the inertial frame is along the negative z-axis).
        # A better simulation would use SPICE to compute this direction.
        srpModel.sunlightDirection_N = [0, 0, -1]

        # The site 'panel_X_centroid' is placed on the centroid of the
        # panel and with frame such that the positive z-axis is normal
        # to the panel normal.
        site: mujoco.MJSite = scene.getSite(f"panel_{side}{i}_centroid")
        srpModel.frameInMsg.subscribeTo(site.stateOutMsg)

        # The SRP actuator is a single (scalar) actuator, since the
        # direction of the force is fixed (perpendicular to the panel)
        srpActuator: mujoco.MJSingleActuator = scene.getSingleActuator(
            f"panel_{side}{i}_srp"
        )
        srpActuator.actuatorInMsg.subscribeTo(srpModel.forceOutMsg)

        # The SRP model is added to the dynamics task because we want
        # the SRP force to be computed at every integrator step. Models
        # that produce force/torques that somehow depend on any state
        # of the system should always be added to the dynamics task.
        # In the case of the SRP, the orientation of the panels influences
        # the magnitude of the SRP force, so this model must be added
        # to the dynamics task.
        scene.AddModelToDynamicsTask(srpModel)

        # Recorders should go in the non-dynamics task
        # We set ``scene.extraEoMCall = True`` to ensure the SRP model gets
        # an extra call to evaluate the force once the dynamics have been
        # integrated, so that they can be recorded through this model.
        recorder = srpModel.forceOutMsg.recorder()
        scSim.AddModelToTask("test", recorder)

        # Cache so these don't go out scope
        srpModels[(side, i)] = srpModel
        srpForceRecorders[(side, i)] = recorder

    # Record the state of the 'hub' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    hubBody: mujoco.MJBody = scene.getBody("hub")
    scStateRecorder = hubBody.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", scStateRecorder)

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
            JOINT_PROFILES[i - 1][0][1]
        )

    # We can only set the attitude rate of the hub because it's
    # a free-floating body.
    if initialSpin:
        scene.getBody("hub").setAttitudeRate([0, 0.8, 0]) # rad/s

    # Run the simulation
    with catchtime() as executeTime:
        # Run for 60 seconds, after which the panels should be fully deployed
        scSim.ConfigureStopTime(macros.sec2nano(59))
        scSim.ExecuteSimulation()

        # Adjust the task rate so that we get higher resolution of data points
        # for a couple of interesting seconds
        task.TaskData.updatePeriod(macros.sec2nano(0.025))
        scSim.ConfigureStopTime(macros.sec2nano(61))
        scSim.ExecuteSimulation()

        # Finish simulation at t=70 with original test rate
        task.TaskData.updatePeriod(macros.sec2nano(1))
        scSim.ConfigureStopTime(macros.sec2nano(70))
        scSim.ExecuteSimulation()

    print("Run sim in", executeTime(), " seconds")

    # Draw plots from recorded values
    if showPlots:
        import matplotlib.pyplot as plt

        # Plot the SRP force on each panel
        fig, axs = plt.subplots(2, 3)
        for ax, (side, i) in zip(axs.flat, panelIds):

            rec = srpForceRecorders[(side, i)]

            ax.plot(rec.times() * macros.NANO2SEC, rec.input)

            ax.set_title(f"Panel {side}{i}")
            ax.set_xticks([])
            ax.set_xlabel("Time [sec]")
            ax.set_ylabel("Force [N]")

        fig.suptitle("Solar Radiation Pressure Force")
        fig.tight_layout()

        # Plot the vertical (along z-axis) velocity of the spacecraft
        _, ax = plt.subplots()
        ax.plot(
            scStateRecorder.times() * macros.NANO2SEC,
            scStateRecorder.v_BN_N[:, 2],
            marker=".",
        )
        ax.set_xlabel("Time [sec]")
        ax.set_ylabel("Z-axis Velocity [m/s]")

        plt.show()

    # Visualize the simulation
    if visualize:
        speedUp = 1
        mujoco.visualize(
            stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, speedUp
        )


class SRPPanel(sysModel.SysModel):
    """A simple ``SysModel`` that simulates the Solar Radiation
    Pressure (SRP) force that would act on a flat plate.
    """

    def __init__(self, srpFactor: float, *args: Any):
        """
        Args:
            srpFactor (float): The magnitude of the SRP force
                when the sunlight direction and panel normal
                are parallel.
        """
        super().__init__(*args)

        self.srpFactor = srpFactor
        self.normalVector_B = [0, 0, 1]
        self.sunlightDirection_N = [0, 0, -1]

        self.frameInMsg = messaging.SCStatesMsgReader()

        self.forceOutMsg = messaging.SingleActuatorMsg()

    def UpdateState(self, CurrentSimNanos: int):
        """Called at every integrator step to compute the
        SRP force magnitude.
        """
        # N frame: inertial frame
        # B frame: body-fixed frame
        frame: messaging.SCStatesMsgPayload = self.frameInMsg()
        dcm_NB = rbk.MRP2C(frame.sigma_BN).T
        normalVector_N = np.dot(dcm_NB, self.normalVector_B)
        cosAngle = np.dot(-normalVector_N, self.sunlightDirection_N)
        forceMagnitude = self.srpFactor * cosAngle**2

        payload = messaging.SingleActuatorMsgPayload()
        payload.input = forceMagnitude
        self.forceOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(False)
