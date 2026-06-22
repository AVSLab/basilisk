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

This scripts shows how one could simulate a spacecraft landing on
an asteroid. It illustrates force-vector actuators, loading meshes
for bodies, and collisions. This is done using the MuJoCo-based
:ref:`DynamicObject<dynamicObject>` :ref:`MJScene<MJScene>`.

The multi-body system is defined in the XML file ``sat_ast_landing.xml``.
This XML illustrates various interesting features. First, is the use
of the ``<default>`` element to avoid data repetition in the XML file.
Second, it's the use of multiple free-floating bodies, in this case
the 'hub' of the spacecraft and the 'asteroid' body. Multiple independent
spacecraft are also possible (see ``examples/mujoco/scenarioSimpleDocking.py``).
Finally, this showcases how to load a mesh (for the geometry of the
asteroid). The actual file path must then be provided on construction of :ref:`MJScene<MJScene>`.

Unless specifically included, gravity is not modeled in `:ref:`MJScene<MJScene>`
simulations. However, in this scenario we want to simulate a spacecraft
landing on the asteroid, thus we need to model gravity. This is done by
creating a custom ``ConstantGravity`` model that applies a constant force
in the inertial frame. This is obviously a simplification, akin to
considering a constant downward gravity near the surface of the Earth.

Previously seen scenarios have always used actuators defined in
the XML file, and of the `MJSingularActuator` type. ``MJSingularActuator``
are characterized by being controlled by a single scalar value.
For the case of ``MJSingularActuator`` tied to a site, these generate
a force/torque in the local frame of their site. The resulting
force and/or torque vectors are the product of the scalar input of the
``MJSingularActuator`` times the constant direction vectors in the site
reference frame defined by the `gear` attribute of this actuator.

``MJSingularActuator`` are ideal to model forces/torques that always
applied along a fixed direction in the site frame. However, in this scenario,
we need to apply a force in the inertial frame. To do so, we use a
``MJForceActuator``, which allows us to define a force vector with arbitrary
direction and magnitude (thus 3 scalar values are used to control it).
Still, the desired force vector given to this actuator is still in the
site reference frame. To produce a force fixed in the inertial frame,
we need to use the attitude of the site frame to continuously rotate
the force in the site-fixed frame such that it's always acting in the
same inertial direction.

This is implemented through two mechanisms. First, we need to add
a new actuator. Previous scripts retrieved actuators defined in
the XML through ``getSingleActuator``. In this case, we create a
new actuator of the ``MJForceActuator`` type with ``addForceActuator``.
Second, we use the ``ConstantGravity`` model, which accounts for
the rotation of the site frame to produce a force vector that
produces a gravity force that is constant in the inertial frame.

It is possible to add ``MJSingleActuator``, ``MJForceActuator``,
``MJTorqueActuator``, and ``MJForceTorqueActuator`` through Python
or C++ function calls (``addSingleActuator``, ``addForceActuator``, etc.).
These can later be retrieved with ``getSingleActuator``,
``getForceActuator``, etc.

The system can also be visualized in :ref:`Vizard <vizard>` through the
``enableUnityVisualization`` call.
"""

import os
from typing import Any, Sequence

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import vizSupport
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import sysModel

from Basilisk.utilities import RigidBodyKinematics as rbk

import numpy as np
import matplotlib.pyplot as plt

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_ast_landing.xml"

AST_OBJ_PATH = os.path.abspath(
    os.path.join(
        CURRENT_FOLDER,
        "..",
        "dataForExamples",
        "Itokawa",
        "ItokawaHayabusa.obj",
    )
)
AST_TEXTURE_PATH = os.path.abspath(
    os.path.join(
        CURRENT_FOLDER,
        "..",
        "dataForExamples",
        "Itokawa",
        "ItokawaGrayscale.jpg",
    )
)
ASTEROID_BODY_NAME = "asteroid"
ASTEROID_VIZ_SCALE = 1000.0  # [-]
SPACECRAFT_BODY_NAME = "hub"
THRUSTER_NAME = "thrust"
THRUSTER_LOCATION = [0.0, 0.0, -1.0]  # [m]
THRUSTER_DIRECTION = [0.0, 0.0, 1.0]  # [-]
THRUSTER_VIZ_SCALE = 100.0  # [-]


def _get_body_geom_info(scene: mujoco.MJScene, body_name: str):
    """Return the first MuJoCo geom attached to a scene body."""
    geomInfos = scene.getGeomInfos()
    for geomIndex in range(len(geomInfos)):
        geomInfo = geomInfos[geomIndex]
        if geomInfo.bodyName == body_name:
            return geomInfo
    raise ValueError(f"Could not find a MuJoCo geom for body '{body_name}'.")


def _attach_thruster_visualization(viz, spacecraft_name: str, writer):
    """Attach a MuJoCo thruster visualization message to a Vizard body."""
    from Basilisk.simulation import vizInterface

    for scDataIndex in range(len(viz.scData)):
        scData = viz.scData[scDataIndex]
        if scData.spacecraftName != spacecraft_name:
            continue

        thrInfo = vizInterface.ThrClusterMap()
        thrInfo.thrTag = writer.ModelTag
        thrInfo.color = vizSupport.toRGBA255("turquoise")
        scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(
            [writer.thrOutMsg.addSubscriber()]
        )
        scData.thrInfo = vizInterface.ThrClusterVector([thrInfo])
        vizSupport.setActuatorGuiSetting(
            viz,
            spacecraftName=spacecraft_name,
            viewThrusterHUD=True,
        )
        return

    raise ValueError(
        f"Could not find spacecraft '{spacecraft_name}' in Vizard spacecraft data."
    )


def run(showPlots: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
    """
    dt = 0.02  # [s]

    timeThrustTurnOff = 47.5  # [s]
    tf = 70  # [s]

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    task = scSim.CreateNewTask("test", macros.sec2nano(dt))
    process.addTask(task)

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file, and additionally
    # load the asteroid shape file so that it's available for MuJoCo
    scene = mujoco.MJScene.fromFile(XML_PATH, files=[AST_OBJ_PATH])
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to RK4(5)
    integ = svIntegrators.svIntegratorRKF45(scene)
    integ.setRelativeTolerance(1e-3)
    integ.setAbsoluteTolerance(1e-3)
    scene.setIntegrator(integ)

    # Create a ConstantGravity model that imparts 200 N along
    # the negative z-axis in the inertial frame
    gravity = ConstantGravity(force_N=[0.0, 0.0, -200.0])  # [N]
    scene.AddModelToDynamicsTask(gravity)

    # We want to apply the gravity force at this site
    gravityApplicationSite = scene.getBody(SPACECRAFT_BODY_NAME).getOrigin()

    # We create an actuator at the site of interest of the "Force"
    # type, which allow us to apply a force vector in any direction
    # (so, unlike single/scalar actuators, the force is set by three
    # scalars, the three values of the force vector in the site frame).
    gravityActuator: mujoco.MJForceActuator = scene.addForceActuator(
        "hub_gravity", gravityApplicationSite
    )
    gravityActuator.forceInMsg.subscribeTo(gravity.forceOutMsg)

    # In MJScene, the force in a ForceActuator is given in the site-fixed
    # reference frame. However, we want to apply the force in the inertial
    # reference frame. Thus, the ConstantGravity needs the attitude of
    # the site to convert the inertial force vector to the equivalent
    # site-fixed reference frame.
    gravity.frameInMsg.subscribeTo(gravityApplicationSite.stateOutMsg)

    # Set a thruster force of 275 N trying to slowdown our descent
    thrust = 275.0  # [N]
    thrustMsg = messaging.SingleActuatorMsg()
    thrustMsg.write(messaging.SingleActuatorMsgPayload(input=thrust))

    scene.getSingleActuator(THRUSTER_NAME).actuatorInMsg.subscribeTo(thrustMsg)

    # Record the state of the 'hub' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    bodyStateRecorder = scene.getBody(
        SPACECRAFT_BODY_NAME
    ).getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    if vizSupport.vizFound:
        asteroidGeom = _get_body_geom_info(scene, ASTEROID_BODY_NAME)
        thrusterVizWriter = ThrusterVizMessageWriter(
            THRUSTER_NAME,
            thrustMsg,
            thrust,
            THRUSTER_LOCATION,
            THRUSTER_DIRECTION,
            THRUSTER_VIZ_SCALE,
        )
        scSim.AddModelToTask("test", thrusterVizWriter)

        viz = vizSupport.enableUnityVisualization(
            scSim,
            "test",
            scene,
            # saveFile=__file__,
        )
        _attach_thruster_visualization(
            viz,
            SPACECRAFT_BODY_NAME,
            thrusterVizWriter,
        )
        viz.settings.showSpacecraftAsSprites = -1
        viz.settings.ambient = 0.1
        viz.settings.spacecraftShadowBrightness = 0.07
        vizSupport.createCustomModel(
            viz,
            modelPath=AST_OBJ_PATH,
            simBodiesToModify=[ASTEROID_BODY_NAME],
            scale=[
                ASTEROID_VIZ_SCALE,
                ASTEROID_VIZ_SCALE,
                ASTEROID_VIZ_SCALE,
            ],
            offset=list(asteroidGeom.pos),
            rotation=list(rbk.EP2Euler321(list(asteroidGeom.quat))),
            customTexturePath=AST_TEXTURE_PATH,
            shader=1,
        )

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Initial velocity of 1 m/s towards asteroid
    scene.getBody(SPACECRAFT_BODY_NAME).setVelocity([0.0, 0.0, -1.0])  # [m/s]

    # Run the simulation for some time with the thruster on
    scSim.ConfigureStopTime(macros.sec2nano(timeThrustTurnOff))
    scSim.ExecuteSimulation()

    # Near surface, turn off thrusters and let gravity land us
    thrustMsg.write(messaging.SingleActuatorMsgPayload(input=0.0))  # [N]

    # Run until simulation completion
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    if showPlots:
        # Plot the velocity of the hub
        vHub = np.squeeze(bodyStateRecorder.v_BN_N)
        for i in range(3):
            plt.plot(
                bodyStateRecorder.times() * macros.NANO2SEC,
                vHub[:, i],
                "-",
                label=f"Vel [{i}]",
            )

        plt.xlabel("Time [s]")
        plt.ylabel("Hub Velocity (Inertial) [m/s]")
        plt.legend()
        plt.show()


class ThrusterVizMessageWriter(sysModel.SysModel):
    """Publish a ``THROutputMsg`` from a MuJoCo scalar thrust command."""

    def __init__(
        self,
        thrusterName: str,
        thrustInMsg: messaging.SingleActuatorMsg,
        maxThrust: float,
        thrusterLocation: Sequence[float],
        thrusterDirection: Sequence[float],
        visualizationScale: float,
        *args: Any,
    ):
        """Create a Vizard thruster message writer.

        :param thrusterName: Name of the MuJoCo actuator represented in Vizard.
        :param thrustInMsg: Scalar thrust command message used by MuJoCo.
        :param maxThrust: Nominal maximum thrust for Vizard scaling.
        :param thrusterLocation: Thruster location in the attached body frame.
        :param thrusterDirection: Unit thrust direction in the attached body frame.
        :param visualizationScale: Scale factor applied only to the Vizard thrust.
        """
        super().__init__(*args)
        self.ModelTag = thrusterName
        self.maxThrust = abs(maxThrust)  # [N]
        self.thrusterLocation = list(thrusterLocation)
        self.thrusterDirection = list(thrusterDirection)
        self.visualizationScale = visualizationScale
        self.thrustInMsg = messaging.SingleActuatorMsgReader()
        self.thrustInMsg.subscribeTo(thrustInMsg)
        self.thrOutMsg = messaging.THROutputMsg()

    def Reset(self, CurrentSimNanos: int):
        """Write the initial thruster visualization payload."""
        self._write_thruster_payload(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int):
        """Write the current thruster visualization payload."""
        self._write_thruster_payload(CurrentSimNanos)

    def _write_thruster_payload(self, CurrentSimNanos: int):
        """Write the current scalar thrust command for Vizard."""
        thrustForce = self.thrustInMsg().input  # [N]
        vizThrustForce = self.visualizationScale * thrustForce  # [N]
        payload = messaging.THROutputMsgPayload()
        payload.maxThrust = self.maxThrust
        payload.thrustForce = vizThrustForce
        if self.maxThrust > 0.0:
            payload.thrustFactor = vizThrustForce / self.maxThrust
        payload.thrustBlowDownFactor = 1.0
        payload.ispBlowDownFactor = 1.0
        payload.thrusterLocation = self.thrusterLocation
        payload.thrusterDirection = self.thrusterDirection
        payload.thrustForce_B = [
            vizThrustForce * directionComponent
            for directionComponent in self.thrusterDirection
        ]
        self.thrOutMsg.write(payload, CurrentSimNanos, self.moduleID)


class ConstantGravity(sysModel.SysModel):
    """
    A class to model constant gravity force in a simulation.

    Attributes:
        force_N (Sequence[float]):
            The constant gravitational force vector in the inertial frame (N frame).
        frameInMsg (messaging.SCStatesMsgReader):
            Reader for spacecraft state messages.
        forceOutMsg (messaging.ForceAtSiteMsg):
            Message to output the computed force in the body-fixed frame (B frame).
    """

    def __init__(self, force_N: Sequence[float], *args: Any):
        """
        Args:
            force_N (Sequence[float]): The gravity force vector in the
            inertial reference frame.
        """
        super().__init__(*args)

        self.force_N = force_N

        self.frameInMsg = messaging.SCStatesMsgReader()

        self.forceOutMsg = messaging.ForceAtSiteMsg()

    def UpdateState(self, CurrentSimNanos: int):
        """Called at every integrator step to compute the force
        in the spacecraft-fixed reference frame."""
        # N frame: inertial frame
        # B frame: body-fixed frame
        frame: messaging.SCStatesMsgPayload = self.frameInMsg()
        dcm_BN = rbk.MRP2C(frame.sigma_BN)
        force_B = np.dot(dcm_BN, self.force_N)

        payload = messaging.ForceAtSiteMsgPayload(force_S=force_B)
        self.forceOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(True)
