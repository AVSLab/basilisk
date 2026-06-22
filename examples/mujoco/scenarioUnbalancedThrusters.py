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

This script shows how to simulate a CubeSat with 4 thrusters attached
each to a different fuel tank. All thrusters produce the same thrust,
but one of the consumes fuel at twice the rate of the others (which
simulates a defect). This is done using the MuJoCo-based :ref:`DynamicObject<dynamicObject>`
:ref:`MJScene<MJScene>`.

The multi-body system is defined in the XML file ``sat_w_thrusters.xml``.
This XML file defines a 'hub' body with 4 tank bodies as their
sub-bodies ('tank_1', 'tank_2', etc.). The 'hub' body is a cube, while the
'tank' bodies are 'pills' and are contained within the 'hub' body. Each
'tank' body has a site attached to it ('tank_X_thrust_point'), which defines
the point at which the thrust vector is applied. The thrust of each thruster
is modeled as a single actuator ('tank_X_thrust') that applies a force at
the corresponding site. The attribute ``gear`` of the actuators is set to
``gear="0 0 -1 0 0 0"``, which means 'apply a force along the negative z-axis'.

For more information on how to define a multi-body system in MuJoCo, see
https://mujoco.readthedocs.io/en/stable/XMLreference.html

To model a thruster, two things must be achieved:

#. The thrust force must be applied.
#. The fuel consumption must be modeled.

To accomplish the first, we tie a standalone ``SingleActuatorMsg`` to
the control input of each of the thrusters. The payload of this message
is kept constant at 5 N.

To model the fuel consumption, we need to tell the system that the mass
of the tank bodies is decreasing. The mass of each of the bodies in the
system is a state, and thus should be updated by defining its time
derivative. This is done by tying a ``SCMassPropsMsg`` to the
``derivativeMassPropertiesInMsg`` of the tank bodies. In this script,
the payload of this message is kept constant at -1 kg/s for the first 3
tanks, and -2 kg/s for the last tank.

The simulation is run for 2.5 minutes, and the mass of each of the tanks
and the state of the 'hub' body are recorded. The mass of the tanks should
decrease linearly with time, with the last tank decreasing at twice the rate
of the others. Because of the initial symmetry of the configuration, the
'hub' body should appear to accelerate in a straight line. However, because
the last tank is consuming fuel at twice the rate of the others, the inertial
properties of the system become unbalanced, and the 'hub' body should start
to rotate. The system can also be visualized in :ref:`Vizard <vizard>` through
the ``enableUnityVisualization`` call.
"""

import os
from typing import Any, Sequence

from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import vizSupport
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.simulation import svIntegrators

import numpy as np
import matplotlib.pyplot as plt

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{CURRENT_FOLDER}/sat_w_thrusters.xml"
HUB_BODY_NAME = "hub"
THRUSTER_VIZ_INFO = {
    "tank_1_thrust": {
        "location": [0.6, 0.6, -1.0],  # [m]
        "direction": [0.0, 0.0, 1.0],  # [-]
    },
    "tank_2_thrust": {
        "location": [-0.6, 0.6, -1.0],  # [m]
        "direction": [0.0, 0.0, 1.0],  # [-]
    },
    "tank_3_thrust": {
        "location": [0.6, -0.6, -1.0],  # [m]
        "direction": [0.0, 0.0, 1.0],  # [-]
    },
    "tank_4_thrust": {
        "location": [-0.6, -0.6, -1.0],  # [m]
        "direction": [0.0, 0.0, 1.0],  # [-]
    },
}
THRUSTER_COLOR = "yellow"


def _attach_thruster_visualization(viz, spacecraftName, thrusterVizWriters):
    """Attach MuJoCo thruster visualization messages to one Vizard body."""
    from Basilisk.simulation import vizInterface

    for scDataIndex in range(len(viz.scData)):
        scData = viz.scData[scDataIndex]
        if scData.spacecraftName == spacecraftName:
            thrList = []
            thrInfo = []
            for writer in thrusterVizWriters:
                thrList.append(writer.thrOutMsg.addSubscriber())
                thrInfo.append(vizInterface.ThrClusterMap())
                thrInfo[-1].thrTag = writer.ModelTag
                thrInfo[-1].color = vizSupport.toRGBA255(THRUSTER_COLOR)

            scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(thrList)
            scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)
            vizSupport.setActuatorGuiSetting(
                viz,
                spacecraftName=spacecraftName,
                viewThrusterHUD=True,
            )
            return

    raise ValueError(
        f"Could not find spacecraft '{spacecraftName}' in Vizard spacecraft data."
    )


def run(showPlots: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
    """
    dt = 1.0  # [s]
    tf = 2.5 * 60.0  # [s]

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

    thrust = 5.0  # [N]
    mDot = -1.0  # [kg/s]

    messages = []
    massPropRecorders = []
    thrusterVizInputs = []

    for i in range(1, 5):
        actuatorName = f"tank_{i}_thrust"
        bodyName = f"tank_{i}"

        # Define a standalone ``SingleActuatorMsg`` used to
        # define the force that each thruster applies.
        # The thrusters defined in the XML ('tank_1_thrust', etc.)
        # are controlled through a scalar value. This scalar value
        # is multiplied by the ``gear`` attribute of the actuator
        # to define the force and torque vectors.
        # In this case, the gear is set to "0 0 -1 0 0 0", which
        # means that the force is applied along the negative z-axis.
        thrustMsg = messaging.SingleActuatorMsg()
        thrustMsg.write(messaging.SingleActuatorMsgPayload(input=thrust))

        scene.getSingleActuator(actuatorName).actuatorInMsg.subscribeTo(thrustMsg)

        messages.append(thrustMsg)
        thrusterVizInputs.append((actuatorName, thrustMsg))

        # Define a ``SCMassPropsMsg`` used to define the time derivative
        # of the mass of the tank bodies. The mass of the tank
        # bodies is a state, and thus should be updated by defining
        # its time derivative. In this case, the mass of the tanks
        # is decreasing linearly with time, with the last tank
        # decreasing at twice the rate of the others.
        mDotMsg = messaging.SCMassPropsMsg()
        mDotMsg.write(messaging.SCMassPropsMsgPayload(massSC=mDot if i < 4 else mDot * 2))

        body = scene.getBody(bodyName)

        body.derivativeMassPropertiesInMsg.subscribeTo(mDotMsg)

        messages.append(mDotMsg)

        # Record the mass of the tanks through the
        # ``massPropertiesOutMsg`` of the tank bodies.
        massPropRecorder = body.massPropertiesOutMsg.recorder()
        scSim.AddModelToTask("test", massPropRecorder)

        massPropRecorders.append(massPropRecorder)

    # Record the state of the 'hub' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    if vizSupport.vizFound:
        thrusterVizWriters = []
        for actuatorName, thrustMsg in thrusterVizInputs:
            vizInfo = THRUSTER_VIZ_INFO[actuatorName]
            writer = ThrusterVizMessageWriter(
                actuatorName,
                thrustMsg,
                thrust,
                vizInfo["location"],
                vizInfo["direction"],
            )
            scSim.AddModelToTask("test", writer)
            thrusterVizWriters.append(writer)

        viz = vizSupport.enableUnityVisualization(
            scSim,
            "test",
            scene,
            # saveFile=__file__,
        )
        _attach_thruster_visualization(viz, HUB_BODY_NAME, thrusterVizWriters)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Run the simulation for 2.5 minutes
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    if showPlots:

        # Plot the mass of the tanks
        for i, (style, rec) in enumerate(zip(("-", "--", ":", "-"), massPropRecorders)):
            plt.plot(
                rec.times(), np.squeeze(rec.massSC), style, label=f"Thruster {i+1}"
            )
        plt.xlabel("Time [s]")
        plt.ylabel("Mass [kg]")
        plt.legend()

        # Plot the attitude of the 'hub' body
        plt.figure()
        att = np.squeeze(bodyStateRecorder.sigma_BN)
        for i in range(3):
            plt.plot(
                bodyStateRecorder.times() * macros.NANO2SEC,
                att[:, i],
                label=f"Hub sigma [{i}]",
            )
        plt.xlabel("Time [s]")
        plt.ylabel(r"$\sigma_{BN}$ [rad]")
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
        *args: Any,
    ):
        """Create a Vizard thruster message writer.

        :param thrusterName: Name of the MuJoCo actuator represented in Vizard.
        :param thrustInMsg: Scalar thrust command message used by MuJoCo.
        :param maxThrust: Nominal maximum thrust for Vizard scaling.
        :param thrusterLocation: Thruster location in the attached body frame.
        :param thrusterDirection: Unit thrust direction in the attached body frame.
        """
        super().__init__(*args)
        self.ModelTag = thrusterName
        self.maxThrust = abs(maxThrust)  # [N]
        self.thrusterLocation = list(thrusterLocation)
        self.thrusterDirection = list(thrusterDirection)
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
        payload = messaging.THROutputMsgPayload()
        payload.maxThrust = self.maxThrust
        payload.thrustForce = thrustForce
        if self.maxThrust > 0.0:
            payload.thrustFactor = thrustForce / self.maxThrust
        payload.thrustBlowDownFactor = 1.0
        payload.ispBlowDownFactor = 1.0
        payload.thrusterLocation = self.thrusterLocation
        payload.thrusterDirection = self.thrusterDirection
        payload.thrustForce_B = [
            thrustForce * directionComponent
            for directionComponent in self.thrusterDirection
        ]
        self.thrOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(True)
