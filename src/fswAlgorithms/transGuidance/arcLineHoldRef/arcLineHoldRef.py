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

"""Fixed-family translation reference with an arc, line, and final hold.

The reference consists of a circular arc with equal inertial ``x`` and ``z``
components, producing a 45-degree out-of-plane trajectory.  A quintic segment
connects that arc to a line, followed by a deceleration constrained to the
inertial ``y`` axis and a final stationary hold.  Radius, timing, and endpoint
values are configurable; the trajectory topology, circle orientation, and
deceleration axis are intentionally fixed.
The blend endpoint and final hold must both lie on the inertial ``y`` axis.
"""

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import macros as mc


class ArcLineHoldRef(sysModel.SysModel):
    """Generate the fixed circular-to-line tracking maneuver.

    The circular path uses ``r_x = r_z`` and is therefore fixed at 45 degrees
    out of the inertial ``x-y`` plane.  The final segment only changes the
    inertial ``y`` position.  This module is not a general-purpose trajectory
    generator: it does not support arbitrary circle planes, path topologies,
    or deceleration directions.
    """

    def __init__(self):
        """Initialize output messaging and supported maneuver parameters.

        The supported parameters tune the fixed maneuver family only: circular
        radius, segment durations, transition endpoint, and final endpoint.
        """
        super().__init__()
        self.transRefOutMsg = messaging.TransRefMsg()
        self.payload = messaging.TransRefMsgPayload()

        self.radius = 20.0  # [m]
        self.finalPosition = np.array([0.0, -5.0, 0.0])  # [m]
        self.blendEndPosition = np.array([0.0, -16.0, 0.0])  # [m]
        self.tCircle = 5.0 * 3600.0  # [s]
        self.tTransition = 1.0 * 3600.0  # [s]
        self.tLinear = 110.0 * 60.0  # [s]

    def setRadius(self, radius):
        """Set the radius of the fixed 45-degree out-of-plane circular arc.

        :param radius: Circular-trajectory radius in meters.
        """
        self.radius = radius

    def setFinalPosition(self, finalPosition):
        """Set the final stationary reference position.

        The inertial ``x`` and ``z`` components must be zero.

        :param finalPosition: Final position in meters.
        """
        self.finalPosition = np.asarray(finalPosition, dtype=float)

    def setBlendEndPosition(self, blendEndPosition):
        """Set the position at the end of the circle-to-line transition.

        The following deceleration occurs only along the inertial ``y`` axis,
        so the inertial ``x`` and ``z`` components must be zero.

        :param blendEndPosition: Transition end position in meters.
        """
        self.blendEndPosition = np.asarray(blendEndPosition, dtype=float)

    def setTCircle(self, tCircle):
        """Set the circular-trajectory duration.

        :param tCircle: Circular-trajectory duration in seconds.
        """
        self.tCircle = tCircle

    def setTTransition(self, tTransition):
        """Set the circle-to-line transition duration.

        :param tTransition: Transition duration in seconds.
        """
        self.tTransition = tTransition

    def setTLinear(self, tLinear):
        """Set the deceleration-segment duration.

        :param tLinear: Deceleration duration in seconds.
        """
        self.tLinear = tLinear

    def circleStates(self, currentTime):
        """Evaluate the circular portion of the trajectory.

        :param currentTime: Elapsed trajectory time in seconds.
        :return: Position, velocity, and acceleration reference vectors.
        """
        theta = self.omega * currentTime  # [rad]
        position = np.array([
            self.radius / np.sqrt(2.0) * np.sin(theta),
            self.radius * np.cos(theta),
            self.radius / np.sqrt(2.0) * np.sin(theta),
        ])  # [m]
        velocity = self.omega * np.array([
            self.radius / np.sqrt(2.0) * np.cos(theta),
            -self.radius * np.sin(theta),
            self.radius / np.sqrt(2.0) * np.cos(theta),
        ])  # [m/s]
        acceleration = -self.omega ** 2 * position  # [m/s^2]
        return position, velocity, acceleration

    def blendStates(self, elapsedTime):
        """Evaluate the quintic circle-to-line transition.

        :param elapsedTime: Time since transition start in seconds.
        :return: Position, velocity, and acceleration reference vectors.
        """
        position = np.zeros(3)  # [m]
        velocity = np.zeros(3)  # [m/s]
        acceleration = np.zeros(3)  # [m/s^2]
        duration = self.tTransition  # [s]

        for index in range(3):
            positionInitial = self.rBlendStart[index]  # [m]
            positionFinal = self.rBlendEnd[index]  # [m]
            velocityInitial = self.vBlendStart[index]  # [m/s]
            velocityFinal = self.vBlendEnd[index]  # [m/s]
            accelerationInitial = self.aBlendStart[index]  # [m/s^2]
            accelerationFinal = self.aBlendEnd[index]  # [m/s^2]

            # Quintic coefficients use position in [m] and elapsedTime in [s].
            coefficient0 = positionInitial
            coefficient1 = velocityInitial
            coefficient2 = accelerationInitial / 2.0
            coefficient3 = (
                -3.0 * accelerationInitial * duration ** 2
                + accelerationFinal * duration ** 2
                - 12.0 * velocityInitial * duration
                - 8.0 * velocityFinal * duration
                - 20.0 * positionInitial + 20.0 * positionFinal
            ) / (2.0 * duration ** 3)
            coefficient4 = (
                1.5 * accelerationInitial * duration ** 2
                - accelerationFinal * duration ** 2
                + 8.0 * duration * velocityInitial
                + 7.0 * duration * velocityFinal
                + 15.0 * positionInitial - 15.0 * positionFinal
            ) / duration ** 4
            coefficient5 = (
                -accelerationInitial * duration ** 2
                + accelerationFinal * duration ** 2
                - 6.0 * duration * velocityInitial
                - 6.0 * duration * velocityFinal
                - 12.0 * positionInitial + 12.0 * positionFinal
            ) / (2.0 * duration ** 5)

            position[index] = (
                coefficient0 + coefficient1 * elapsedTime
                + coefficient2 * elapsedTime ** 2 + coefficient3 * elapsedTime ** 3
                + coefficient4 * elapsedTime ** 4 + coefficient5 * elapsedTime ** 5
            )
            velocity[index] = (
                coefficient1 + 2.0 * coefficient2 * elapsedTime
                + 3.0 * coefficient3 * elapsedTime ** 2
                + 4.0 * coefficient4 * elapsedTime ** 3
                + 5.0 * coefficient5 * elapsedTime ** 4
            )
            acceleration[index] = (
                2.0 * coefficient2 + 6.0 * coefficient3 * elapsedTime
                + 12.0 * coefficient4 * elapsedTime ** 2
                + 20.0 * coefficient5 * elapsedTime ** 3
            )

        return position, velocity, acceleration

    def decelStates(self, elapsedTime):
        """Evaluate the final inertial-``y``-axis deceleration segment.

        :param elapsedTime: Time since deceleration start in seconds.
        :return: Position, velocity, and acceleration reference vectors.
        """
        position = np.zeros(3)  # [m]
        velocity = np.zeros(3)  # [m/s]
        acceleration = np.zeros(3)  # [m/s^2]

        initialVelocity = self.vBlendEnd[1]  # [m/s]
        positionChange = self.finalPosition[1] - self.rBlendEnd[1]  # [m]
        coefficient3 = (
            10.0 * positionChange - 6.0 * initialVelocity * self.tLinear
        ) / self.tLinear ** 3
        coefficient4 = (
            -15.0 * positionChange + 8.0 * initialVelocity * self.tLinear
        ) / self.tLinear ** 4
        coefficient5 = (
            6.0 * positionChange - 3.0 * initialVelocity * self.tLinear
        ) / self.tLinear ** 5

        displacement = (
            initialVelocity * elapsedTime + coefficient3 * elapsedTime ** 3
            + coefficient4 * elapsedTime ** 4 + coefficient5 * elapsedTime ** 5
        )  # [m]
        displacementRate = (
            initialVelocity + 3.0 * coefficient3 * elapsedTime ** 2
            + 4.0 * coefficient4 * elapsedTime ** 3
            + 5.0 * coefficient5 * elapsedTime ** 4
        )  # [m/s]
        displacementAcceleration = (
            6.0 * coefficient3 * elapsedTime + 12.0 * coefficient4 * elapsedTime ** 2
            + 20.0 * coefficient5 * elapsedTime ** 3
        )  # [m/s^2]

        position[1] = self.rBlendEnd[1] + displacement
        velocity[1] = displacementRate
        acceleration[1] = displacementAcceleration
        return position, velocity, acceleration

    def Reset(self, CurrentSimNanos):
        """Initialize the fixed-maneuver boundary states before simulation."""
        for name, value in (
            ("radius", self.radius),
            ("tCircle", self.tCircle),
            ("tTransition", self.tTransition),
            ("tLinear", self.tLinear),
        ):
            if not np.isfinite(value):
                self.bskLogger.error(f"ArcLineHoldRef.{name} must be finite.")

        for name, endpoint in (
            ("blendEndPosition", self.blendEndPosition),
            ("finalPosition", self.finalPosition),
        ):
            if endpoint.shape != (3,) or not np.all(np.isfinite(endpoint)):
                self.bskLogger.error(
                    f"ArcLineHoldRef.{name} must be a finite three-component position."
                )
            if endpoint[0] != 0.0 or endpoint[2] != 0.0:
                self.bskLogger.error(
                    f"ArcLineHoldRef.{name} must have zero inertial x and z components "
                    "because the deceleration and final hold lie on the y axis."
                )

        if self.tTransition <= 0.0 or self.tLinear <= 0.0:
            self.bskLogger.error(
                "ArcLineHoldRef requires positive transition and deceleration durations."
            )
        if self.tCircle + self.tTransition <= 0.0:
            self.bskLogger.error(
                "ArcLineHoldRef requires a positive circular-plus-transition duration."
            )

        self.omega = np.pi / (self.tCircle + self.tTransition)  # [rad/s]
        self.rBlendStart, self.vBlendStart, self.aBlendStart = self.circleStates(
            self.tCircle
        )
        self.rBlendEnd = self.blendEndPosition.copy()  # [m]
        self.vBlendEnd = self.omega * self.radius * np.array([0.0, 1.0, 0.0])  # [m/s]
        self.aBlendEnd = np.array([0.0, 0.0, 0.0])  # [m/s^2]

    def UpdateState(self, CurrentSimNanos):
        """Publish the translation reference corresponding to the simulation time."""
        currentTime = mc.NANO2SEC * CurrentSimNanos  # [s]

        if currentTime < self.tCircle:
            position, velocity, acceleration = self.circleStates(currentTime)
        elif currentTime < self.tCircle + self.tTransition:
            position, velocity, acceleration = self.blendStates(currentTime - self.tCircle)
        elif currentTime < self.tCircle + self.tTransition + self.tLinear:
            position, velocity, acceleration = self.decelStates(
                currentTime - self.tCircle - self.tTransition
            )
        else:
            position = self.finalPosition
            velocity = np.zeros(3)  # [m/s]
            acceleration = np.zeros(3)  # [m/s^2]

        self.payload.r_RN_N = position.tolist()
        self.payload.v_RN_N = velocity.tolist()
        self.payload.a_RN_N = acceleration.tolist()
        self.transRefOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)
