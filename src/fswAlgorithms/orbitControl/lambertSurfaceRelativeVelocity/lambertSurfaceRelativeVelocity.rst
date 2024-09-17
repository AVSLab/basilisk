Executive Summary
-----------------
This module computes the required inertial spacecraft velocity :math:`{}^N\mathbf{v}_{B/N}` to satisfy the desired
relative velocity to the surface :math:`{}^S\mathbf{v}_{rel,des}` at position :math:`{}^N\mathbf{r}_{B/N}` and
(maneuver) time :math:`t`. The module takes into account the rotation and orientation of the celestial body, provided
by the :ref:`EphemerisMsgPayload`. The spacecraft position :math:`{}^N\mathbf{r}_{B/N}` is provided by the second
position vector :math:`{}^N\mathbf{r}_{2}` in :ref:`LambertProblemMsgPayload`, as this module is designed to be used at
the end of a Lambert problem transfer arc. The surface frame S, in which the desired relative velocity vector is
expressed in, is an East-North-Up frame (third unit vector is in the radial direction, first unit vector is
perpendicular to the angular velocity vector of the celestial body and the radial direction, and the second unit vector
completes the right-handed coordinate frame). Note that the frame is not fully defined when the angular velocity vector
of the celestial body :math:`\mathbf{\omega}_{P/N}` is zero. In this case, the frame S is defined using the inertial z
vector :math:`[0, 0, 1]` instead of :math:`\mathbf{\omega}_{P/N}`. The computed required inertial spacecraft velocity
and maneuver time are written to the :ref:`DesiredVelocityMsgPayload` output message.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - lambertProblemInMsg
      - :ref:`lambertProblemMsgPayload`
      - lambert problem setup input message
    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - ephemeris input message
    * - desiredVelocityOutMsg
      - :ref:`DesiredVelocityMsgPayload`
      - Desired velocity output message


Algorithm
---------
The required inertial spacecraft velocity is computed by:

.. math::
    :label: eq:vInertial

    \mathbf{v}_{B/N} = \mathbf{\omega}_{P/N} \times \mathbf{r}_{B/N} + \mathbf{v}_{rel,des}

where :math:`\mathbf{\omega}_{P/N}` is the angular velocity of the planet fixed frame P w.r.t. the inertial frame N.


User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = lambertSurfaceRelativeVelocity.LambertSurfaceRelativeVelocity()
    module.ModelTag = "lambertSurfaceRelativeVelocity"
    module.setVRelativeDesired_S(np.array([0., 0., 10.]))  # in surface frame (East-North-Up)
    module.setTime(1000.)
    unitTestSim.AddModelToTask(unitTaskName, module)

The input messages are then connected:

.. code-block:: python

    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)
    module.ephemerisInMsg.subscribeTo(ephemerisInMsg)
