Executive Summary
-----------------
This module computes the required Delta-V to change the spacecraft velocity to the desired velocity
:math:`{}^N\mathbf{v}_{desired}` at maneuver time :math:`t_{maneuver}`. The desired velocity and maneuver time are
obtained by the :ref:`DesiredVelocityMsgPayload` input message. The expected velocity :math:`{}^N\mathbf{v}_{expected}`
at maneuver time is obtained from the second velocity in :ref:`LambertSolutionMsgPayload`. That is, this module computes
the Delta-V that is required at the end of a Lambert problem transfer arc (at the second position vector of Lambert
problem) to obtain some desired velocity.
The :ref:`DvBurnCmdMsgPayload` output message is zeroed if the :ref:`LambertSolutionMsgPayload` message indicates that
the Lambert solution is not valid.


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
    * - lambertSolutionInMsg
      - :ref:`LambertSolutionMsgPayload`
      - lambert problem solution input message
    * - desiredVelocityInMsg
      - :ref:`DesiredVelocityMsgPayload`
      - desired inertial velocity input message
    * - dvBurnCmdOutMsg
      - :ref:`DvBurnCmdMsgPayload`
      - Delta-V command output message


User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = lambertSecondDV.LambertSecondDV()
    module.ModelTag = "lambertSecondDV"
    unitTestSim.AddModelToTask(unitTaskName, module)

The input messages are then connected:

.. code-block:: python

    module.lambertSolutionInMsg.subscribeTo(lambertSolutionInMsg)
    module.desiredVelocityInMsg.subscribeTo(desiredVelocityInMsg)
