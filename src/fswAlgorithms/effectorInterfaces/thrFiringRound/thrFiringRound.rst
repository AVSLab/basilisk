Executive Summary
-----------------

This module converts thruster force commands into thruster on-time commands.
Each positive force command is normalized by the configured maximum thrust,
scaled by the control period, clipped to the control period, and rounded to the
nearest configured on-time resolution.


Message Connection Descriptions
-------------------------------
The following table lists the module input and output messages.  The message
type contains a link to the message structure definition, while the description
provides information on what the message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - Input thruster force command message.
    * - thrOnTimeOutMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - Output thruster on-time command message.


Module Assumptions and Limitations
----------------------------------
Negative force commands are treated as zero-force requests because the on-time
logic is one-sided.  The number of thrusters can be set directly with
``setNumThrusters``.  If it is not set, the module uses the length of the
configured maximum-thrust vector, or the length of the input force command when
the maximum thrust is scalar.


User Guide
----------
The module is imported through the standard flight-software package:

.. code-block:: python

    from Basilisk.fswAlgorithms import thrFiringRound

    rounder = thrFiringRound.ThrFiringRound()
    rounder.ModelTag = "thrFiringRound"

The control period, on-time resolution, number of thrusters, and maximum
thruster force are configured with setter methods:

.. code-block:: python

    rounder.setControlPeriodSec(10.0)
    rounder.setOnTimeResolutionSec(0.1)
    rounder.setNumThrusters(2)
    rounder.setThrForceMax([2.5, 2.5])
