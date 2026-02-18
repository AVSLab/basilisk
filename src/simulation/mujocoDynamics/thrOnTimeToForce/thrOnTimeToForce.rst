Executive Summary
-----------------
The ``thrOnTimeToForce`` module converts commanded thruster on times into thruster force outputs to be used in a MuJoCo simulation. If a new commanded on-time is received for a thruster that on-time replaces any remaining on-time for that thruster.

.. note::
    This module uses a zero-order-hold approach to convert on-time commands to forces and is designed to be used with fixed thruster force magnitudes.
    This module additionally assumes that the on-time commands are built using some external logic and this module is not responsible for enforcing any minimum off-time or maximum duty cycle limits on the thrusters.

    The module only latches a new on-time command when the input message has a new ``timeWritten`` value. If a Python user rewrites the command without advancing ``timeWritten`` (or only writes once and expects it to be interpreted as a new command later), the module treats it as stale and does not re-latch it.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from Python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - onTimeInMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - input on-time command array
    * - thrusterForceOutMsgs
      - :ref:`SingleActuatorMsgPayload`
      - vector of thruster force output messages


User Guide
----------
This section is to outline the steps needed to set up the ``thrOnTimeToForce`` module in Python using Basilisk.

#. Import the thrOnTimeToForce class::

    from Basilisk.simulation import thrOnTimeToForce

#. Create an instance of thrOnTimeToForce::

    module = thrOnTimeToForce.ThrOnTimeToForce()

#. set the fixed thruster force magnitudes for the module::

    thrMag = [2.0, 3.0]
    module.setThrMag(thrMag)

#. For each thruster in the system, add a thruster to the module::

    module.addThruster()

#. Add the module to the task list::

    sim.AddModelToTask(unitTaskName, module)
