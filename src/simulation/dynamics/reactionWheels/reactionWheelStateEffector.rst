Executive Summary
-----------------

Class that is used to implement an effector impacting a dynamic body
that does not itself maintain a state or represent a changing component of
the body (for example: gravity, thrusters, solar radiation pressure, etc.)

The module
:download:`PDF Description </../../src/simulation/dynamics/reactionWheels/_Documentation/Basilisk-REACTIONWHEELSTATEEFFECTOR-20170816.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwMotorCmdInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - (optional) RW motor torque array cmd input message.  If not connected the motor torques are set to zero.
    * - rwSpeedOutMsg
      - :ref:`RWSpeedMsgPayload`
      - RW speed array output message
    * - rwOutMsgs
      - :ref:`RWConfigLogMsgPayload`
      - vector of RW log output messages

User Guide
-----------

The reaction wheel state effector module provides functionality for simulating reaction wheels in a spacecraft.
It includes safety mechanisms to prevent numerical instability that can occur with excessive wheel acceleration
or when using unlimited torque with small spacecraft inertia.

Threshold Parameters
~~~~~~~~~~~~~~~~~~~~

The module includes two configurable threshold parameters:

* ``maxWheelAcceleration``: Maximum allowed wheel acceleration to prevent numerical instability. Default value is 1.0e6 rad/s^2.
* ``largeTorqueThreshold``: Threshold for warning about large torque with unlimited torque setting. Default value is 10.0 Nm.

These parameters can be accessed and modified using the following getter and setter methods:

.. code-block:: python

    # Get the current maximum wheel acceleration threshold
    current_max_accel = reactionWheelStateEffector.getMaxWheelAcceleration()

    # Set a new maximum wheel acceleration threshold
    reactionWheelStateEffector.setMaxWheelAcceleration(2.0e6)  # rad/s^2

    # Get the current large torque threshold
    current_torque_threshold = reactionWheelStateEffector.getLargeTorqueThreshold()

    # Set a new large torque threshold
    reactionWheelStateEffector.setLargeTorqueThreshold(15.0)  # Nm
