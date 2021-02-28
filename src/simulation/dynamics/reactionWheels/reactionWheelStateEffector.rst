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














