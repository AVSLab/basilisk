Executive Summary
-----------------
This C++ module determines the needed joint torques to prevent the arms from accelerating when the thrusters fire.

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
    * - massMatrixInMsg
      - :ref:`MJSysMassMatrixMsgPayload`
      - system mass matrix input msg
    * - nonActForceInMsg
      - :ref:`MJNonActuatorForcesMsgPayload`
      - non-actuator forces input msg
    * - bodyForceInMsg
      - :ref:`CmdForceBodyMsgPayload`
      - body force input msg
    * - bodyTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - body torque input msg
    * - jointTorqueInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - joint torque input msg
    * - jointTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - joint motor torque C++ output msg
