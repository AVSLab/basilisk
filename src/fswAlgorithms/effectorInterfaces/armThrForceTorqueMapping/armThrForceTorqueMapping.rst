Executive Summary
-----------------
This C++ module maps the thruster forces to body force and torques and joint torquesfor a spacecraft with arm mounted thrusters.

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
    * - configInMsg
      - :ref:`MJSCConfigMsgPayload`
      - spacecraft configuration input msg
    * - jointStatesInMsg
      - :ref:`JointArrayStateMsgPayload`
      - joint states input msg
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - thruster forces input msg
    * - bodyForceOutMsg
      - :ref:`CmdForceBodyMsgPayload`
      - body force output msg
    * - bodyTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - body torque output msg
    * - jointTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - joint torque output msg
