Executive Summary
-----------------
This module determines the torque on the hub required to negate hub angular acceleration

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
    * - jointTorqueInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - joint motor torques input msg
    * - cmdTorqueOutMsg
      - :ref:`cmdTorqueBodyMsgPayload`
      - prescribed hub torque output msg in body frame components
