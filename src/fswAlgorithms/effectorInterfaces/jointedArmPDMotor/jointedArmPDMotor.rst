Executive Summary
-----------------
This module determines the motor torques for a jointed arm using a PD control law.

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
    * - jointStateInMsg
      - :ref:`JointArrayStateMsgPayload`
      - current joint states input msg
    * - desJointStateInMsg
      - :ref:`JointArrayStateMsgPayload`
      - desired joint states input msg
    * - nonActForceInMsg
      - :ref:`MJNonActuatorForcesMsgPayload`
      - non-actuator forces input msg
    * - jointTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - joint motor torque output msg
