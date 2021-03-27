Executive Summary
-----------------

This module computes the desired Body frame torque to dump the momentum in the reaction wheels.

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
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - input message for RW parameters
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - input message for RW speeds
    * - tauMtbRequestOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - output message containing control torque in the Body frame

User Guide
----------
See the example script :ref:`scenarioMtbMomentumManagementSimple` for an illustration on how to use this module.

The user must set the momentum dumping gain value ``Kp`` to a postive value. 
