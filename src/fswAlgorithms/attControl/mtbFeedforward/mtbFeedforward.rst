Executive Summary
-----------------

This module computes the feedforward torque command from the expected torque to be produced by the torque rods given their current dipole commands.

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
    * - vehControlInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - input message containing the current control torque in the Body frame
    * - dipoleRequestMtbInMsg
      - :ref:`MTBCmdMsgPayload`
      - input message containing the individual dipole requests for each torque bar on the vehicle
    * - tamSensorBodyInMsg
      - :ref:`TAMSensorBodyMsgPayload`
      - input message for magnetic field sensor data
    * - mtbParamsInMsg
      - :ref:`MTBArrayConfigMsgPayload`
      - input message for MTB layout
    * - vehControlOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - output message containing the current control torque in the Body frame

User Guide
----------
See the example script :ref:`scenarioMtbMomentumManagementSimple` for an illustration on how to use this module.
