Executive Summary
-----------------

This module computes a Body frame reequested dipole given a requested body torque and magnetic field vector.

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
    * - tamSensorBodyInMsg
      - :ref:`TAMSensorBodyMsgPayload`
      - input message for magnetic field sensor data
    * - tauRequestInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - input message containing control torque in the Body frame
    * - dipoleRequestOutMsg
      - :ref:`DipoleRequestBodyMsgPayload`
      - output message containing dipole request in the Body frame

User Guide
----------
See the example script :ref:`scenarioMtbMomentumManagementSimple` for an illustration on how to use this module.
