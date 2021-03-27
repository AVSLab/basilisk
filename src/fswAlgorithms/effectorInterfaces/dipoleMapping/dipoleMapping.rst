Executive Summary
-----------------

This module computes individual torque rod dipole commands taking into account saturation limits.

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
    * - mtbParamsInMsg
      - :ref:`MTBArrayConfigMsgPayload`
      - input message for MTB layout
    * - dipoleRequestBodyInMsg
      - :ref:`DipoleRequestBodyMsgPayload`
      - input message containing the requested body frame dipole
    * - dipoleRequestMtbOutMsg
      - :ref:`MTBCmdMsgPayload`
      - input message containing the individual dipole requests for each torque bar on the vehicle

User Guide
----------
See the example script :ref:`scenarioMtbMomentumManagementSimple` for an illustration on how to use this module.

The user must set the ``steeringMatrix``, which is the psuedoinverse of ``GtMatrix_B``. Note that the MTB input configuration message variable ``GtMatrix_B`` must be provided in a row major format.
