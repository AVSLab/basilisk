Executive Summary
-----------------
This is a module that computes the translational tracking error and uses a PD controller to output the required forces.

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
    * - transRefInMsg
      - :ref:`TransRefMsgPayload`
      - input msg reference translation
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - scStateInMsg
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - vehicle configuration input message
    * - cmdForceOutMsg
      - :ref:`CmdForceInertialMsgPayload`
      - commanded spacecraft external control force output message
