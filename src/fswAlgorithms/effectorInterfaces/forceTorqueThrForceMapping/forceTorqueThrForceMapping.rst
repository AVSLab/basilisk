Executive Summary
-----------------
This module maps thruster forces for arbitrary forces and torques

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
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - The name of the vehicle control (Lr) input message
    * - cmdForceInMsg
      - :ref:`CmdForceBodyMsgPayload`
      - The name of the vehicle control force input message
    * - thrConfigInMsg
      - :ref:`THRArrayConfigMsgPayload`
      - The name of the thruster cluster input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - The name of the vehicle config input message
    * - thrForceCmdOutMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - The name of the output thruster force message

