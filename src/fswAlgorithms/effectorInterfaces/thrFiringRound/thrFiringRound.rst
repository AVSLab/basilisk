Executive Summary
-----------------
This module takes in thruster on times and uses them to determine the thrust from the thruster at the current time step.

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
    * - onTimeInMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - total thruster on time input message
    * - thrForceOutMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - thruster force output msg
