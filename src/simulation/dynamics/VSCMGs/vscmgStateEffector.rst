
Executive Summary
-----------------

This dynamic effector class implements a variable speed control moment gyroscope or VSCMG device.

The module
:download:`PDF Description </../../src/simulation/dynamics/VSCMGs/_Documentation/Basilisk-VSCMGSTATEEFFECTOR-20180718.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdsInMsg
      - :ref:`VSCMGArrayTorqueMsgPayload`
      - motor torque command input message
    * - speedOutMsg
      - :ref:`VSCMGSpeedMsgPayload`
      - VSCMG speed output message
    * - vscmgOutMsgs
      - :ref:`VSCMGConfigMsgPayload`
      - vector of VSCMG output messages

