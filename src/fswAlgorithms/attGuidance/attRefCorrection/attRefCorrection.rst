Executive Summary
-----------------
This module reads in the attitude reference message and adjusts it by a fixed rotation.  This allows a general body-fixed frame
:math:`B` to align with this corrected reference frame :math:`R_c`.

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
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference input message
    * - attRef2OutMsg
      - :ref:`AttRefMsgPayload`
      - corrected attitude reference input message

