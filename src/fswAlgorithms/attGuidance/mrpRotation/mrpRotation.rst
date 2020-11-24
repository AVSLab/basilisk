Executive Summary
-----------------

This module creates a dynamic reference frame attitude state message where the initial orientation relative to the input reference frame is specified through an MRP set, and the angular velocity vector is held fixed as seen by the resulting reference frame. More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/attGuidance/mrpRotation/_Documentation/Basilisk-MRPROTATION-20180522.pdf>`.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - name of the output message containing the Reference
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - name of the guidance reference input message
    * - desiredAttInMsg
      - :ref:`AttStateMsgPayload`
      - (optional) name of the incoming message containing the desired Euler angle set

