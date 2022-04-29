Executive Summary
-----------------
This module checks that a vector of mapping points are visible to a spacecraft's imager, outputting a vector of accessMessages for each mapping point

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
    * - planetInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Planet state input message
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Spacecraft state input message

