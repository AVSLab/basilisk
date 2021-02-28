Executive Summary
-----------------

General atmosphere base class used to calculate neutral density/temperature using arbitrary models
The Atmosphere class is used to calculate the neutral density and temperature above a body using arbitrary models.
Each atmosphere is attached to a specific planet, but provides support for
multiple spacecraft through ``addSpacecraftToModel()``.

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
    * - scStateInMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of spacecraft state input messages
    * - envOutMsgs
      - :ref:`AtmoPropsMsgPayload`
      - vector of atmospheric density output messages
    * - planetPosInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) planet state input message.  If not provided the planet state is zero information.
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) epoch date/time input message


