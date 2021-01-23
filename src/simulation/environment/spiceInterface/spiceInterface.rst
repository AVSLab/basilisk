Executive Summary
-----------------

The SPICE interface class gets time and planetary body information from the JPL ephemeris library


The module
:download:`PDF Description </../../src/simulation/environment/spice/_Documentation/Basilisk-SPICE_INTERFACE20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.



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
    * - spiceTimeOutMsg
      - :ref:`SpiceTimeMsgPayload`
      - spice time sampling output message
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) input epoch message
    * - planetStateOutMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet state output messages
