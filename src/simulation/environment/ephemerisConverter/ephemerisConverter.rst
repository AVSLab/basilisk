Executive Summary
-----------------

This class is used to take ephemeris data from the environmental models
and convert it over to a FSW representation so that the ephemeris from
SPICE can be patched into the FSW directly instead of generating data
from an ephemeris model.

The module
:download:`PDF Description </../../src/simulation/environment/ephemerisConverter/_Documentation/Basilisk-EPHEMERIS_CONVERTER20170712.pdf>`
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
    * - ephemOutMsgs
      - :ref:`EphemerisMsgPayload`
      - vector of planet ephemeris output messages
    * - spiceInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet spice state input messages
