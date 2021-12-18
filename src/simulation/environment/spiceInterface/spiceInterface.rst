Executive Summary
-----------------

The SPICE interface class gets time and planetary or spacecraft body information from the JPL ephemeris library


The module
:download:`PDF Description </../../src/simulation/environment/spiceInterface/_Documentation/Basilisk-SPICE_INTERFACE20170712.pdf>`
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
    * - scStateOutMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of spacecraft state messages
    * - attRefStateOutMsgs
      - :ref:`AttRefMsgPayload`
      - vector of spacecraft attitude reference state output messages
    * - transRefStateOutMsgs
      - :ref:`TransRefMsgPayload`
      - vector of spacecraft translation reference state output messages


User Guide
----------
This module uses the JPL Spice software to determine the position and orientation of both a celestial body, or a spacecraft.
The appropriate Spice kernel must be loaded up to provide the state information for the selected body names.

To setup a celestial body, use the module method ``addPlanetNames(vector<string>)`` in python.  Multiple object names can be
provided by providing a list of names.  With each module update cycle, the corresponding celestial body
states are provided in the vector of output messages ``planetStateOutMsgs[]``.  Note that the vector elements are in
the order that the celestial body names were added.

To use this module to read in spacecraft states from a spice kernel, then the spacecraft Spice name is added using
the method ``addSpacecraftNames(vector<string>)``.  The module provides a vector out corresponding spacecraft state output
messages in three different formats.

- ``scStateOutMsgs[]``: these are the :ref:`SCStatesMsgPayload` state output messages that the :ref:`spacecraft` module provides
- ``attRefStateOutMsgs[]``: these are the attitude reference messages :ref:`AttRefMsgPayload`.  These are useful to
  only prescribe the spacecraft attitude motion.
- ``transRefStateOutMsgs[]``: these are the translational reference message :ref:`TransRefMsgPayload`.  These are useful to only
  prescribe the translational motion and leave the attitude motion free.
