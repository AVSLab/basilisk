Executive Summary
-----------------

General magnetic field base class used to calculate the magnetic field above a planet using multiple models. The MagneticField class is used to calculate the magnetic field vector above a body using multiple models. This base class is used to hold relevant planetary magnetic field properties to compute answers for a given set of spacecraft locations relative to a specified planet.  Specific magnetic field models are implemented as classes that inherit from this base class. Planetary parameters, including position and input message, are settable by the user. In a given simulation, each planet of interest should have only one magnetic field  model associated with it linked to the spacecraft in orbit about that body.

Each magnetic field is attached to a specific planet, but provides support for
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
      - :ref:`MagneticFieldMsgPayload`
      - vector of magnetic density output messages
    * - planetPosInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) planet state input message.  If not provided the planet state is zero information.
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) epoch date/time input message
