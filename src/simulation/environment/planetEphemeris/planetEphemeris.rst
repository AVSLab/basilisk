Executive Summary
-----------------

The planetEphemeris module uses classical heliocentric orbit elements to specify a planet's
position and velocity vectors.

Optionally, the planet's orientation can also be specified through a right ascension angle, a declination angle, and a location sidereal time at epoch.  The planet rotation about its third (polar) axis can be specified as well.  If the planet attitude information is not complete or missing then an inertially fixed zero planet orientation is modeled.  The module is able to receive a stack or vector of classical orbit elements and orientation information to output a series of planet ephemeris messages. The module
:download:`PDF Description </../../src/simulation/environment/planetEphemeris/_Documentation/Basilisk-planetEphemeris-20190422.pdf>`
contains further information on this module's function, how to run it, as well as testing.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_Planet_Ephemeris:
.. figure:: /../../src/simulation/environment/planetEphemeris/_Documentation/Images/planetEphemerisIO.svg
    :align: center

    Figure 1: ``planetEphemeris()`` Module I/O Illustration


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetOutMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet spice state output messages

