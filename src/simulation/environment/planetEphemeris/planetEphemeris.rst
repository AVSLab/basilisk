
The planetEphemeris module uses classical heliocentric orbit elements to specify a planet's
position and velocity vectors.

Optionally, the planet's orientation can also be specified through a right ascension angle, a declination angle, and a location sidereal time at epoch.  The planet rotation about its third (polar) axis can be specified as well.  If the planet attitude information is not complete or missing then an inertially fixed zero planet orientation is modeled.  The module is able to receive a stack or vector of classical orbit elements and orientation information to output a series of planet ephemeris messages. The module
:download:`PDF Description </../../src/simulation/environment/planetEphemeris/_Documentation/Basilisk-planetEphemeris-20190422.pdf>`
contains further information on this module's function, how to run it, as well as testing.

