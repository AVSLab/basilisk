Executive Summary
-----------------
The purpose of this module is to implement a tabular atmospheric model that returns density and
temperature values at a specified altitude and planet. The module linearly interpolates tables
and is a child of the base class Atmosphere.

The module is a sub-class of the :ref:`atmosphereBase` base class.  See that class for the
nominal messages used and general instructions.


Detailed Module Description
---------------------------
Within the configured altitude reach, the module returns the tabulated density and temperature
at either endpoint and linearly interpolates between adjacent rows.
An altitude that falls just outside the table by roundoff in the spacecraft radius uses the nearest endpoint.

Module Assumptions and Limitations
----------------------------------
Returns density = 0 kg/m^3 and temperature = 0 K when altitude is outside range of provided data
OR if outside range set by (envMinReach, envMaxReach) if those module parameters are defined.
This module uses a python helper function as access to different atmospheric tables for various planets.
See ``Basilisk\supportData\AtmosphereData\support\README.txt`` for more detail on python
helper function and currently provided data tables.

User Guide
----------
Required variables are ``altList``, ``rhoList``, and ``tempList``, each a standard vector of doubles.
The lists must be sorted corresponding to ascending altitude, and be of the same nonzero length.
Altitude must be provided in meters, density in kg/m^3, and temperature in Kelvin.
