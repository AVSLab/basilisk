Executive Summary
-----------------
The purpose of this module is to implement a tabular atmospheric model that returns density and
temperature values at a specified altitude and planet. The module linearly interpolates tables
and is a child of the base class Atmosphere.

The module is a sub-class of the :ref:`atmosphereBase` base class.  See that class for the
nominal messages used and general instructions.


Detailed Module Description
---------------------------
The ``tabularAtmosphere`` module handles the following behavior:

#. Linear interpolation when requested altitude lies within the range of values on the atmosphere
   table but is not already included in the list.
#. Iterates through the list until the requested altitude is greater than the previous value in
   the list and less than the next value.
#. Will interpolate between the altitude and return the interpolated density and temperature.
      
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
    
