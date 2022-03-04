.. _tabularAtmosphere:

Executive Summary
-----------------
The purpose of this module is to implement a tabular atmospheric model that returns density and temperature values at a specified altitude and planet. The module linearly interpolates tables and is a child of the base class Atmosphere.

The module is a sub-class of the :ref:`atmosphereBase` base class.  See that class for the nominal messages
used and general instructions.


Detailed Module Description
---------------------------
The ``tabularAtmosphere`` module handles the following behavior:

#. Linear interpolation when requested altitude lies within the range of values on the atmosphere table but is not already included in the list. 
   Iterates through the list until the requested altitude is greater than the previous value in the list and less than the next value.  Will interpolates
   between the altitude and return the interpolated density and temperature. 
      
Module Assumptions and Limitations
----------------------------------
Returns a density of 0 kg/m^3 when the input altitude is above the maximum altitude or below the minimum altitude in the table.
Returns a temperature of 0 K when the input altitude is below the minimum altitude in the table.
Returns a temperature of 2.7 K when the input altitude is beyond the maximum altitude in the table, which is the temperature of space. 
This module uses a python helper function as access to different atmospheric tables for various planets.

User Guide
----------
    
Required variables are altList, rhoList, and tempList and 
Each are a list, altitude in ascending order, length of lists are same, 
Can be defined using helper function
    
    