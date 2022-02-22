Executive Summary
-----------------
The purpose of this module is to implement a tabular atmospheric model that returns density and temperature values at a specified altitude and planet. The module linearly interpolates tables and is a child of the base class Atmosphere.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - altList
      - Double
      - vector of various altitude values in meters and ascending order
    * - rhoList
      - Double
      - vector of densities in kilogram per cubic meter at corresponding altitude in altList
    * - tempList
      - Double
      - vector of temperatures in Kelvin at corresponding altitude in altList


Detailed Module Description
---------------------------
The ``tabularAtmosphere`` module handles the following behavior:


      
Module Assumptions and Limitations
----------------------------------
Returns a density of 0 kg/m^3 when the input altitude is beyond the maximum altitude in the table, indicating that at such an altitude falls to being in space and no longer in the atmosphere.
Returns a density of __ when the input altitude is below the minimum altitude in the table, meaning either below the surface of the planet in the ground, or for the US Standard Atmosphere 1976 table where the minimum altitude recording is -5000 meters. 
Returns a temperature of 2.7 K when the input altitude is beyond the maximum altitude in the table, which is the temperature of space. 
Returns a temperature of __ K when the input altitude is below the minimum value in the table. 
This module uses a python helper function as access to different atmospheric tables for various planets.

User Guide
----------
The tabular atmosphere module is created using:

.. code-block:: python
    :linenos:

    module = tabularAtmosphere.TabularAtmosphere()   
    module.ModelTag = "tabularAtmosphere"
    unitTestSim.AddModelToTask(unitTaskName, module)
    
To add a spacecraft:

    module.addSpacecraftToModel(scInMsg)
    
 To assign data to variables:
                            
    module.altList = tabularAtmosphere.DoubleVector(altList)    
    module.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    module.tempList = tabularAtmosphere.DoubleVector(tempList)
    
    