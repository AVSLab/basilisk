Executive Summary
-----------------

Atmosphere class used to calculate temperature / density above a body using multiple models.
This class is used to hold relevant atmospheric properties and to compute the density for a given set of spacecraft
relative to a specified planet. Planetary parameters, including position and input message, are settable by the user.
Internal support is provided for Venus, Earth, and Mars. In a given simulation, each planet of interest should have only
one MsisAtmosphere model associated with it linked to the spacecraft in orbit about that body.  For more information see the
:download:`PDF Description </../../src/simulation/environment/MsisAtmosphere/_Documentation/Basilisk-msisAtmosphere-20190221.pdf>`.



Message Connection Descriptions
-------------------------------
The module is a sub-class of the :ref:`atmosphereBase` base class.  See that class for the nominal messages
used and general instructions.

The following table lists all the module specific input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - swDataInMsgs
      - :ref:`SwDataMsgPayload`
      - vector of 23 space weather data input messages

Regarding the vector ``swDataInMsgs``, the order of these 23 messages must follow this message order::

         0 - ap_24_0
         1 - ap_3_0
         2 - ap_3_-3
         3 - ap_3_-6
         4 - ap_3_-9
         5 - ap_3_-12
         6 - ap_3_-15
         7 - ap_3_-18
         8 - ap_3_-21
         9 - ap_3_-24
         10 - ap_3_-27
         11 - ap_3_-30
         12 - ap_3_-33
         13 - ap_3_-36
         14 - ap_3_-39
         15 - ap_3_-42
         16 - ap_3_-45
         17 - ap_3_-48
         18 - ap_3_-51
         19 - ap_3_-54
         20 - ap_3_-57
         21 - f107_1944_0
         22 - f107_24_-24


