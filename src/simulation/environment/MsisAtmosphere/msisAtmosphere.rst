
Atmosphere class used to calculate temperature / density above a body using multiple models.
This class is used to hold relevant atmospheric properties and to compute the density for a given set of spacecraft
relative to a specified planet. Planetary parameters, including position and input message, are settable by the user.
Internal support is provided for Venus, Earth, and Mars. In a given simulation, each planet of interest should have only
one MsisAtmosphere model associated with it linked to the spacecraft in orbit about that body.  For more information see the
:download:`PDF Description </../../src/simulation/environment/MsisAtmosphere/_Documentation/Basilisk-msisAtmosphere-20190221.pdf>`.



