Executive Summary
-----------------
This module provides the GEO electron and ion flux based on 82 satellite-years as discussed in `this paper <https://doi
.org/10.1002/2015SW001168>`__ by M. H. Denton and on `this website <https://gemelli.spacescience.org/mdenton/>`__.
A Fortran module can be downloaded from this website which computes the electron and ion flux for a given :math:`K_p` index,
local time and particle energy, and also includes the relevant flux data. The Basilisk module only uses the mean
flux data.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. _ModuleIO_Denton_Flux_Model:
.. figure:: /../../src/simulation/environment/dentonFluxModel/_Documentation/Images/moduleDentonFluxModel.svg
    :align: center

    Figure 1: ``dentonFluxModel()`` Module I/O Illustration


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``scStateInMsg``
      - :ref:`SCStatesMsgPayload`
      - spacecraft state input message
    * - ``earthStateInMsg``
      - :ref:`SpicePlanetStateMsgPayload`
      - Earth planet state input message
    * - ``sunStateInMsg``
      - :ref:`SpicePlanetStateMsgPayload`
      - Sun state input message
    * - ``fluxOutMsg``
      - :ref:`PlasmaFluxMsgPayload`
      - output ion and electron fluxes

Module Assumptions and Limitations
----------------------------------
This module assumes that the position vectors of the spacecraft, Sun and Earth are expressed in an Earth equatorial
frame with the z-component perpendicular to the equatorial plane. The local time of the spacecraft is determined by
projecting the spacecraft and Sun position vectors onto the equatorial plane and computing the angle between the
resulting projected vectors. When both projected vectors point in the same direction, then the local time (LT) is 12
hours.

The particle energies are limited between 100 eV and 40 keV. The lower limit of 100 eV is recommended by M. H.
Denton, because the fluxes of smaller energies are unreliable due to contamination with secondary electrons and
photoelectrons. The flux data provided by Denton does not provide information above 40 keV.

Since the flux data is only valid for the GEO regime, a BSK warning message is generated if the spacecraft location is
more than 4,000 km away from Geostationary Orbit (equatorial orbit with radius of 42,000 km).

User Guide
----------
The Denton model averaged GEO space plasma properties module is created using:

.. code-block:: python
    :linenos:

    fluxModule = dentonFluxModel.DentonFluxModel()
    fluxModule.ModelTag = "dentonFluxModule"
    fluxModule.dataPath = bskPath + '/supportData/DentonGEO/'
    fluxModule.kpIndex = "2+"
    fluxModule.numOutputEnergies = 30
    scSim.AddModelToTask(dynTaskName, fluxModule)

The :math:`K_p` index (``kpIndex``) and number of output energies (``numOutputEnergies``) must be
added to FluxModule as well, while
the local time is computed within the module using the position vectors of the spacecraft, Sun and Earth.

Note that the `Kp index <https://www.spaceweatherlive.com/en/help/the-kp-index.html>`__ (global geomagnetic activity
index) ranges from 0 to 9, with sub-indices '-','o' and '+' and therefore a total number of 28 indices (:math:`K_p` index 0-
and 9+ do not exist). In this module, the Kp index is specified with a string of length 2, for example '5-'.
