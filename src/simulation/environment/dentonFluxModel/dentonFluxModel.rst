Executive Summary
-----------------
This module provides the 10-year averaged GEO elecon and ion flux as discussed in the paper by Denton.


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
    * - ``scStateInMsg``
      - :ref:`SCStatesMsgPayload`
      - spacecraft state input message
    * - ``earthStateInMsg``
      - :ref:`SpicePlanetStateMsgPayload`
      - Earth planet state input message
    * - ``sunStateInMsg``
      - :ref:`SpicePlanetStateMsgPayload`
      - sun state input message
    * - ``fluxOutMsg``
      - :ref:`PlasmaFluxMsgPayload`
      - output ion and electron fluxes

