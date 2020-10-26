Executive Summary
-----------------
This module acts as an attitude reference generator based on attitude-driven formation control laws whose gain matrices are derived elsewhere. Specifically, it:

1.  Obtains a relative state in the form of a :ref:HillRelStateFswMsg`
2.  Applies a defined gain matrix to the state to obtain a relative attitude
3.  Maps that attitude given a reference attitude to a full attitude reference message
4.  Writes out a :ref:`AttRefFswMsg` describing the current commanded attitude


Module Assumptions and Limitations
----------------------------------


Message Connection Descriptions
-------------------------------


.. table:: Module I/O Messages
        :widths: 25 25 100

        +-----------------------+---------------------------------+---------------------------------------------------+
        | Msg Variable Name     | Msg Type                        | Description                                       |
        +=======================+=================================+===================================================+
        | sunInMsgName          | :ref:`SpicePlanetStateSimMsg`   | Describes sun position.                           |
        +-----------------------+---------------------------------+---------------------------------------------------+
        | stateInMsgName        | :ref:`SCPlusStatesSimMsg`       | Describes spacecraft position, attitude.          |
        +-----------------------+---------------------------------+---------------------------------------------------+
        | sunEclipseInMsgName   | :ref:`EclipseSimMsg`            | Optional input message. Describes shadow factor   |
        |                       |                                 | due to planetary bodies.                          |
        +-----------------------+---------------------------------+---------------------------------------------------+


User Guide
----------
