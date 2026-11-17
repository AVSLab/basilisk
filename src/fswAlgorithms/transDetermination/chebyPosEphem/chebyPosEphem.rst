Executive Summary
-----------------

This module allows the user to specify a set of Chebyshev
coefficients to fit a space ephemeris trajectory. Next the input time is used to determine where a given body is in space.

Module Assumptions and Limitations
----------------------------------

Each configured record requires a positive ``ephemTimeRad`` in seconds and
between one and ``MAX_CHEB_COEFF`` coefficients per axis. The coefficients
describe position in meters as a function of normalized time over the record's
interval. The input clock correlation maps simulation time to ephemeris time.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.


.. bsk-module-io:: chebyPosEphem
    :caption: Module I/O Messages

    output posFitOutMsg EphemerisMsgPayload
        Output navigation message for pos/vel
    input clockCorrInMsg TDBVehicleClockCorrelationMsgPayload
        clock correlation input message

User Guide
----------

Set each record's ``nChebCoeff``, ``ephemTimeMid``, ``ephemTimeRad``, and
``posChebyCoeff`` before initializing the simulation. Store the position
coefficients consecutively for the x, y, and z axes, with ``nChebCoeff`` values
per axis. The module derives ``velChebyCoeff`` in meters per second during reset;
users do not need to populate it.

After changing position coefficients, their count, or the time radius, call
the module's ``Reset()`` method or reinitialize the simulation with
``InitializeSimulation()`` before evaluating the new fit. Every reset clears
and rebuilds the velocity coefficients for every record, so changing the
coefficient count or resetting repeatedly does not retain a previous fit's
derivative coefficients.
