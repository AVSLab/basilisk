Executive Summary
-----------------

General magnetic field base class used to calculate the magnetic field above a planet using multiple models. The MagneticField class is used to calculate the magnetic field vector above a body using multiple models. This base class is used to hold relevant planetary magnetic field properties to compute answers for a given set of spacecraft locations relative to a specified planet.  Specific magnetic field models are implemented as classes that inherit from this base class. Planetary parameters, including position and input message, are settable by the user. In a given simulation, each planet of interest should have only one magnetic field  model associated with it linked to the spacecraft in orbit about that body.

Each magnetic field is attached to a specific planet, but provides support for
multiple spacecraft through ``addSpacecraftToModel()``.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: magneticFieldBase
    :caption: Module I/O Messages

    input scStateInMsgs SCStatesMsgPayload
        vector of spacecraft state input messages.
    output envOutMsgs MagneticFieldMsgPayload
        vector of magnetic density output messages.
    input planetPosInMsg SpicePlanetStateMsgPayload
        (optional) planet state input message.  If not provided the planet state is zero information.
    input epochInMsg EpochMsgPayload
        (optional) epoch date/time input message. The date and time are interpreted as UTC, independent of the time zone of the computer.

Spacecraft State Timing
-----------------------
The module is normally executed before the spacecraft within a task, so the spacecraft state message it reads was written
at the end of the previous step. By default the message is used as written. To avoid an output that lags the interval it is applied
to, the extrapolation can be enabled with ``setExtrapolateScStateToStepMidpoint(True)``. The spacecraft position is
then advanced with the message velocity by half of the message age, which is the middle of the interval that the next
spacecraft update integrates. It is skipped for a message that was written at the current time and for a stale
message that was written before the previous update of the module, for example a state written once at the start of
the simulation.

.. warning::

    The extrapolation requires that the module is updated with the same task period as the spacecraft, so that the state
    message written by the spacecraft at the previous module update is exactly one module interval old. If the task
    periods differ, whether the spacecraft is faster or slower than the module, half of the message age is not the
    middle of the module interval. The state is then not extrapolated: the planets and all the spacecraft of the module
    are left at the epoch of their messages, so that every spacecraft is evaluated against the same planet epoch. A
    warning is logged once when a spacecraft message is found not to have been written at the previous module update. A
    task period mismatch is not always detectable this way: a message written at the current module update, for example
    by a faster spacecraft that runs before the module, is used as written and gives no warning. No message that is not
    the output of the previous module update is ever extrapolated. A task period that changes during the simulation has
    the same effect. The extrapolation only starts once two write times of the spacecraft state message have been
    observed and their interval equals the module update interval, so the first updates are not extrapolated. Run the
    module in the same task as the spacecraft, or at the same period.

When the extrapolation is enabled the position and the orientation of the planet (or of the sun and the planets for
the eclipse) are advanced, or moved back, with their message velocity and angular rate to the same middle of the
interval as the spacecraft, so that the relative geometry is evaluated at a single epoch. Other time-dependent inputs,
such as the epoch used for solar time or a space weather sample, are not shifted and are evaluated at the current
time.
As in the gravity effector, the planet is advanced by the time since its message was written, whatever the age of the
message, so a planet message written once with a non-zero velocity is projected forward over the whole simulation.
