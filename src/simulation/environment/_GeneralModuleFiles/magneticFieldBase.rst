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

    The extrapolation assumes that the module and the spacecraft run at the same task rate and that the spacecraft
    uses a constant step. If the task rates differ, the position offset alternates between extrapolated and
    unextrapolated on successive steps, and the module input jumps by about ``v * dt / 2``. A spacecraft with a
    variable step, such as a variable-step integrator or a changing task period, has the same problem, because the
    message age no longer matches the interval the spacecraft integrates. A warning is logged once if a different
    task rate is detected while the extrapolation is enabled. Run the module in the same task and at the same rate as
    the spacecraft.
