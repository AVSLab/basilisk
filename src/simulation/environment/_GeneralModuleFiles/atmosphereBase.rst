Executive Summary
-----------------

General atmosphere base class used to calculate neutral density/temperature using arbitrary models
The Atmosphere class is used to calculate the neutral density and temperature above a body using arbitrary models.
Each atmosphere is attached to a specific planet, but provides support for
multiple spacecraft through ``addSpacecraftToModel()``.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: atmosphereBase
    :caption: Module I/O Messages

    input scStateInMsgs SCStatesMsgPayload
        vector of spacecraft state input messages.
    output envOutMsgs AtmoPropsMsgPayload
        vector of atmospheric density output messages.
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

Planet Shape
------------
By default the planet is a sphere and the altitude is the distance to the planet center minus ``planetRadius``. If the
planet polar radius is set with ``setPlanetPolarRadius()``, the altitude (and, for models that use it such as
:ref:`msisAtmosphere`, the latitude) is computed above the oblate ellipsoid defined by ``planetRadius`` and the polar
radius, and ``planetPosInMsg`` must be connected to provide the planet orientation. For the Earth use the equatorial
radius ``REQ_EARTH * 1000`` and the polar radius ``RP_EARTH * 1000`` from ``astroConstants``::

    atmosphere.setPlanetPolarRadius(orbitalMotion.RP_EARTH * 1000.0)  # [m]

A negative polar radius, the default, selects the sphere. The difference between the two altitudes is up to about 21 km,
which changes the density of an atmosphere model by tens of percent at mid and high latitudes.
