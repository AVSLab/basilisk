Executive Summary
-----------------

Abstract base class for atmospheric wind models. ``WindBase`` reads spacecraft
state input messages and writes wind velocity output messages in the inertial frame.
Concrete subclasses implement ``evaluateWindModel()`` to fill the
:ref:`WindMsgPayload` for each tracked spacecraft. The module supports multiple
spacecraft through the ``addSpacecraftToModel()`` method.

Module Description
------------------

**Multi-spacecraft support.**
``WindBase`` supports multiple spacecraft through the ``addSpacecraftToModel()``
method. Each call to this method subscribes to a spacecraft's state message and
creates a corresponding wind output message. The module processes all connected
spacecraft in each update cycle, computing wind velocities for each spacecraft's
position.

**Planet-relative position.**
At every time step, ``WindBase`` computes ``r_BP_N`` as each spacecraft's position
relative to the planet center in the inertial frame.  ``r_BN_N`` is read from
:ref:`SCStatesMsgPayload` and ``r_PN_N`` (stored as ``PositionVector``) is read
from :ref:`SpicePlanetStateMsgPayload`:

.. math::

   \mathbf{r}_{BP,N} = \mathbf{r}_{BN,N} - \mathbf{r}_{PN,N}

This correctly handles simulations where the planet is not at the inertial origin
(e.g., heliocentric simulations).

**Planet angular velocity.**
``WindBase`` provides two modes for the planet angular velocity used in co-rotation calculations,
selected via ``setUseSpiceOmegaFlag()``:

- **SPICE mode** (default, ``True``): angular velocity is derived from ``J20002Pfix_dot`` each time
  step. Falls back to the manually set value only when ``planetPosInMsg`` has not been written to.
- **Manual mode** (``False``): the value set via ``setPlanetOmega_N()`` is always used
  (default: Earth rotation rate).

**Epoch handling.**
For time-dependent empirical wind models, ``WindBase`` maintains an ``epochDateTime``
structure (``struct tm``) initialised to the Basilisk standard epoch (2019-01-01
00:00:00).  During ``Reset()``:

- If ``epochInMsg`` is linked, the epoch is read from that message. It is interpreted as UTC, independent of the time
  zone of the computer.
- Otherwise, ``customSetEpochFromVariable()`` is called, giving subclasses the
  opportunity to set the epoch from a module-level variable.

Message Connection Descriptions
--------------------------------
The following table lists all the module input and output messages.

.. bsk-module-io:: windBase
    :caption: Module I/O Messages

    input scStateInMsgs SCStatesMsgPayload
        Spacecraft state input messages (vector). Use ``addSpacecraftToModel()`` to add spacecraft and automatically create corresponding output messages.
    output envOutMsgs WindMsgPayload
        Atmospheric wind velocity output messages (vector). Automatically created when spacecraft are added via ``addSpacecraftToModel()``. Each message contains: ``v_air_N`` (full air velocity in inertial frame) and ``v_wind_N`` (wind perturbation velocity), both expressed in inertial frame N.
    input planetPosInMsg SpicePlanetStateMsgPayload
        Planet SPICE state input message. Provides ``PositionVector`` used to compute ``r_BP_N``. Must be connected before ``InitializeSimulation()``.
    input epochInMsg EpochMsgPayload
        (Optional) Epoch date/time message. When connected, overrides the default Basilisk epoch stored in ``epochDateTime``. Required by empirical wind models that depend on calendar date.

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

    The extrapolation requires that the module is updated with the same task period as the spacecraft, so that the
    state message written by the spacecraft at the previous module update is exactly one module interval old. If the
    task periods differ, whether the spacecraft is faster or slower than the module, half of the message age is not the
    middle of the module interval. The state is then not extrapolated to the midpoint: the spacecraft stay at the epoch
    of their messages, and the planets are moved to that epoch if all the spacecraft messages were written at the
    previous module update (they are left as written otherwise), so that every spacecraft is evaluated against the same
    planet epoch. A warning is logged once when a spacecraft message is found not to have been written at the previous
    module update. A task period mismatch is not always detectable this way: a message written at the current module
    update, for example by a faster spacecraft that runs before the module, is used as written and gives no warning. No
    message that is not the output of the previous module update is ever extrapolated. A task period that changes
    during the simulation has the same effect. The extrapolation only starts once two write times of the spacecraft
    state message have been observed and their interval equals the module update interval, so the first updates are not
    extrapolated. Until then the spacecraft state is used as written and the planet is moved back to the epoch of that
    state, so the relative geometry stays consistent. Run the module in the same task as the spacecraft, or at the same
    period.

When the extrapolation is enabled the position and the orientation of the planet (or of the sun and the planets for
the eclipse) are advanced, or moved back, with their message velocity and angular rate to the same middle of the
interval as the spacecraft, so that the relative geometry is evaluated at a single epoch. Time-dependent models, such
as the local solar time or the decimal year of the epoch, are evaluated at the same epoch as the geometry: the middle
of the interval if the extrapolation applies, the epoch of the previous update while the spacecraft state is not yet
extrapolated, and the current time otherwise. Inputs read from messages, such as a space weather sample, are not
shifted.
As in the gravity effector, the planet is advanced by the time since its message was written, whatever the age of the
message, so a planet message written once with a non-zero velocity is projected forward over the whole simulation.
