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

- If ``epochInMsg`` is linked, the epoch is read from that message.
- Otherwise, ``customSetEpochFromVariable()`` is called, giving subclasses the
  opportunity to set the epoch from a module-level variable.

Message Connection Descriptions
--------------------------------
The following table lists all the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scStateInMsgs
      - :ref:`SCStatesMsgPayload`
      - Spacecraft state input messages (vector). Use ``addSpacecraftToModel()``
        to add spacecraft and automatically create corresponding output messages.
    * - envOutMsgs
      - :ref:`WindMsgPayload`
      - Atmospheric wind velocity output messages (vector). Automatically created when
        spacecraft are added via ``addSpacecraftToModel()``. Each message contains:
        ``v_air_N`` (full air velocity in inertial frame) and ``v_wind_N``
        (wind perturbation velocity), both expressed in inertial frame N
    * - planetPosInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Planet SPICE state input message. Provides ``PositionVector`` used to compute ``r_BP_N``.
        Must be connected before ``InitializeSimulation()``.
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (Optional) Epoch date/time message. When connected, overrides the default
        Basilisk epoch stored in ``epochDateTime``. Required by empirical wind
        models that depend on calendar date.
