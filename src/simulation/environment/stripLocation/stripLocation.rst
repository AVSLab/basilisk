
Executive Summary
-----------------
This module extends the :ref:`groundLocation` concept to model a **strip imaging target** on a
celestial body's surface.  Instead of a single fixed point, the user defines a ground strip by
specifying a **start** and an **end** location (both body-fixed).  The module uses spherical
linear interpolation (SLERP) to sweep a moving target point :math:`L` along the great-circle arc
connecting these two locations at a configurable constant acquisition speed. A **pre-imaging time**
parameter allows the strip's effective start point to be extrapolated backwards along the great-circle
arc so that the target point begins moving before the nominal start location, giving the spacecraft
time to slew.

The inertial position and velocity of the current target point :math:`L` are evaluated and output as
a :ref:`StripStateMsgPayload` message.

One or more spacecraft states can be added to compute each spacecraft's position relative
to the current target point :math:`L`.  The associated :ref:`AccessMsgPayload` output message contains the
relative position vector in spherical coordinates (range, azimuth, elevation) and in
South-East-Zenith (SEZ) Cartesian coordinates, as well as the corresponding coordinate rates.
An integer flag indicates whether the spacecraft has access (i.e. is above the minimum
elevation angle of the :math:`L` SEZ frame, within range, and the pre-imaging time has elapsed).

Module Assumptions and Limitations
----------------------------------
- The body is assumed to be a **sphere** with a constant radius (``planetRadius``).
- The strip's central line is a great-circle arc on that sphere; local terrain is not considered.
- The target point moves along the arc at a **constant acquisition speed** (``acquisitionSpeed``).
- Elevation constraints are computed assuming a conical field of view around the surface normal
  at the current target location.
- The pre-imaging time feature uses the same SLERP extrapolation, extending the arc backwards.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) planet state input message. Default is a zero state for the planet.
    * - scStateInMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of sc state input messages.  These are set through ``addSpacecraftToModel()``
    * - currentStripStateOutMsg
      - :ref:`StripStateMsgPayload`
      - strip location output message providing the current target point inertial position and velocity
    * - accessOutMsgs
      - :ref:`AccessMsgPayload`
      - vector of access output messages (one per spacecraft)


Detailed Module Description
---------------------------
The ``stripLocation`` module handles the following behavior:

#. Reads in the planet's states using the ``planetInMsg`` input message.
#. Converts the start and end latitude/longitude/altitude coordinates to
   planet-centered, planet-fixed (PCPF) coordinates.
#. (Optionally) Computes an updated start point by extrapolating backwards along the
    great-circle arc using the ``preImagingTime`` parameter.
#. Propagates the current target point :math:`L` along the great-circle arc using SLERP,
    at the constant ``acquisitionSpeed``.
#. Computes the inertial position and velocity of :math:`L`, accounting for planet rotation.
#. Converts each spacecraft's relative position to the SEZ frame and evaluates range,
   azimuth, elevation, and their rates.
#. Determines spacecraft access (visibility) considering minimum elevation, maximum range,
   and whether the pre-imaging time has elapsed.
#. Supports multiple spacecraft given one ``stripLocation`` instance.


Target Propagation along the Strip
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Given a start point :math:`\mathbf{p}_s` and end point :math:`\mathbf{p}_e` on the sphere of
radius :math:`R`, both expressed in PCPF coordinates, the central angle is:

.. math:: \theta = \arccos\!\left(\frac{\mathbf{p}_s \cdot \mathbf{p}_e}{R^2}\right)
    :label: eq:theta:1

The arc length of the central line is:

.. math:: \ell = \theta \, R
    :label: eq:arc_length:2

A normalised time parameter :math:`\tau \in [0,1]` is computed from the elapsed imaging time
:math:`\Delta t` and the acquisition speed :math:`v_a`:

.. math:: \tau = \frac{\Delta t}{\ell / v_a}
    :label: eq:tau:3

The target position is obtained via spherical linear interpolation (SLERP):

.. math:: \mathbf{p}(\tau) = \frac{\sin\!\bigl((1-\tau)\,\theta\bigr)}{\sin\theta}\,\mathbf{p}_s
          + \frac{\sin(\tau\,\theta)}{\sin\theta}\,\mathbf{p}_e
    :label: eq:slerp:4

and projected back onto the sphere:

.. math:: \mathbf{r}_{L/P}^P = R \, \frac{\mathbf{p}(\tau)}{\|\mathbf{p}(\tau)\|}
    :label: eq:project:5

The PCPF velocity of the target is obtained by differentiating the SLERP expression and scaling
to the acquisition speed.


Determining Spacecraft-to-Target States
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The position of the spacecraft relative to the current target point in the SEZ frame is
parameterized by the Cartesian coordinates:

.. math:: \mathbf{r}_{B/L} = x\hat{S} + y\hat{E} + z\hat{Z}
    :label: eq:strip_SEZ_coords:1

The Cartesian coordinates are converted to spherical coordinates centered at the target point,
giving the range (:math:`\rho`), azimuth (:math:`Az`), and elevation (:math:`El`):

.. math:: \rho = \sqrt{x^2 + y^2 + z^2}
    :label: eq:strip_rho:2

.. math:: Az = \arctan\!\frac{y}{x}
    :label: eq:strip_az:3

.. math:: El = \arctan\!\frac{z}{\sqrt{x^2+y^2}}
    :label: eq:strip_el:4

The spherical coordinate rates are computed by differentiating the range, azimuth, and elevation
with respect to the rotating SEZ frame:

.. math:: \dot{\rho} = \frac{x\dot{x}+y\dot{y}+z\dot{z}}{\sqrt{x^2 + y^2 + z^2}}
    :label: eq:strip_SEZ_rhoDot:5

.. math:: \dot{Az} = \frac{-x\dot{y}+y\dot{x}}{x^2+y^2}
    :label: eq:strip_SEZ_AzDot:6

.. math:: \dot{El} = \frac{1}{1+\frac{z^2}{x^2+y^2}} \left( \frac{\dot{z}}{\sqrt{x^2+y^2}} - \frac{z(x\dot{x}+y\dot{y})}{(x^2+y^2)^{3/2}} \right)
    :label: eq:strip_SEZ_ElDot:7


User Guide
----------

To use this module, instantiate the class and provide it with two body-fixed locations (start and
end of the strip) and the desired acquisition speed.  A planet
position/attitude message (i.e., an instance of :ref:`SpicePlanetStateMsgPayload`) can optionally
be provided; to compute access, at least one :ref:`SCStatesMsgPayload` message must be added to
the module using the ``addSpacecraftToModel()`` method.  The first spacecraft is index 0, the
second is 1, and so on.

A new instance of ``stripLocation``, alongside necessary user-supplied parameters, can be created
by calling:

.. code-block:: python

    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.  # defaults to Earth's radius
    stripTarget.maximumRange = 100e3              # (optional) maximum slant range for access [m]
    stripTarget.minimumElevation = np.radians(10.)  # minimum elevation for access [rad]; defaults to 10 deg
    stripTarget.smallAngle = 1e-12                # (optional) threshold used to treat the strip as having zero length [rad]; defaults to 1e-12
    stripTarget.acquisitionSpeed = 3e-6           # constant acquisition speed along the strip [m/ns]; defaults to 3 km/s
    stripTarget.preImagingTime = 6e10          # (optional) pre-imaging offset time [ns]; defaults to 0
    stripTarget.specifyLocationStart(np.radians(40.0), np.radians(-105.0), 0.)  # start lat, lon, alt
    stripTarget.specifyLocationEnd(np.radians(42.0), np.radians(-103.0), 0.)    # end lat, lon, alt
    scSim.AddModelToTask(simTaskName, stripTarget)

The ``planetRadius`` variable is optional and defaults to Earth's radius.  Strip endpoints are
specified through the ``specifyLocationStart()`` and ``specifyLocationEnd()`` methods, which
accept geodetic latitude, longitude (both in radians), and altitude (in meters).  Alternatively,
the PCPF position vectors ``r_LP_P_Start`` and ``r_LP_P_End`` can be set directly.

The ``maximumRange`` variable is optional and defaults to -1, meaning no maximum range is
considered.  Set it to a positive value to have the ``hasAccess`` output depend on range.

The ``smallAngle`` variable is optional and defaults to :math:`10^{-12}` rad.  It is
used as a numerical tolerance when evaluating near-zero strip central angles.  If set to a
non-positive value, it is reset to this default at module reset.

The ``preImagingTime`` variable is optional and defaults to 0.  When set to a
positive value, the effective start point is extrapolated backwards along the great-circle arc
so that the target begins moving before the nominal start location, allowing time for
spacecraft maneuvers before the imaging pass begins.  Access is not granted until the
pre-imaging phase has elapsed.

A ``stripLocation`` can be affixed to a specific planet by setting its ``planetInMsg`` input
message:

.. code-block:: python

    stripTarget.planetInMsg.subscribeTo(planetMsg)

Spacecraft can be added to the model by calling:

.. code-block:: python

    stripTarget.addSpacecraftToModel(sc1.scStateOutMsg)
    stripTarget.addSpacecraftToModel(sc2.scStateOutMsg)

    #   log code
    dataLog0 = stripTarget.currentStripStateOutMsg.recorder()
    dataLog1 = stripTarget.accessOutMsgs[0].recorder()
    dataLog2 = stripTarget.accessOutMsgs[1].recorder()
