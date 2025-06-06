
Executive Summary
-----------------
This module determines if one satellite has a line of sight vector to another satellite.  An oblate planet is modeled through the equatorial and the polar radius. If the line of sight vector between two satellites is above this surface, then the output access message is set to true.  The module has a primary spacecraft which determines access to `N` other spacecraft orbiting the same planet.

Further, the module tracks a body-fixed location `L` on the primary spacecraft (i.e. location where an antenna is attached), and you can specify an optional sensor/communication bore sight axis :math:`\hat{\bf a}` and a center-to-edge sensor/communication cone angle :math:`\theta`.  In this case the access message is true if the line of sight vector between spacecraft is above the surface and the relative position vector to the other spacecraft is within this cone.

Lighting conditions are also considered. If `theta_solar` is set, the module will check that the normal :math:`\hat{\bf a}` is illuminated by the sun with at most that incidence angle.

Finally, if the other spacecraft is accessible, the range to the other satellite is stored in the output message.


Module Assumptions and Limitations
----------------------------------
This module assumes all spacecraft are orbiting the same planet.  The planet shape is assumed to be an ellipsoid specified through the equatorial and polar radius.  To account for a planet's atmosphere, increase these radii to account for the atmospheric height.

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
    * - primaryScStateInMsg
      - :ref:`SCStatesMsgPayload`
      - primary spacecraft state message relative to which to evaluate access to other spacecraft in ``scStateInMsgs``
    * - scStateInMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of other spacecraft state input messages.  These are set through ``addSpacecraftToModel()``
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) sun state input message. Used for illumination checking if the message is connected and ``theta_solar`` is set.
    * - eclipseInMsg
      - :ref:`EclipseMsgPayload`
      - (optional) eclipse input message. Used for illumination checking if the message is connected and ``min_shadow_factor`` is set.
    * - accessOutMsgs
      - :ref:`AccessMsgPayload`
      - output vector of ground location access messages


Detailed Module Description
---------------------------
The ``spacecraftLocation`` module handles the following behavior:

#. Spacecraft body-fixed location representation: a single ``spacecraftLocation`` instance represents one body-fixed location on a
   spacecraft
#. The planet-centered planet-fixed frame is either a zero state (default), or read in through an optional planet ephemeris message
#. An optional sensor axis can be specified to ensure the primary spacecraft is pointing this axis
#. Computation of spacecraft visibility (i.e. access) considering range and relative field-of-view constraints
#. Support for multiple other spacecraft given one ``spacecraftLocation`` instance


Determining Line of Sight to another spacecraft
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Let `P` be the planet-centered planet-fixed frame.  The inertial planet origin is given by :math:`{}^{N} {\bf r}_{P/N}`.  The inertial position vector of the primary spacecraft `B` is :math:`{}^{N}{\bf r}_{B/N}`, while the inertial position vector to the other satellite is :math:`{}^{\cal N}{\bf r}_{S/N}`.  The first step is to determine the spacecraft positions, expressed in the `P` frame.

.. math::
    {}^{P} {\bf r}_{B/P} = [PN] ( {}^{N} {\bf r}_{B/N} - {}^{N} {\bf r}_{P/N})

.. math::
    {}^{P} {\bf r}_{S/P} = [PN] ( {}^{N} {\bf r}_{S/N} - {}^{N} {\bf r}_{P/N})

.. math::
    {}^{P} {\bf r}_{S/B} = {}^{P} {\bf r}_{S/P} - {}^{P} {\bf r}_{B/P}

The line of sight vector is defined through the point `B` and the direction :math:`{\bf r}_{S/B}`.  Let :math:`\kappa` be a scaling factor, the line is then defined as:

.. math::
    {\bf l} =  {\bf r}_{B/P} + \kappa * {\bf r}_{S/B}

The process of intersecting a line with an ellipsoid is simplified by using an affine project to render the ellipsoid a sphere.  This affine mapping preserves a line to remain a line.  The math of this is as follow.  The planet is assumed to have rotational symmetry about the 3:sup`rd` axis about which it is spinning.  Thus, to map the ellipsoid into a sphere the planet relative 3:sup`rd` coordinates must scaled by the ratio of the equatorial radius to the polar radius.  The following math assumes this affine mapping has been performed in the above planet-relative position coordinates.

To determine the minimum distance of the line :math:`\bf l` to the planet origin, consider the cost function :math:`J`:

.. math::
    J = {\bf l} \cdot {\bf l} = {\bf r}_{B/P} \cdot {\bf r}_{B/P} + 2 \kappa {\bf r}_{B/P} \cdot {\bf r}_{S/B} + \kappa^2 {\bf r}_{S/B} \cdot {\bf r}_{S/B}

Setting the first variation of :math:`J` with respect to :math:`\kappa` to zero and solving for optimal :math:`\kappa^\ast` yields

.. math::
    \kappa^\ast = - \frac{{\bf r}_{B/P} \cdot {\bf r}_{S/B}}{{\bf r}_{S/B} \cdot {\bf r}_{S/B}}

Note that if :math:`\kappa<0` or :math:`\kappa>1`, then the point of closed approach is outside of the
line-of-sight interval between the two spacecraft and the planet cannot be blocking access
of one spacecraft from another.

Thus, the point of closed approach is determined through:

.. math::
    {\bf r}^\ast = {\bf r}_{B/P} + \kappa^\ast * {\bf r}_{S/B}

If :math:`|{\bf r}^\ast| > r_{\text{eq}}` then the other spacecraft is visible relative to the primary spacecraft.


Determining Sensor Cone Inclusion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
If the line of sight property is established, then the module can also take into consideration a sensor or communication boresight axis :math:`\hat {\bf a}` which is fixed relative to the primary spacecraft body frame.  The angle :math:`\phi` between the relative position vector and this body fixed unit direction vector is found through:

.. math::
    \phi = \arccos \left( \frac{ {\bf r}_{S/B} \cdot \hat{\bf a}}{|{\bf r}_{S/B} |} \right)

The module sets the sensor cone half-angle :math:`\theta`.  If :math:`\phi > \theta` then the sensor or communication axis does not have access to the other spacecraft.

If this :math:`\hat{\bf a}` is considered, then the access output message sets the message elevation angle as

.. math::

    \text{elevation} = \frac{\pi}{2} - \phi

Determining Solar Illumination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
If the ``sunInMsg`` is connected and :math:`\theta_{\text{solar,max}}` is set, the illumination of the surface is considered, treating :math:`\hat{\bf a}` as the surface normal. Taking :math:`\hat{\bf s}` as the sun vector, the illumination incidence condition is met if

.. math::
    \arccos \left( \hat{\bf a} \cdot \hat{\bf s} \right) = \theta_{\text{solar}} \le \theta_{\text{solar,max}}

If an eclipse message is connected and ``min_shadow_factor`` is set, the module will also check that the shadow factor is above this threshold.

User Guide
----------
A new instance of ``spacecraftLocation``, alongside necessary user-supplied parameters, can be created by calling:

.. code-block:: python

    location = spacecraftLocation.SpacecraftLocation()
    location.ModelTag = "scLocation"
    location.rEquator = orbitalMotion.REQ_EARTH * 1000.
    location.rPolar = orbitalMotion.RP_EARTH * 1000.  # optional, include to account for oblateness
    location.maximumRange = 100e3 # optinal, sets maximum range for visibility in meters
    scSim.AddModelToTask(simTaskName, location)

The variable ``maximumRange`` is optional and set to -1 by default.  If it is set to a positive value, then the ``hasAccess`` variable is only set to 1 if the relative spacecraft distance is less than this maximum range.

A optional planet emphemeris is connected via the``planetInMsg`` input message:

.. code-block:: python

    location.planetInMsg.subscribeTo(planetMsg)

It this message is not connected, then zero planet position and attitude orientation are set.


To set a primary spacecraft body fixed sensor or communication axis :math:`\hat{\bf a}` and half-cone angle :math:`\theta`, use::

    module.aHat_B = [xxx, xxx, xxx]
    module.theta = xxx * macros.D2R


Spacecraft can be added to the model by calling::

    location.addSpacecraftToModel(sc1.scStateOutMsg)
    location.addSpacecraftToModel(sc2.scStateOutMsg)

The access output messages can be logged through::

    dataRec0 = location.accessOutMsgs[0].recorder()
    dataRec1 = location.accessOutMsgs[1].recorder()
