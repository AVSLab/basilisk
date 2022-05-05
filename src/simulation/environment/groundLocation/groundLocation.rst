
Executive Summary
-----------------
This module allows the user to specify a single celestial-body fixed ground location :math:`L`.
Reading in the planet's ephemeris state the position of :math:`L` relative to the planet origin and
the inertial frame are evaluated at output as a message.

Further, one or more spacecraft states can be added to this module to compute the spacecraft position
relative to the ground location :math:`L`.  The associated output access output message contains the relative
position vector in terms the spherical coordinates range, azimuth and elevation angle, as well as in terms of
South-East-Zenith (SEZ) coordinates.  Finally, the velocity of the spacecraft as seen by the :math:`L` location
is provided in terms of range, azimuth, elevation, south east and zenith rates.  Finally, this access message
has a integer variable indicating if the spacecraft is above a minimum elevation angle of the :math:`L` SEC frame.



Module Assumptions and Limitations
----------------------------------
This module assumes that the location is affixed to a spherical body with a constant radius.
Elevation constraints are computed assuming
a conical field of view around the normal vector from the body's surface at the location.

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
    * - currentGroundStateOutMsg
      - :ref:`GroundStateMsgPayload`
      - ground location output message
    * - accessOutMsgs
      - :ref:`AccessMsgPayload`
      - vector of ground location access messages


Detailed Module Description
---------------------------
The ``groundLocation`` module handles the following behavior:

#. Reads in the planet's states using the ``planetInMsg`` input message
#. Conversion of latitude, longitude, altitude coordinates to planet-centered, planet-fixed coordinates
#. Conversion of relative position vector in SEZ range, azimuth, elevation, south, east and zenith coordinates
#. Determine the :math:`L` relative spacecraft velocity as range, azimuth, elevation, south,
   east and zenith coordinate rates
#. Computation of spacecraft visibility (i.e. access) considering range and ground location field-of-view constraints
#. Support for multiple spacecraft given one groundLocation instance


Determining States
~~~~~~~~~~~~~~~~~~
The position of the spacecraft in the SEZ frame, relative to the ground station, is parameterized
by the cartesian coordinates:

.. math:: \mathbf{r}_{B/L} = x\hat{S} + y\hat{E} + z\hat{Z}
    :label: eq:SEZ_coords:1

The cartesian coordinates are converted to spherical coordinates, centered at the ground station position,
which give the range (:math:`\rho`), azimuth(:math:`Az`), and elevation(:math:`El`).

.. math:: \rho = \sqrt{x^2 + y^2 + z^2}
    :label: eq:rho:2

.. math:: Az = \tan{\frac{y}{x}}
    :label: eq:az:3

.. math:: El = \tan{\frac{z}{\sqrt{x^2+y^2}}}
    :label: eq:el:4

.. _glAzElSketch:
.. figure:: /../../src/simulation/environment/groundLocation/_Documentation/Images/AzEl_diagram/AzEl_sketch.jpg
    :align: center

    Figure 1: Diagram of the Azimuth and Elevation, in the South-East-Zenith frame

The spherical coordinate rates are computed by differentiating the range, azimuth, and elevation with respect to the rotating SEZ frame.

.. math:: \dot{\rho} = \frac{{}^L\text{d}}{\text{d}t} \rho = \frac{x\dot{x}+y\dot{y}+z\dot{z}}{\sqrt{x^2 + y^2 + z^2}}
    :label: eq:SEZ_rhoDot:5

.. math:: \dot{Az} = \frac{{}^L\text{d}}{\text{d}t} Az = \frac{1}{1+\frac{y^2}{x^2}} \bigg( \frac{y\dot{x}-x\dot{y}}{x^2} \bigg)
    :label: eq:SEZ_AzDot:6

.. math:: \dot{El} = \frac{{}^L\text{d}}{\text{d}t} El = \frac{1}{1+\frac{z^2}{x^2+y^2}} \bigg( \frac{\dot{z}}{\sqrt{x^2+y^2}} - \frac{z(x\dot{x}+y\dot{y})}{(x^2+y^2)^{3/2}} \bigg)
    :label: eq:SEZ_ElDot:7


User Guide
----------

To use this module, instantiate the class and provide it with a body-fixed location (in either lat/long/altitude,
via the specifyLocation method, or in
planet-centered planet-fixed coordinates directly via the ``r_LP_P_Init`` attribute) and a planet position/attitude
message (i.e., an instance of :ref:`SpicePlanetStateMsgPayload`);
to compute access, at least one :ref:`SCStatesMsgPayload` message must be added to the module using the ``addSpacecraftToModel()`` method.
The first spacecraft is 0, the second is 1, and so on.

A new instance of groundLocation, alongside necessary user-supplied parameters, can be created by calling:

.. code-block:: python

    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 100e3 # Sets maximum range for visibility in meters
    groundTarget.minimumElevation = np.radians(10.) #   Sets necessary minimum elevation for visibility to 10 deg in radians
    groundTarget.specifyLocation(np.radians(0.), np.radians(0.), 0.) #  Sets location in latitude, longitude, altitude coordinates
    scSim.AddModelToTask(simTaskName, groundTarget)

The ``planetRadius`` variable is optional and defaults to Earth's radius.  Instead of specifying the ground location
through the ``specifyLocation()`` method, you can also set the module variable ``r_LP_P_Init`` using the
``specifyLocationPCPF()`` method. Avoid setting the module variable ``r_LP_P_Init`` directly, as there are computations
that occur within the ``specifyLocation()`` and ``specifylocationPCPF()`` methods that are important for access message
states.

The ``maximumRange`` variable is optional and defaults to -1.  This means by default no maximum range is considered.  Set it to a positive value to have ``hasAccess`` output message variable depend on range.

A groundLocation can be affixed to a specific planet by setting its ``planetInMsg`` input message:

.. code-block:: python

    groundTarget.planetInMsg.subscribeTo(planetMsg)

Spacecraft can be added to the model by calling:

.. code-block:: python

    groundTarget.addSpacecraftToModel(sc1.scStateOutMsg)
    groundTarget.addSpacecraftToModel(sc2.scStateOutMsg)

    #   log code
    dataLog0 = groundTarget.currentGroundStateOutMsg.recorder()
    dataLog1 = groundTarget.accessOutMsgs[0].recorder()
    dataLog2 = groundTarget.accessOutMsgs[1].recorder()
