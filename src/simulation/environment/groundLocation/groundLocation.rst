
Executive Summary
-----------------
This class is used to represent a point attached to the surface of a planetary body which computes the access of a set of spacecraft.
It considers both the relative position of a set of spacecraft to the location and a minimum elevation for the
location (set to 10 deg by default).

To use this module, instantiate the class and provide it with a body-fixed location (in either lat/long/altitude,
via the specifyLocation method, or in
planet-centered planet-fixed coordinates directly via the ``r_LP_P_Init`` attribute) and a planet position/attitude
message (i.e., an instance of :ref:`SpicePlanetStateMsgPayload`);
to compute access, at least one :ref:`scPlusStatesMsgPayload` message must be added to the module using the ``addSpacecraftToModel()`` method.
The first spacecraft is 0, the second is 1, and so on.

Module Assumptions and Limitations
----------------------------------
This module assumes that it is affixed to a spherical body with a constant radius. Elevation constraints are computed assuming
a conical field of view around the normal vector fom the body's surface at the location.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. table:: Module I/O Messages
    :widths: 25 25 100

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) planet state input message. Default is a zero state for the planet.
    * - vec:scStateInMsgs
      - :ref:`SCPlusStatesMsgPayload`
      - vector of sc state input messages.  These are set through ``addSpacecraftToModel()``
    * - currentGroundStateOutMsg
      - :ref:`GroundStateMsgPayload`
      - ground location output message
    * - vec:accessOutMsgs
      - :ref:`AccessMsgPayload`
      - vector of ground location access messages


Detailed Module Description
---------------------------
The ``groundLocation`` module handles the following behavior:

#. Body-fixed location representation: a single groundLocation instance represents one body-fixed location on a
   body, including translation and rotation due to the motion of that body as computed by a module that
   writes a SPICEPlanetStateSimMsg.
#. Conversion of latitude, longitude, altitude coordinates to planet-centered, planet-fixed coordinates
#. Computation of spacecraft visibility (i.e. access) considering range and ground location field-of-view constraints
#. Support for multiple spacecraft given one groundLocation instance

User Guide
----------
A new instance of groundLocation, alongside necessary user-supplied parameters, can be created by calling:

.. code-block:: python

    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 100e3 # Sets maximum range for visibility in meters
    groundTarget.minimumElevation = np.radians(10.) #   Sets necessary minimum elevation for visibility to 10 deg in radians
    groundTarget.specifyLocation(np.radians(0.), np.radians(0.), 0.) #  Sets location in latitude, longitude, altitude coordinates
    scSim.AddModelToTask(simTaskName, groundTarget)


A groundLocation can be affixed to a specific planet by setting its ``planetInMsg`` input message:

.. code-block:: python

    groundTarget.planetInMsg.subscribeTo(planetMsg)

Spacecraft can be added to the model by calling:

.. code-block:: python

    groundTarget.addSpacecraftToModel(sc1.scStateOutMsg)
    groundTarget.addSpacecraftToModel(sc2.scStateOutMsg)
    groundTarget.addSpacecraftToModel(sc2.scStateOutMsg)

    #   log code
    dataLog0 = groundTarget.currentGroundStateOutMsg.log()
    dataLog1 = groundTarget.accessOutMsgs[0].log()
