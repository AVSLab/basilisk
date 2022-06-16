Executive Summary
-----------------
This module checks that a vector of mapping points are visible to a spacecraft instrument, outputting a vector of access
messages for each mapping point.

Module Assumptions and Limitations
----------------------------------
This module assumes that the location is affixed to a spherical body with a constant radius. Elevation constraints are
computed assuming a conical field of view around the normal vector from the body's surface at the location. This module
also assumes the spacecraft instrument has a spherical field-of-view (FOV).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) Planet state input message
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Spacecraft state input message
    * - accessOutMsgs
      - :ref:`AccessMsgPayload`
      - vector of ground location access messages
    * - currentGroundStateOutMsgs
      - :ref:`GroundStateMsgPayload`
      - vector of ground state messages

Detailed Module Description
---------------------------
The ``groundMapping`` module loops through a number of mapping points defined in the planet-frame, checking that:

#. The spacecraft is within the range and elevation requirements of the mapping point
#. The point is within the spacecraft instrument's FOV cone

Range and Elevation
~~~~~~~~~~~~~~~~~~~
The range and elevation check follows the same logic as the :ref:`groundLocation` module.

FOV Checking
~~~~~~~~~~~~
Each mapping point is also checked to determine if its within the instrument's FOV cone. This check is performed as
follows:

#. The projection of the mapping point along the instrument's boresight is computed and stored
#. If the above projection is less than 0, the point is not within the instrument's FOV cone
#. If the above projection is greater than 0, the point may be within the instrument's FOV cone
#. A final check is performed to compare the distance from mapping point to the instrument's boresight vector. If this
   distance is less than the radius at the boresight projection distance, the point is within the instrument's FOV


User Guide
----------
To use this module, the user must first instantiate the module

.. code-block:: python

    groundMap = groundMapping.GroundMapping()
    groundMap.ModelTag = "groundMapping"

The user must then set several variables within the module. First, the minimum elevation and maximum range should be
set. The minimum elevation is the minimum required elevation of the spacecraft with respect to the mapping points. The
minimum elevation is an optional parameter, defaulting to zero. The maximum range is the farthest distance the
spacecraft can be from a mapping point to have access. This parameter is also optional, defaulting to a value of -1,
which means that no maximum range is considered.

.. code-block:: python

    groundMap.minimumElevation = np.radians(45.)
    groundMap.maximumRange = 1e9

The user must then set several variables regarding the spacecraft instrument. These include the position of the camera
in the body-frame of the spacecraft, the normal vector of the instrument boresight defined in the body-frame of the
spacecraft, and the half field-of-view of the instrument. The position of the camera defaults to zero and is an optional
parameter. The normal vector of the instrument boresight, nHat_B, is also defaulted to zero, but is not an optional
parameter. Finally, the ``halfFieldOfView`` is a required parameter and defaults to 10 degrees if not set.

.. code-block:: python

    groundMap.cameraPos_B = [0, 0, 0]
    groundMap.nHat_B = [0, 0, 1]
    groundMap.halfFieldOfView = np.radians(22.5)

The mapping points should then be added to the module one at a time. This is done as follows:

.. code-block:: python

    groundMap.addPointToModel(map_point)

The ``maximumRange`` variable is optional and defaults to -1.  This means by default no maximum range is considered.  Set it to a positive value to have ``hasAccess`` output message variable depend on range.

A groundLocation can be affixed to a specific planet by setting its ``planetInMsg`` input message:

.. code-block:: python

    groundTarget.planetInMsg.subscribeTo(planetMsg)

The spacecraft can be added to the module by calling:

.. code-block:: python

    groundMap.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

Finally, logs for every mapping point can be created as follows:

.. code-block:: python

    mapLog = []
    for idx in range(0, N):
        mapLog.append(groundMap.accessOutMsgs[idx].recorder())
        scSim.AddModelToTask(simTaskName, mapLog[idx])
