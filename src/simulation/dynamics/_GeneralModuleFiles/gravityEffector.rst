Executive Summary
-----------------

Abstract class that is used to implement an effector impacting a GRAVITY body
that does not itself maintain a state or represent a changing component of
the body (for example: gravity, thrusters, solar radiation pressure, etc.)


The module
:download:`PDF Description </../../src/simulation/dynamics/gravityEffector/_Documentation/Basilisk-GravityEffector-20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.




Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - centralBodyOutMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - central planet body state output message

The gravity effector contains a list of ``GravBodyData`` objects which contain the planet mass and size properties etc.
The following table lists the Spice planet ephemeris input message that can be connected to a ``GravBodyData`` object.
If no message is connected, then the planet has zero position and orientation information by default.  

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetBodyInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - planet spice ephemerisis input message



User Guide
----------
The user must provide a list of ``GravBodyData`` objects to the spacecraft using::

    scObject.gravField.gravBodies = spacecraft.GravBodyVector(gravBodyList)

Each gravity body data object can be created using::

        earth = gravityEffector.GravBodyData()
        earth.planetName = 'earth_planet_data'
        earth.mu = 0.3986004415E+15  # meters^3/s^2
        earth.radEquator = 6378136.6  # meters
        earth.isCentralBody = False

Note that the ``simIncludeGradBody.py`` helper file contains a gravity body factor class to facilitate
setting up gravity bodies.
