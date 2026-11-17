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

.. bsk-module-io:: gravityEffector
    :caption: GravityEffector Module I/O Messages

    output centralBodyOutMsg SpicePlanetStateMsgPayload
        Central planet body state output message.

The gravity effector contains a list of ``GravBodyData`` objects which contain the planet mass and size properties etc.
The following table lists the Spice planet ephemeris input message that can be connected to a ``GravBodyData`` object.
If no message is connected, then the planet has zero position and orientation information by default.

.. bsk-module-io:: GravBodyData
    :caption: GravBodyData Message Interfaces

    input planetBodyInMsg SpicePlanetStateMsgPayload
        Planet spice ephemeris input message.



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

Spherical Harmonics Reference Radius and Gravitational Parameter
----------------------------------------------------------------
``useSphericalHarmonicsGravityModel(file, maxDeg)`` loads the reference radius and the gravitational parameter of the
first row of the coefficient file into ``spherHarm.radEquator`` and ``spherHarm.muBody``. When the body is
initialized:

- The field keeps the reference radius of the file, since the normalized coefficients are only valid with the
  reference radius of their model. ``radEquator`` of the ``GravBodyData`` is used only if ``spherHarm.radEquator``
  is not positive, for example when the coefficients were set directly instead of being loaded from a file. To
  evaluate the field with a different reference radius, set ``spherHarm.radEquator``.
- The gravitational parameter of the field is replaced by ``mu`` of the ``GravBodyData``, so that the field and
  gravity-gradient torque use the same value. Body initialization raises ``BasiliskError`` if ``mu`` is not
  finite and positive, even when the file supplies a valid GM. To use the file GM, explicitly set
  ``planet.mu = planet.spherHarm.muBody`` after loading the file and before initializing the simulation.
  No warning is given when a valid body GM differs from the gravitational parameter of the file.

Spherical Harmonics Coefficient Tables and Initialization
---------------------------------------------------------
When setting ``spherHarm.cBar``, ``spherHarm.sBar``, and ``spherHarm.maxDeg``
directly, both coefficient tables must contain at least ``maxDeg + 1`` rows.
Row ``l`` holds orders zero through ``l`` and must contain at least ``l + 1``
entries. This includes degree zero, which requires one row with at least one
entry in each table. Additional columns and rows above ``maxDeg`` are ignored,
so padded tables and truncation of a higher-degree model are supported.

Both initialization overloads discard the previous recurrence tables on each
attempt. In Python, direct ``initializeParameters()`` calls return ``None`` on
success or an error string identifying an incomplete coefficient table.
Initialization through ``GravBodyData`` raises ``BasiliskError`` on failure.
After correcting a rejected configuration, initialize successfully before
evaluating the field. For example:

.. code-block:: python

    from Basilisk.simulation import sphericalHarmonicsGravityModel

    model = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
    model.muBody = 3.986004415e14  # [m^3/s^2]
    model.radEquator = 6378136.3  # [m]
    model.maxDeg = 2
    model.cBar = [[1.0], [0.0, 0.0], [-4.84e-4, 0.0, 2.44e-6]]  # [-]
    model.sBar = [[0.0], [0.0, 0.0], [0.0, 0.0, -1.40e-6]]  # [-]
    error = model.initializeParameters()
    if error is not None:
        raise ValueError(error)
    position = [7.0e6, 1.0e6, 2.0e6]  # [m] Body-fixed position
    acceleration = model.computeField(position)  # [m/s^2]

``computeField()`` raises ``BasiliskError`` if initialization has not succeeded,
the requested degree exceeds either the current ``maxDeg`` or the last
successfully initialized degree, or a required coefficient row or entry is
missing. These checks also apply after public attributes are changed.
Reinitialize before evaluating degrees above the initialized range. Changing
coefficient values within that range does not require reinitialization, provided
the required table dimensions remain valid. The explicit-degree overload checks
only the coefficient rows used by that evaluation, allowing lower-degree
evaluation even when unused higher-degree rows are incomplete.

Planet Orientation
------------------
The planet orientation is read from ``planetBodyInMsg`` together with its time derivative ``J20002Pfix_dot``. Between
message updates the orientation is advanced as a rotation about the planet angular velocity, which keeps the
orientation matrix orthonormal at every integrator stage. A first-order update of the matrix elements would scale the
evaluated field by a relative error of order :math:`(\omega\,\Delta t)^2`.
The advanced orientation and its time derivative at the same epoch are stored as the state-engine properties
``[planetName].J20002Pfix`` and ``[planetName].J20002Pfix_dot``, both in the :math:`[PN]` convention of the
planet state message.
