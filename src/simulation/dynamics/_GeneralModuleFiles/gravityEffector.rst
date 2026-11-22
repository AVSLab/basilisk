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
        Central planet state. A connected ephemeris is forwarded after its first write and retains its source timestamp.

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

Potential Energy
----------------
All gravity-model ``computePotentialEnergy(position)`` methods accept a position
relative to the gravity body in its body-fixed frame and return potential energy
per unit spacecraft mass in :math:`\mathrm{m^2/s^2}`, with zero at infinity.
The point-mass limit is :math:`-\mu/r`. For the polyhedral model this is the
negative of the positive potential used in the Werner and Scheeres formulation.
The spherical-harmonic model currently returns only the point-mass potential;
its higher-degree acceleration terms are not included in the energy calculation.

``GravBodyData.computePotentialEnergyInertial(position, timeNanos)`` accepts an
inertial position relative to the body and performs the frame conversion at the
requested simulation time. Like acceleration evaluation, it uses the loaded
planet ephemeris and advances its orientation from the message timestamp.
An absent or all-zero orientation uses the same identity fallback as acceleration.

The spacecraft's reported ``totOrbEnergy`` includes mass times the specific
potential. Planet positions and orientations for this calculation are evaluated
at the output epoch, even when the integrator's last stage occurs earlier.
For an isolated, static polyhedral body, orbital energy is conserved up to
integration error. A rotating, non-axisymmetric body can exchange energy with
the orbit. For uniform rotation about a fixed axis through the body's center,
the conserved quantity is the Jacobi integral
:math:`E/m-\boldsymbol{\omega}\cdot(\mathbf{r}\times\mathbf{v})`, using position
and velocity relative to that center in an inertial frame.

Custom C++ gravity models must follow the body-fixed position and negative
potential-energy convention of ``GravityModel.computePotentialEnergy()``.
The C++ ``GravityEffector.updateEnergyContributions()`` method also requires the
evaluation time in nanoseconds so it does not depend on a cached integrator-stage
time. Python calls to individual gravity models retain their existing signatures,
including the ``positionWrtPlanet_N`` keyword on the ``GravityModel`` base-class
wrapper and the point-mass and spherical-harmonic potential methods. This
historical keyword still takes a body-fixed position despite its ``_N`` suffix.
The point-mass and spherical-harmonic potentials are radial, so their potential
energy is independent of the position frame's orientation.

Planet Orientation
------------------
The planet orientation is read from ``planetBodyInMsg`` together with its time derivative ``J20002Pfix_dot``. Between
message updates the orientation is advanced as a rotation about the planet angular velocity, which keeps the
orientation matrix orthonormal at every integrator stage. A first-order update of the matrix elements would scale the
evaluated field by a relative error of order :math:`(\omega\,\Delta t)^2`.
The advanced orientation and its time derivative at the same epoch are stored as the state-engine properties
``[planetName].J20002Pfix`` and ``[planetName].J20002Pfix_dot``, both in the :math:`[PN]` convention of the
planet state message.
When spacecraft share a ``GravBodyData``, each spacecraft updates the planet position, velocity,
gravitational parameter, orientation, and orientation derivative in its own state manager.

For spacecraft dynamics, gravity extrapolates planet position and orientation to the signed stage
time supplied by the integrator in double-precision seconds. SOSRI can evaluate a stage before time
zero. When the spacecraft accepts a state, the central body is extrapolated to the
accepted clock tick. Subtracting the integer output and message timestamps before conversion to
seconds preserves small offsets even years into a mission.
A synchronized spacecraft can publish between its primary's steps. It republishes its accepted
inertial state and timestamp even if another spacecraft has since refreshed a shared gravity body.
Its mass-state message also carries the accepted timestamp.
Schedule articulated effectors at their parent spacecraft module's rate. Their inertial outputs use
the spacecraft's accepted pose and timestamp, including when the hub follows a prescribed reference.
A rejected integration trial leaves the previously published inertial messages available.
A secondary subscribed to its primary's ``centralBodyOutMsg`` uses the sample forwarded at
the start of the same integration step, with the original source timestamp.

If a linked planet message is still unwritten during spacecraft reset, the spacecraft defers
its initial derivatives. On a zero-duration update, the spacecraft applies written attitude
and translation references and reevaluates the derivatives with the latest available ephemerides
once all linked planet messages are written. This includes replacements at the same timestamp.
Synchronized spacecraft apply their written motion references and refresh articulated endpoint positions,
attitudes, and velocities before evaluating initial constraint loads. The initial derivatives, accepted endpoints,
and all gravity evaluations during a zero-duration update use the exact integer tick, including at multi-year epochs.
At reset and during zero-duration updates, the spacecraft stores that tick directly in the double-precision
nanosecond component of ``systemTime``. Other stages report their signed stage time.
At each subsequent integrator stage, constraints use endpoint positions and velocities in a common inertial frame.
A constraint with an unknown central-body origin defers its loads until both endpoints can be located.
Until its reference is first written, each component of the motion retains
its configured or integrated state.
Write every linked ephemeris before advancing simulation time. While a linked central-body input
is unwritten, the spacecraft and its articulated bodies defer their inertial state messages.
Mass properties and relative joint states can already be read.
State-effector reaction forces and torques retain their initial zero values until the spacecraft
evaluates the derivatives with a complete gravity sample. After a rejected integration trial,
queries retain the last accepted reaction loads until the next successful update.
Hinge reaction forces and torques use the accepted hub angular velocity, including a written
attitude reference, and the hinge acceleration from the last dynamics evaluation.
A missing perturbing-body input alone does not prevent publication of the inertial state.

In six-degree-of-freedom mode, the spacecraft computes energy from the accepted state after
applying any prescribed attitude and translation. A direct call to ``spacecraft.computeEnergyMomentum(time)``
evaluates the gravitational potential of the current state at the supplied time in seconds.
At a zero-duration update with a pending ephemeris, ``totOrbEnergy`` and ``orbPotentialEnergyContr``
are ``NaN``. An advance rejected for a missing ephemeris leaves the stored diagnostics intact.
Direct energy queries raise ``BasiliskError`` if any required body has an unavailable sample.
If an integration error leaves effector data at a trial stage, direct spacecraft energy queries
also raise ``BasiliskError``. The last reported energy remains available until the next successful spacecraft update.
The public C++ method ``GravityEffector.updateEnergyContributions()`` also checks sample
availability before adding to the caller's accumulator.
Spacecraft sharing a ``GravBodyData`` see the same sample validity. After a missing ephemeris
is written, sampling it on either spacecraft makes that body's data available to both.
Direct energy queries in ``pointMassTranslationalOnly`` mode also raise ``BasiliskError``.
