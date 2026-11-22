Executive Summary
-----------------
The ``NBodyGravity`` module computes gravitational forces on one or more targets due to one or more gravity sources
and applies those forces in a MuJoCo simulation. It supports arbitrary gravity models and
can be tied to MuJoCo sites or bodies for direct force application.

Message Interfaces
------------------
Messages are registered per source and per target via ``addGravitySource`` and ``addGravityTarget``.

.. bsk-module-io:: NBodyGravity

   input source.stateInMsg SpicePlanetStateMsgPayload
      Inertial position and velocity of the source, with optional planet-fixed orientation and its derivative.

   input target.massPropertiesInMsg SCMassPropsMsgPayload
      Mass properties of the gravity target. One per registered target.

   input target.centerOfMassStateInMsg SCStatesMsgPayload
      Inertial state (position, velocity, attitude) of the target's center of mass. One per registered target.

   output target.massFixedForceOutMsg ForceAtSiteMsgPayload
      Output gravitational force expressed in the site frame. One per registered target.

Module Description
------------------
For each gravity target, the module accumulates the gravitational acceleration from all sources:

.. math::

   \mathbf{a} = \sum_k \mathbf{a}_k(\mathbf{r})

where :math:`\mathbf{a}_k` is the acceleration from source :math:`k` evaluated at the target's position
:math:`\mathbf{r}` in the inertial frame. The resulting force is

.. math::

   \mathbf{F} = m\,\mathbf{a}

and is published as a ``ForceAtSiteMsgPayload`` in the site frame.

If one source is designated as the central body, target positions are relative to it. For each other source,
the module subtracts the acceleration that source exerts on the central body from the acceleration it exerts
on the target.

Multiple gravity sources and targets can be registered. Each source provides its own ``GravityModel`` (e.g.,
point-mass or spherical harmonics). A source reads its ephemeris from ``stateInMsg``. For a factory-created
source, connecting ``stateInMsg`` overrides the shared gravity body's ``planetBodyInMsg``.
Each target provides mass properties and center-of-mass state via ``massPropertiesInMsg`` and ``centerOfMassStateInMsg``.

Detailed Behavior
-----------------
At each dynamics evaluation, the module advances each source from its own message write time to the
integrator stage. Position uses the reported velocity. Orientation uses the angular velocity derived
from ``J20002Pfix`` and ``J20002Pfix_dot``. An omitted orientation defaults to inertial alignment.
The same source poses are used for every target and for the central-body acceleration. The module
then multiplies each target's acceleration by its mass and expresses the force in the site frame.

Stage times may precede the message epoch, including before simulation time zero with SOSRI.
For a 5 ns RK4 step, both middle stages evaluate gravity at 2.5 ns into the step and stamp
their force messages at 2 ns into the step.
When scheduled outside a scene dynamics task, the module evaluates at its scheduled tick.
In both cases the target state is read as supplied and should describe that evaluation epoch.
The untimed helpers ``computeAccelerationFromSource()`` and ``computeAccelerationOnTarget()``
evaluate the ephemeris messages as written.

During ``Reset()``, the module checks target connections and permits at most one central body.
A single source with no ephemeris is treated as stationary at the origin. Multiple sources each
require an ephemeris connection.

Module Assumptions and Limitations
----------------------------------
- The module computes forces at the target center of mass; torques due to gravity gradient are not modeled.
- Between ephemeris samples the source velocity and spin are constant. Choose the publisher rate
  to resolve changes in either quantity over the interval.
- Sources and targets are processed in alphabetical order by name.

Verification and Testing
------------------------
The module is verified in
``src/simulation/mujocoDynamics/NBodyGravity/_UnitTest/test_gravity.py``.
