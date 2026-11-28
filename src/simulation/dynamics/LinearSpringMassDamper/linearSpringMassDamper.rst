
Executive Summary
-----------------

This state effector models a particle constrained to move along a linear spring and damper.
It can attach directly to a spacecraft hub or to a :ref:`prescribedMotionStateEffector`.

The module
:download:`PDF Description </../../src/simulation/dynamics/LinearSpringMassDamper/_Documentation/Basilisk-LINEARSPRINGMASSDAMPER-20180102.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Module Assumptions and Limitations
----------------------------------
The particle has one translational degree of freedom along the unit vector ``pHat_B``.
``rho`` measures displacement from the spring equilibrium point, and the spring and damping
forces are ``-k * rho`` and ``-c * rhoDot`` along that axis.

Prescribed-motion attachments support constant particle mass. Nested variable-mass effectors
are not supported: a particle whose mass is depleted through :ref:`fuelTank` must be attached
directly to :ref:`spacecraft`.


Message Connection Descriptions
-------------------------------
This state effector does not have any input or output messages.


Initialization and Reset
------------------------
``massInit`` must be finite and non-negative. Zero initial mass and depletion to zero during
integration remain supported. The initial-mass check runs before state registration during
spacecraft initialization, including when the effector is attached without being added to a
task. Invalid input raises ``BasiliskError``.

``Reset()`` repeats this configuration check without accessing parent states or changing
integrated displacement, velocity, or mass. In particular, it does not restore depleted mass
from ``massInit``. Initial values are applied during state registration.
See :ref:`effectorInitialization`.


User Guide
----------
For direct hub attachment, express ``pHat_B`` and ``r_PB_B`` in hub-frame components and call
``scObject.addStateEffector(particle)``. The PDF description above covers this configuration.

.. _linearSpringMassDamperPrescribed:

Attachment to a Prescribed Body
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The public variable names retain their direct-attachment spelling. For a prescribed-motion
attachment, their reference frame changes to the prescribed body:

- ``pHat_B`` is the spring's unit direction expressed in prescribed-body components.
- ``r_PB_B`` is the vector from the prescribed body's origin to the particle's equilibrium
  point, expressed in prescribed-body components.
- ``rhoInit`` and ``rhoDotInit`` are the initial displacement and displacement rate relative
  to that equilibrium point along ``pHat_B``.

Given an existing spacecraft ``scObject`` and simulation task ``unitTaskName``, the following
example creates a stationary prescribed parent and attaches an oscillating particle to it.
Add each effector to one parent only, before ``InitializeSimulation()``.

.. code-block:: python

    from Basilisk.simulation import linearSpringMassDamper, prescribedMotionStateEffector

    prescribed_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_body.ModelTag = "springPlatform"
    prescribed_body.setMass(100.0)  # [kg]
    prescribed_body.setIPntPc_P([[20.0, 0.0, 0.0],
                                [0.0, 25.0, 0.0],
                                [0.0, 0.0, 30.0]])  # [kg m^2]
    prescribed_body.setR_MB_B([0.5, 0.0, 0.0])  # [m] Mount offset in hub components
    prescribed_body.setSigma_MB([0.1, -0.2, 0.15])  # [-] Mount attitude as MRPs

    particle = linearSpringMassDamper.LinearSpringMassDamper()
    particle.ModelTag = "springParticle"
    particle.massInit = 20.0  # [kg]
    particle.k = 100.0  # [N/m]
    particle.c = 2.0  # [N s/m]
    particle.rhoInit = 0.5  # [m]
    particle.rhoDotInit = 0.0  # [m/s]
    particle.pHat_B = [0.0, 1.0, 0.0]  # [-] Direction in prescribed-body components
    particle.r_PB_B = [0.4, -0.2, 0.6]  # [m] Equilibrium offset from prescribed origin

    prescribed_body.addStateEffector(particle)
    scObject.addStateEffector(prescribed_body)
    unitTestSim.AddModelToTask(unitTaskName, prescribed_body)
    unitTestSim.AddModelToTask(unitTaskName, particle)

To move the parent, connect kinematic profiler messages to its ``prescribedTranslationInMsg``
and/or ``prescribedRotationInMsg`` as described in :ref:`prescribedMotionStateEffector`.
The particle's spring and damper motion remains dynamically coupled to the spacecraft while
the prescribed parent translates and rotates. The particle itself has no message ports.

After initialization, access the particle states through ``scObject.dynManager`` using
``particle.nameOfRhoState`` and ``particle.nameOfRhoDotState``. See
:ref:`test_linearSpringMassDamperPrescribed` for a displacement and velocity logging example.


Module Testing
--------------
:ref:`test_linearSpringMassDamperPrescribed` compares undamped and damped displacement,
velocity, and hub recoil against the analytical two-mass oscillator solution with a fixed
prescribed parent. It also checks independent Newton/Euler force and torque balances for
three oblique configurations with simultaneous prescribed translation and rotation.

The spring case in :ref:`test_PrescribedMotionStateEffectorBranching` exercises the kinematic
profiler modules, default and renamed hub states, and non-collinear attachment geometry.
It checks orbital energy and orbital and rotational angular momentum over the full simulation history and
checks convergence of the sampled-profiler momentum error when the timestep is halved.
Rotational energy is not required to be constant while the prescribed actuators do work.
