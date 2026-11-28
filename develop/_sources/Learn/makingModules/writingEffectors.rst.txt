.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

.. _writingEffectors:

Writing Dynamic and State Effectors
===================================

Effectors extend the equations of motion of a parent dynamics module that uses the Backsubstitution Method
(BSM), such as :ref:`spacecraft`. The parent registers and links their states and properties, evaluates
their contributions, and integrates the coupled system. This guide introduces the C++ interfaces and
lifecycle requirements for writing effectors for the spacecraft BSM dynamics engine.

The automation script described in :ref:`makingDraftModule` generates ordinary scheduled modules;
it does not generate dynamic or state effectors. Start from the appropriate abstract effector interface
and consult an existing implementation with similar physics. Concrete effectors declared ``final``
are examples to study, not base classes to extend.

Choosing an Effector Type
-------------------------

A :ref:`dynamicEffector` supplies external forces and torques without registering its own integrated
states. Examples include :ref:`extForceTorque` and :ref:`GravityGradientEffector`. Derive the new
effector from ``DynamicEffector`` and implement its force and torque evaluation and state-linking methods.

A :ref:`stateEffector` participates in the coupled dynamics through its own states and contributions
to mass properties and the BSM equations of motion. Examples include :ref:`hingedRigidBodyStateEffector`
and :ref:`reactionWheelStateEffector`. Derive the new effector from ``StateEffector`` and implement
state registration, state linking, and derivatives, together with the other contributions required
by its physical model. Consult :ref:`spacecraft` and its linked mathematical description when deriving
the Backsubstitution terms.

.. note::

    Effectors that also provide scheduled behavior inherit from ``SysModel``. This supplies the familiar
    ``Reset()`` and ``UpdateState()`` interface for processing commands, updating discrete logic, and
    publishing messages. The effector and scheduler interfaces have separate responsibilities.

Attachment and Task Scheduling
------------------------------

Attach a dynamic effector with the spacecraft's ``addDynamicEffector()`` method or a state effector
with ``addStateEffector()``. Schedule the spacecraft so that it initializes and integrates the system.
Attachment does not automatically add the effector to a simulation task.

During spacecraft initialization, the parent calls each state effector's ``registerStates()`` and
then links the state and dynamic effectors through ``linkInStates()``. During integration, the parent
calls their dynamics methods as needed by the integrator. These calls occur even when the effector
has not been separately added to a task.

Use ``AddModelToTask()`` for an effector when its ``UpdateState()`` behavior is needed. For example,
an effector that reads commands in ``UpdateState()`` needs task scheduling to receive new commands.
Document the required execution order relative to command producers and the spacecraft. The scheduler
calls the effector's ``Reset()`` only when it has been added to a task.

Message publication can also use a parent dynamics hook: ``Spacecraft`` calls state effectors'
``writeOutputStateMessages()`` after initialization and after integration. State clearly which messages
use that hook and which require the effector's scheduled ``UpdateState()``.

.. _effectorInitialization:

Initialization and Configuration Validation
-------------------------------------------

Configuration work required by the equations of motion must not rely exclusively on ``Reset()``.
An effector that is only attached to a spacecraft still participates in the dynamics and must be
fully initialized and validated before those dynamics are evaluated.

Place shared configuration validation, normalization, and initialization of derived dynamics parameters
in a dedicated helper method. Call that helper from ``Reset()`` when the effector provides that method,
and from each applicable attachment path:

.. list-table:: Configuration initialization hooks
   :header-rows: 1
   :widths: 45 55

   * - Effector attachment
     - Hook that calls the shared helper
   * - State effector attached to a spacecraft
     - ``registerStates()``, before configuration-dependent state registration and initialization
   * - Dynamic effector attached directly to a spacecraft
     - ``linkInStates()``
   * - Dynamic effector attached to a supporting state effector
     - ``linkInProperties()``

The helper must be safe to call more than once because an effector can be both attached and scheduled.
Derive values from the configured inputs without accumulating changes on successive calls. Keep state
registration in ``registerStates()`` and retrieval of parent state or property references in the linking
methods. The shared helper should not assume that parent references or integrated states already exist,
because the scheduler can call the effector's ``Reset()`` before the spacecraft initializes them.

Validate dimensions, finite values, physical bounds, and other assumptions required by the model before
using them in its equations. Report invalid configurations through :ref:`bskLogging` with ``bskLogger.bskError()``
and an explanation identifying the offending parameter. A check that requires a linked parent reference
belongs in the corresponding linking method, after that reference is obtained.

Keep operations that belong only to the scheduled lifecycle, such as clearing command buffers or
resetting discrete filter history, in ``Reset()`` when they are not required by the dynamics.
Initialize any buffers used by the dynamics to safe defaults even if the effector is never scheduled.
Document what repeated resets change, especially for commands, filters, and integrated initial conditions.

Implementing a Dynamic Effector
-------------------------------

Implement ``linkInStates()`` to obtain the parent states and properties required by the model from
:ref:`dynParamManager`. Use the state and property names supplied by the attachment interface rather
than assuming a particular spacecraft name. A model that needs no parent states still implements this
required method and performs any necessary configuration initialization there.

Implement ``computeForceTorque()`` to evaluate the loads at the supplied integration time, using the
current integration-stage states. For direct spacecraft attachment, the output members have these meanings:

.. list-table:: Dynamic effector load contributions
   :header-rows: 1
   :widths: 35 65

   * - Member
     - Meaning
   * - ``forceExternal_N``
     - Applied force expressed in the inertial frame, in newtons
   * - ``forceExternal_B``
     - Applied force expressed in the spacecraft body frame, in newtons
   * - ``torqueExternalPntB_B``
     - Applied torque about body point B, expressed in the body frame, in newton meters

The spacecraft adds both force members after expressing them in a common frame. Do not place the
same physical force in both members. Include the moment of an offset applied force in the torque
about the specified reference point. Assign or clear the load contributions on each evaluation so
that loads from a previous call do not remain when the model becomes inactive.

Keep continuous load evaluation in the dynamics callback rather than computing it only once per
scheduled ``UpdateState()``. Integrators may evaluate intermediate states multiple times within one
task interval. Use the integration time in seconds supplied to dynamics callbacks; scheduler callbacks
use integer nanoseconds. Avoid advancing discrete command or filter state simply because an integrator
requests another evaluation.

Attachment to a state effector is an additional capability. Implement its property-linking path and
parent-frame conventions before enabling ``isAttachableToStateEffector``. See :ref:`effectorBranching`
for the required changes to both the child dynamic effector and its parent state effector.

Implementing a State Effector
-----------------------------

Register each integrated state through ``DynParamManager`` in ``registerStates()``, retain its
``StateData`` reference, and set its configured initial value. Ensure state names remain distinct
when multiple instances are attached. Use ``linkInStates()`` to obtain other states and properties
required for coupling after the parent has registered the system's states.

Implement the following contributions as required by the model:

.. list-table:: State effector dynamics methods
   :header-rows: 1
   :widths: 38 62

   * - Method
     - Responsibility
   * - ``updateEffectorMassProps()``
     - Evaluate ``effProps`` mass, center of mass, inertia, and their applicable rates from the current state.
   * - ``updateContributions()``
     - Supply the ``BackSubMatrices`` terms that couple the effector dynamics to the parent equations of motion.
   * - ``computeDerivatives()``
     - Set the registered state derivatives using the current state and the solved parent accelerations.
   * - ``updateEnergyMomContributions()``
     - Supply the effector energy and angular momentum contributions used in system diagnostics.
   * - ``writeOutputStateMessages()``
     - Publish state outputs when called by the parent after initialization and integration.

Use the base-class declarations for the exact signatures and required overrides. Optional hooks with
default implementations still need an override when the physical model contributes those quantities.
In particular, omitting a mass, coupling, or energy contribution merely because the code compiles can
produce an incomplete model.

Evaluate mass properties and coupling terms from the current integration-stage states, and clear
accumulated contributions before recomputing them. Set derivatives through the registered state objects;
the parent integrator advances the coupled states. Use ``Eigen::MRPd`` for C++ MRP attitude values,
with raw vector storage only at interfaces that require it, such as state containers and message payloads.

For variable-mass models, follow the retained-property and dynamics-rate contracts in :ref:`stateEffector`.
For a state effector that hosts child effectors, also follow :ref:`effectorBranching` for property
registration, frame conventions, and propagation of child loads.

Documentation and Testing
-------------------------

Provide the C++ source and self-contained header, a SWIG interface, a module RST page, and unit tests.
Use :ref:`cppModuleTemplate` for the documentation structure and common module packaging conventions.
Document physical units, reference frames and points, initial conditions, valid parameter ranges,
attachment support, and any task scheduling requirements. Describe messages with the
``bsk-module-io`` directive, identifying optional inputs and their defaults.

Test the effector through a spacecraft as well as any useful isolated calculations. The tests should cover:

* Initialization with the effector attached but omitted from ``AddModelToTask()``. Check that invalid
  configurations fail and that valid configurations initialize the required derived parameters and states.
* Initialization with the effector both attached and scheduled. Verify that repeated shared initialization
  does not accumulate changes or depend on the effector's ``Reset()`` running first.
* Physical behavior against an analytical result or an independent reference. For conservative systems,
  check the appropriate energy and momentum balances; account for external work or dissipation when present.
* Multiple instances, limiting configurations, and disabled or zero-load behavior, as applicable to the model.
* Command processing, optional inputs, output messages, and reset behavior when scheduled operation is supported.
* Attachment to a state effector, including initialization and transformed loads, when branching is supported.

Follow :ref:`bskModuleCheckoutList` for the remaining coding, documentation, build, and test requirements.
