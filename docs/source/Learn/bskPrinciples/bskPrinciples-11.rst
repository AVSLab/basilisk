.. _bskPrinciples-11:

Advanced: Effector Module Branching
===================================

Module branching in Basilisk enables the attachment of compatible dynamic effectors on state
effectors as an alternative to attaching them directly to the hub. For example, a gimbaled thruster
can be modeled as a ``thrusterDynamicEffector`` attached to a ``spinningBodyTwoDOFStateEffector``,
or a robotic docking arm can be modeled as a ``constraintDynamicEffector`` attached to a 7-DOF
revolute ``spinningBodyNDOFStateEffector``. Multiple dynamic effectors can be attached to a state
effector as well, for example applying both drag and SRP to a solar panel.

Further details on how to set up new state effectors and dynamic effector modules for branching can
be found in :ref:`effectorBranching`. Integrated testing of effector branching is elaborated on in
:ref:`test_effectorBranching_integrated`.

Allowable Configurations
------------------------
Branching pairs a state effector acting as a parent with a dynamic effector acting as a child. The
two sides are supported independently, so any parent listed below accepts any child listed below,
and a parent accepts more than one child at a time.

State Effectors Supported as Parents
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`hingedRigidBodyStateEffector`
- :ref:`dualHingedRigidBodyStateEffector`
- :ref:`nHingedRigidBodyStateEffector`
- :ref:`spinningBodyOneDOFStateEffector`
- :ref:`spinningBodyTwoDOFStateEffector`
- :ref:`spinningBodyNDOFStateEffector`
- :ref:`linearTranslationOneDOFStateEffector`
- :ref:`linearTranslationNDOFStateEffector`

A parent with more than one degree of freedom additionally takes the segment its child attaches to,
counting outward from the hub starting at 1.

:ref:`prescribedMotionStateEffector` hosts children as well, through the separate mechanism
described on its own page. Its state is prescribed rather than integrated, so a child attached to it
does not feed a load back through a joint.

Dynamic Effectors Supported as Children
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`extForceTorque`
- :ref:`thrusterDynamicEffector`
- :ref:`constraintDynamicEffector`
- :ref:`facetDragDynamicEffector`
- :ref:`facetSRPDynamicEffector`

A child reads its parent's inertial position, velocity, attitude, and angular velocity rather than
the hub's, and any geometry the user gives it is then expressed relative to the parent's frame
rather than the hub body frame.

Effectors Not Supported for Branching
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Every other effector attaches to the hub only.

A state effector cannot act as a child. ``addDynamicEffector`` accepts a dynamic effector, and a
state effector contributes Backsubstitution matrices rather than a force and a torque, so the two
do not interchange. This covers the spinning bodies, the translating bodies, the hinged rigid
bodies, :ref:`linearSpringMassDamper`, :ref:`sphericalPendulum`, :ref:`reactionWheelStateEffector`,
:ref:`vscmgStateEffector`, :ref:`thrusterStateEffector`, and :ref:`fuelTank`.

A load that does not depend on the parent's attitude is the same whether the effector attaches to a
segment or to the hub. This covers the cannonball models, namely :ref:`dragDynamicEffector` and the
cannonball option of :ref:`radiationPressure`.

:ref:`gravityEffector` is not a dynamic effector. It is a member of :ref:`spacecraft` whose field
reaches the equations of motion as the gravitational acceleration handed to every state effector, so
each branched body already carries its own weight. The vehicle level gravity gradient torque is
covered separately by :ref:`GravityGradientEffector`, which reads the spacecraft inertia tensor
about point B and therefore already tracks the configuration of every attached body.

:ref:`MtbEffector` and :ref:`ExtPulsedTorque` are branchable in principle and are listed here only
because no use case has called for them.

:ref:`thrusterStateEffector` and :ref:`fuelTank` are the two worth revisiting if a mission calls for
them, though either would first need the child side mechanism that a state effector does not have
today.

Setup
-----
Attaching a dynamic effector on a state effector is performed the same way as attaching a dynamic
effector to the hub, but instead called with the state effector. Given a setup as::

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraft.Spacecraft()

    stateEff = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    dynEff = extForceTorque.ExtForceTorque()

    scObject.addStateEffector(stateEff)

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, stateEff)
    scSim.AddModelToTask(simTaskName, dynEff)

By default, effectors are attached to the hub using::

    scObject.addDynamicEffector(dynEff)
    scSim.AddModelToTask(simTaskName, dynEff)

Dynamic effectors can instead be attached to a state effector as::

    stateEff.addDynamicEffector(dynEff)

Or in the case of a multi-degree-of-freedom parent effector, additionally specify which segment to
attach the dynamic effector to as::

    stateEff = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    segment = 2
    stateEff.addDynamicEffector(dynEff, segment)

Where segment is the integer number of the segment to be attached to, with 1 being the segment
attached to the hub, increasing outward from the hub. For example, spinningBodyOneDOFStateEffector
will always attach to segment 1, attaching to the second to last segment of a 7DOF robotic arm would
be segment 6. Note that in either case the dynamic effector is still added to the sim task.

However, some effectors use alternative methods for adding to a spacecraft. For example, the
attachment of a :ref:`thrusterDynamicEffector` set using the :ref:`simIncludeThruster`::

    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()
    thFactory.create('MOOG_Monarc_22_6', [0, 0, 0], [0, -1.5, 0])

    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

Would instead be attached using::

    thFactory.addToSpacecraftSubcomponent(thrModelTag, thrusterSet, stateEff, segment)

Additionally, the :ref:`constraintDynamicEffector` requires attachment to two parents. It works with
either a hub attached to the state effector of another vehicle, or the attachment of two different
state effectors to one another.::

    scObject2 = spacecraft.Spacecraft()
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()

    stateEff.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)
