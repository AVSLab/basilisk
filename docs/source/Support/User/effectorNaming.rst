.. _effectorNaming:

Effector Naming Policies
========================

A state effector adds a component with its own integrated quantities to a
:ref:`spacecraft`. Examples include a hinged panel's angle, a reaction wheel's
speed, and a fuel tank's remaining mass. Basilisk stores these quantities as
named **states** in the spacecraft's dynamics manager, ``vehicle.dynManager``.
Effectors can also provide named **properties**, such as a panel's inertial
position and attitude, for other modules to use.

These names identify data inside the dynamics manager. They are separate from
the Python variable holding an effector, its ``ModelTag`` diagnostic label, and
its message attributes. For example, ``panel.nameOfThetaState`` is the name of
the panel's angle state; assigning ``panel.ModelTag = "leftPanel"`` does not
rename that state.

.. note::

   **Use manager-local naming for new Spacecraft scripts.** It is the recommended
   naming policy and the direction of future Basilisk development. Opting in now
   makes automatic names independent of other simulations and earlier object
   creation in the same Python session. Existing scripts should also migrate
   after updating any lookups that depend on generated names, as described below.

   Legacy automatic naming is deprecated and is scheduled for removal on **September 14,
   2027**. Legacy naming remains the default during the transition.
   Deprecated ``SpacecraftSystem`` and unsupported external effectors require
   migration before they can use the new policy; see the compatibility section.

Enabling Manager-Local Naming
-----------------------------

Set the flag immediately after creating each spacecraft, before calling
``InitializeSimulation()``:

.. code-block:: python

    from Basilisk.simulation import spacecraft

    vehicle = spacecraft.Spacecraft()
    vehicle.dynManager.useManagerLocalEffectorNames = True

Here, *manager-local* means that each dynamics manager allocates its own names.
Two independent spacecraft can safely use the same generated names. Within one
manager, automatic naming keeps effector states and properties distinct.

Set the flag to ``False``, or omit it, to retain legacy naming. Changing policies
after preparation begins raises ``BasiliskError``; assigning the selected value
again is allowed. Code that directly collects names or registers states or
properties must select the policy before those operations.

The code blocks below assume an existing ``simulation`` with a task named
``"dynamicsTask"`` and ``vehicle`` scheduled on that task. Configure the effector's
physical parameters as described in its module documentation. For a complete
runnable example, start with
:ref:`scenarioEffectorNaming`, which prints names before and after initialization
under both policies.

Optional Custom Names
---------------------

Automatic names are provided without user assignments. Override only the fields
your script needs, before initialization:

.. code-block:: python

    from Basilisk.simulation import hingedRigidBodyStateEffector

    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel.nameOfThetaState = "leftPanelAngle"  # Optional; omit for an automatic name.
    # Configure the panel's physical parameters before initializing the simulation.
    vehicle.addStateEffector(panel)
    simulation.AddModelToTask("dynamicsTask", panel)

``addStateEffector()`` includes the panel in the spacecraft's dynamics.
``AddModelToTask()`` schedules the panel's message handling. The angular-rate state
and inertial properties remain automatic even when the angle state has a custom
name. Python attribute syntax is the same under both policies.

In manager-local mode, custom names are used exactly as supplied and reserved
before automatic names are assigned. Empty names and conflicting custom names
raise an error. Assigning a string that happens to match a constructor-generated
name still counts as a custom override.

When Names Become Final
-----------------------

Use this setup sequence:

#. Create and configure the spacecraft and all its effectors. Attach any nested
   components as well, such as bodies attached to a prescribed moving frame.
#. Configure message connections, recorders, and logging callbacks.
#. Call ``InitializeSimulation()``. After it succeeds, read generated names from
   their owning effectors and run the simulation.

**Manager-local automatic names read before initialization are provisional.**
They may change when the spacecraft assigns names to its complete set of
effectors. For example, after configuring all models and loggers:

.. code-block:: python

    simulation.InitializeSimulation()
    angleState = vehicle.dynManager.getStateObject(panel.nameOfThetaState)

Names are fixed once resolved. To change them or the attachment configuration,
create a new simulation with new spacecraft and effector objects. Repeating
initialization with the same configuration preserves names and state storage,
but modules still perform their usual state initialization; it is not a way to
resume from the current simulated state. ``Reset()`` does not restart name
allocation.

Logging and Connections Before Initialization
---------------------------------------------

Prefer a module's output message recorder when it provides the required data.
It can be configured before initialization and does not depend on state names:

.. code-block:: python

    recorder = panel.hingedRigidBodyOutMsg.recorder()
    simulation.AddModelToTask("dynamicsTask", recorder)

For internal states without the required message output, read both the name and
the state when the logging callback runs. This angle example demonstrates the
technique, although the message recorder above already provides that angle:

.. code-block:: python

    from Basilisk.utilities import pythonVariableLogger

    def readPanelAngle(_currentTimeNanos):
        # The logger calls this during simulation, after names are finalized.
        stateName = panel.nameOfThetaState
        angleState = vehicle.dynManager.getStateObject(stateName)
        return angleState.getState()[0][0]  # [rad] Scalar state stored as a 1-by-1 matrix.

    stateLog = pythonVariableLogger.PythonVariableLogger({"angle": readPanelAngle})
    simulation.AddModelToTask("dynamicsTask", stateLog)

The logger supplies the current simulation time in nanoseconds; this callback
does not need that argument. Copying ``panel.nameOfThetaState`` into a string
before initialization and capturing that string in the callback can leave it
pointing to a provisional name. Hard-coding a generated suffix has the same
problem. When creating callbacks in a loop, give each callback its own effector
reference, for example ``lambda _, panel=panel: ...``.

When a separate module requires a state-name string during its own initialization,
give the producer an explicit custom name before connecting it. For example,
:ref:`scenarioStochasticDragSpacecraft` uses:

.. code-block:: python

    stochasticAtmo.setStateName("atmosphericDensityCorrection")
    drag.densityCorrectionStateName = stochasticAtmo.getStateName()

This explicit connection remains stable under both naming policies. Built-in
dynamic effectors attached to supported panels or bodies have their property
connections refreshed automatically during initialization.

Migrating an Existing Script
----------------------------

#. Find hard-coded generated names and names copied before initialization.
   Replace those lookups with message recorders or reads of the effector's
   current name at the time of use.
#. For connections that need a name string before initialization, assign an
   explicit custom name to the producing effector first.
#. Enable ``useManagerLocalEffectorNames`` on each spacecraft and run the script's
   tests. For otherwise identical, valid configurations, changing the naming
   policy should leave the physical results unchanged.

Legacy Automatic-Naming Warning
--------------------------------

When a built-in effector registers an automatically named state or property in
legacy mode, its Python interface reports a dated Basilisk deprecation warning:

* Before **September 14, 2027**: ``BSKDeprecationWarning``.
* On or after that date: ``BSKUrgentDeprecationWarning``.

The warning identifies the opt-in flag, links to these migration instructions,
and gives the removal date. Reaching the date escalates the warning; it does not
disable legacy naming or change the selected policy. Removing legacy support
requires a future Basilisk release.

Each manager reports this warning only once, even with multiple effectors,
nested attachments, or repeated initialization. A new spacecraft has a new
manager and can report its own warning. These are Python warnings, independent
of the ``BSKLogger`` verbosity setting. Python warning filters may suppress
repeated messages. Basilisk's ``deprecated.ignore()`` helper can suppress the
ordinary warning, but does not suppress the urgent category; see
:ref:`deprecatingCode`.

Setting ``vehicle.dynManager.useManagerLocalEffectorNames = True`` suppresses the
warning and selects the new naming policy. Explicit names alone do not trigger
it, but a partial override can still leave automatic names. For example,
assigning a panel's angle and angular-rate names leaves its four inertial-property
names automatic. Unused names do not count: a balanced reaction-wheel array does
not register its wheel-angle state and therefore does not warn about that name.
You do not need to assign every name to avoid the warning; opting in retains
the convenience of automatic naming.

How Automatic Names Are Allocated
---------------------------------

.. list-table:: Naming behavior
   :header-rows: 1
   :widths: 25 35 40

   * - Behavior
     - Legacy, the current default
     - Manager-local, recommended
   * - Automatic allocation
     - Existing constructor counters or fixed module defaults.
     - An index allocated within the owning dynamics manager.
   * - Allocation order
     - Object creation order for modules with counters.
     - Attachment order, visiting each parent's children before the next sibling.
   * - Other simulations and garbage collection
     - Can affect constructor counters and generated names.
     - Do not affect another manager's automatic names.
   * - Custom names
     - Retain each module's existing behavior.
     - Reserved before automatic candidates and used exactly as supplied.
   * - Collisions
     - Retain existing duplicate-registration behavior.
     - Automatic groups skip occupied candidates; duplicate custom names raise an error.
   * - When names are final
     - Usually construction; ``SpacecraftSystem`` adds owner prefixes during initialization.
     - After successful ``InitializeSimulation()``.

Each effector declares a group of state and property names. Its automatic entries
share an index, starting at one for each naming family, such as hinged rigid
bodies. For two panels with no conflicting names, the angle states are
``hingedRigidBodyTheta1`` and ``hingedRigidBodyTheta2`` in attachment order.
Customizing the first panel's angle name leaves its other automatic names in
group 1, so the second panel still uses group 2.

A collision in any entry advances the whole group to an available index. Checks
include other effector types and fixed hub and gravity names. States and
properties have separate namespaces: the same text may name one state and one
property. Do not assume that automatic suffixes will always be consecutive;
read each final name from its effector.

Compatibility and C++ Effectors
-------------------------------

All built-in spacecraft state effectors participate, including reaction-wheel
and VSCMG arrays. Their legacy defaults remain ``reactionWheelOmegas``,
``reactionWheelThetas``, and the corresponding ``VSCMG...`` names. Manager-local
allocation adds an array index, allowing multiple arrays without custom names.
Existing physical restrictions on which effectors can attach to a prescribed
frame or another body still apply.

Deprecated ``SpacecraftSystem`` and ``SpacecraftUnit`` remain legacy-only and
reject the opt-in policy. Migrate these simulations to ``Spacecraft`` to use
manager-local names. External C++ state effectors must implement the naming
protocol described in :ref:`stateEffector`. An unsupported effector raises a
diagnostic identifying its ``ModelTag`` when available and recommending migration
or legacy mode during the transition. External effectors using generic
string-only registration do not report the automatic-naming warning unless
they adopt the legacy registration helpers described there.

As with Basilisk's other SWIG deprecations, the dated warnings are reported
through Python. Direct C++ use records automatic-name use without emitting a
Python warning. Normal ``InitializeSimulation()`` reports pending warnings after
the C++ initialization workers finish. For scheduled managers, this also defers
warnings triggered by Python callbacks until initialization has completed.
If an initialization callback initializes another simulation, that simulation's
warnings also wait until the outer initialization or task reset completes.
Independent simulations initialized concurrently report to their own callers.
``ResetTask()`` and a task's ``resetTask()`` report pending warnings after the
task finishes resetting. Direct Python calls to effector registration,
``initializeDynamics()``, and spacecraft ``Reset()`` also report them.

In C++, select the policy with
``vehicle.dynManager.setUseManagerLocalEffectorNames(true)`` or the
``EffectorNamingPolicy`` interface. Callers of migrated public name fields use
their corresponding getters and setters, such as ``getNameOfThetaState()`` and
``setNameOfThetaState()``.

During initialization, the spacecraft collects name declarations, registers fixed
hub and gravity data, resolves names, registers all effector states and properties,
and then connects dependent effectors to that data. The preparation protocol in
:ref:`stateEffector` explains how to participate when developing an external
effector.

Further Examples
----------------

:ref:`scenarioEffectorNaming` demonstrates both policies, a custom override,
before-and-after name tables, message recording, and a deferred state logger.
:ref:`scenarioFuelSlosh`, :ref:`scenarioConstrainedDynamicsComponentAnalysis`,
:ref:`scenarioStochasticDragSpacecraft`, and :ref:`scenarioCompareVariableMass`
also accept ``useManagerLocalEffectorNames=True``. Their default arguments retain
legacy behavior for compatibility.
