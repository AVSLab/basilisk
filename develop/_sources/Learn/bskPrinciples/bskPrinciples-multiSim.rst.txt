.. _bskPrinciples-multiSim:

Advanced: Naming Effector States
================================

.. sidebar:: Source Code

    Download the runnable :download:`bsk-multiSim.py <../../codeSamples/bsk-multiSim.py>`
    example. It prints state names from four short simulations without opening
    any plot windows.

A state effector adds integrated quantities to a spacecraft, such as a hinged
panel's angle and angular rate, a reaction wheel's speed, or a fuel tank's mass.
The spacecraft's :ref:`dynParamManager` stores these quantities as named states.
Effectors supply default names, so you usually do not need to choose them yourself.
The available name attributes and default naming conventions depend on the module.

.. note::

    **Read automatic names from the effector. Assign custom names when their
    exact text must repeat across simulation builds.** Do not hard-code a
    generated numeric suffix or assume that each simulation starts its counter
    at one.

This tutorial uses :ref:`hingedRigidBodyStateEffector` to illustrate
constructor-based automatic naming and optional custom names.

Which Names Identify States?
----------------------------

For a hinged panel, these attributes identify two different integrated states:

.. list-table:: Hinged-panel state names
   :header-rows: 1
   :widths: 45 55

   * - Attribute
     - Quantity stored in the dynamics manager
   * - ``panel.nameOfThetaState``
     - Panel angle, in radians
   * - ``panel.nameOfThetaDotState``
     - Panel angular rate, in radians per second

The Python variable ``panel`` refers to the effector object. Its ``ModelTag``
labels the module for diagnostics. Neither renaming the Python variable nor
setting ``panel.ModelTag = "leftPanel"`` changes its state names.

After initializing the simulation, retrieve a state through its actual name:

.. code-block:: python

    sim.InitializeSimulation()
    angle_state = vehicle.dynManager.getStateObject(panel.nameOfThetaState)
    angle = angle_state.getState()[0][0]  # [rad]

This avoids assuming a name such as ``"hingedRigidBodyTheta1"``. The state
storage does not exist until registration during initialization, even when the
name string is already available.

Automatic Names and Object Lifetimes
------------------------------------

With constructor-based naming, the hinged-body module increments a static
counter each time a panel is constructed. All instances of that module in the
same Python process share the counter, including panels belonging to different
spacecraft or different simulations. The counter records constructions, not
the number of panels currently alive.

For example, assuming no earlier panel constructions, two panels receive
``hingedRigidBodyTheta1`` and ``hingedRigidBodyTheta2``. The next panel receives
``hingedRigidBodyTheta3``, even if the first two panels have been destroyed.
Constructing a panel and then assigning custom names still advances this counter.

.. important::

    **Destroying an effector does not reset its naming counter.** The built-in
    effectors that use constructor counters consistently leave them unchanged
    during destruction. New instances continue from the next counter value,
    even after previous instances have been destroyed.

``del panel`` removes that Python reference; it does not necessarily destroy
the underlying effector immediately. A simulation or another object can still
hold a reference. Forcing ``gc.collect()`` is not a naming strategy: it does not
reset the counter or remove references that are still in use.

Creating New Simulations in One Python Process
----------------------------------------------

Repeated builds mean creating new spacecraft and new effectors on each pass
through a loop. For example, using the builder in the downloadable script:

.. code-block:: python

    for case in range(2):
        sim = build_simulation(custom_names=False)
        sim.InitializeSimulation()
        sim.ExecuteSimulation()
        del sim

The builder configures the stop time and physical parameters for each run.
Although these are new simulations, Python and the loaded Basilisk library
remain alive. Creating a new ``SimBaseClass`` does not reset static counters.
An illustrative sequence is:

.. list-table:: Two panels constructed per simulation
   :header-rows: 1
   :widths: 20 40 40

   * - Build
     - Left panel's angle-state name
     - Right panel's angle-state name
   * - First
     - ``hingedRigidBodyTheta1``
     - ``hingedRigidBodyTheta2``
   * - Second
     - ``hingedRigidBodyTheta3``
     - ``hingedRigidBodyTheta4``

The starting suffix can be larger in a notebook or test session that has already
created panels. This does not make the states incorrect: each simulation can
look them up through its own panels' name attributes. A lookup that assumes
``"hingedRigidBodyTheta1"`` can fail in the second simulation.

This pattern occurs in parameter sweeps, optimization loops, tests, notebook
sessions, and Monte Carlo workers that execute several cases in one Python
process. Finishing a BSK run does not necessarily terminate its Python process.
A fresh Python interpreter starts with fresh library state; a forked worker
can inherit its parent's counter state. Also, a Basilisk ``Process`` created
with ``CreateNewProcess()`` is a scheduling container, not a new Python process.

Resetting or continuing the same simulation is a separate operation. Keeping
the same effector objects does not rerun their constructors or assign new
constructor-based names. Restoring initial states, random seeds, and recorder
contents is a separate concern from naming.

Choosing Repeatable Custom Names
--------------------------------

Set the names inside the builder, before attaching the effectors and before
initialization. Give each component a distinct, meaningful prefix:

.. code-block:: python

    left = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    left.nameOfThetaState = "leftPanelAngle"
    left.nameOfThetaDotState = "leftPanelRate"

    right = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    right.nameOfThetaState = "rightPanelAngle"
    right.nameOfThetaDotState = "rightPanelRate"

Every call to the builder can use these same four strings. The user is
responsible for keeping custom state names unique among all states in a shared
dynamics manager, including the spacecraft hub and other effector types.
Giving two panels the same angle-state name does not create two independent
states. Independent spacecraft with separate managers can reuse the same names.

Assign every name whose exact text matters to your application. Setting only
``nameOfThetaState`` leaves the angular-rate name automatic. Effectors may also
publish named properties, such as inertial position and attitude; these have
their own names and a separate property namespace. Renaming a state does not
rename its properties. Consult the module documentation for its setters and
attributes, and set custom property names before attaching dependent effectors.
Treat names as configuration: do not change them after initialization or after
another module has copied them for a connection.

Recording Data and Connecting Modules
-------------------------------------

Prefer an output-message recorder when the effector already publishes the
quantity you need. For example, the panel's output contains both angle and rate:

.. code-block:: python

    recorder = panel.hingedRigidBodyOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", recorder)

The recorder can be configured before initialization and is independent of the
state-name strings. For an internal state without a suitable message, retrieve
its name from the effector rather than hard-coding a generated suffix.

With the hinged-panel setup above, state names are available during construction
and remain unchanged during initialization. A connection configured during
setup can therefore copy the name from its producing panel. If you use a custom
name, assign it before making that connection.

Running the Example
-------------------

From the repository root, with Basilisk available in your Python environment:

.. code-block:: console

    python docs/source/codeSamples/bsk-multiSim.py

The script creates two spring-loaded panels in each spacecraft and executes
four independent simulations in one interpreter: two with automatic state names,
then two with custom state names. It prints the angle and rate names for each
panel, looks up the final states using those names, and reads the same quantities
from message recorders. Only strings and numerical results are retained between
runs.

The automatic-name rows have different suffixes between builds. The custom-name
rows repeat ``leftPanelAngle``, ``leftPanelRate``, ``rightPanelAngle``, and
``rightPanelRate``. The physical configuration is identical in all four runs;
the example's test checks that the final states agree with the message data and
with the other runs.

The builder and repeated-build loop are shown below. No explicit garbage
collection or counter manipulation is needed.

.. literalinclude:: ../../codeSamples/bsk-multiSim.py
   :language: python
   :pyobject: build_simulation

.. literalinclude:: ../../codeSamples/bsk-multiSim.py
   :language: python
   :pyobject: run_cases
