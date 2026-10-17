.. _integratorArchitecture:

Dynamics state and integrator architecture
==========================================

Basilisk separates the definition of a dynamics model from the storage and arithmetic
used to integrate it. A model registers named states once, then reads their values and
writes derivatives through stable ``StateData`` handles. Integrators bind to the finalized
storage and reuse flat numerical buffers for every stage. BSM spacecraft and MuJoCo scenes
use this same integration layer, with different equations of motion and state-update rules.

This guide explains the boundaries between those components. For a minimal model
implementation, start with :ref:`creatingDynObject`; for MuJoCo model setup, see
:ref:`mujocoDynObject`.

Components and ownership
------------------------

.. list-table::
   :header-rows: 1
   :widths: 24 39 37

   * - Component
     - Responsibility
     - Owns or borrows
   * - ``DynamicObject``
     - Coordinates Reset, integration, and synchronized models.
     - Owns its ``DynParamManager`` and active integrator.
   * - ``DynParamManager``
     - Exposes registration, state lookup, and named properties to modules.
     - Owns properties and one native ``StateRegistry``.
   * - ``StateRegistry``
     - Validates declarations and allocates fixed contiguous state storage.
     - Owns state handles, update policies, layouts, and value buffers.
   * - ``StateData``
     - Provides shaped access to one state's values, drift, and diffusion.
     - Borrows storage from its registry; models borrow the handle.
   * - ``StateVecIntegrator``
     - Prepares workspaces once and calls the dynamics models.
     - Borrows the ordered list of synchronized dynamics objects.
   * - ``FlatStateBinding``
     - Translates registered states into validated offsets and update runs.
     - Borrows finalized buffers and policies; owns descriptors.
   * - Numerical integrator and workspace
     - Evaluates stages and combines their derivatives or noise tangents.
     - Owns preallocated stage, candidate, and rollback buffers.

The usual module-facing path is::

    DynamicObject
      dynManager.registerState(...) -> borrowed StateData*
      equationsOfMotion(...)        -> stateView(), derivativeView()
      integrateState(...)           -> owned StateVecIntegrator

The integrator-facing path is::

    StateVecIntegrator
      FlatStateBinding -> finalized StateRegistry layouts and buffers
      stage workspace  -> candidate -> live buffers -> dynamics callback
                       <- captured derivatives and diffusion tangents

To access contiguous segments or complete state layouts from C++, include
``stateRegistry.h`` and call ``DynParamManager::getStateRegistry()``.

Registration, values, and shapes
--------------------------------

A state declaration has a name and a ``StateSpec``. The specification records the
state matrix shape, derivative shape, diffusion-tangent shape, local noise count,
error-control mode, and update kind. The simple ``registerState(rows, cols, name)``
overload declares an ordinary Euclidean state with matching shapes.

Each registry stores three column-major arrays of doubles:

* state values, in registration order;
* derivatives, in registration order;
* diffusion tangents, in state order and then local noise-source order.

A ``StateLayout`` gives offsets into those arrays. Its offsets count scalars, not bytes.
An integrator concatenates objects in synchronized integration order; its descriptors
therefore also have offsets into the integrator's combined buffers. Registry-local and
integrator-global offsets are different coordinate systems and must not be interchanged.

For a Euclidean state, propagation uses ordinary addition. A special state instead supplies
an immutable ``StateUpdatePolicy`` that constructs drift candidates and applies diffusion
increments. Its derivative or tangent may have fewer components than its stored value:
a unit quaternion, for example, has four stored components but can use a three-component
angular-velocity derivative. The policy owns the meaning of that update; the integrator
owns the stage coefficients and ordering.

``StateData`` is a handle, not an owning matrix. ``stateView()``, ``derivativeView()``,
and ``diffusionView(i)`` return borrowed Eigen maps. Use them for local calculations and
reacquire them after the first finalization. Later resets preserve their addresses. The copying getters, such as
``getState()``, remain useful when a snapshot must survive a later update.
Setters require exact shapes and never resize established state storage.

Properties are separate named matrices used for model information that is not itself
integrated. They belong to ``DynParamManager`` and do not appear in integrator workspaces.

Registration and Reset
----------------------

A model's Reset declares and initializes states, finalizes storage, links handles, and
performs the remaining model setup::

    Reset
      register named states and initialize values
      finalizeStates(): allocate contiguous storage on the first call
      link handles and finish initialization

Before finalization, each state has temporary matrices because the total storage size is
not yet known. Finalization computes offsets, allocates the three buffers, copies initial
values, and redirects the existing handles into those buffers. The temporary matrices are
then released. The layout and buffer addresses remain fixed for the manager's lifetime.

Later resets look up states by name and write values directly into the same storage.
Declarations may occur in a different order, but each existing state's dimensions, update
policy, and noise count must match. Shared-noise groups may be repeated with their endpoints
in any order. New states or changed noise connections require a new manager. Repeated calls
to ``finalizeStates()`` have no effect, including for an empty finalized manager.

Reset is ordinary model initialization. Errors propagate to the caller and may leave
partially initialized values. There is no registration transaction, saved pre-Reset copy,
or automatic retry protocol. Complete model initialization successfully before integration.

Binding and lifetime
--------------------

An integrator prepares its descriptors and numerical workspaces on its first step.
Later steps reuse that storage. If a native secondary is destroyed, the surviving
primary rebuilds its workspace before advancing again. The registry records whether its buffers have been
allocated, and the integrator records whether its workspace is prepared. These flags
track one-time setup; they do not describe Reset success or recovery from model errors.

``StateBufferSegment`` identifies a registry, a buffer kind, and a range of scalars.
Resolve it through the owning registry before using the pointer. Matching resets preserve
the range and its address. The registry must outlive the segment and any pointer obtained
from it; segments do not track owner destruction or allocator address reuse.

The primary ``DynamicObject`` owns the integrator and calls the synchronized objects'
pre- and post-integration hooks. Configure synchronization before the first step. Do not destroy objects,
replace the integrator, or modify the integration group from a dynamics callback.
``setIntegrator()`` transfers the dynamics list and destroys the previous integrator.
Pointers to that previous integrator are no longer valid.

Python retains synchronized secondaries for the primary's lifetime. Native connections
are detached when either object is destroyed; a surviving secondary can integrate
independently again. Keep the dynamics object alive while using its borrowed manager or
integrator. Reacquire the integrator property after replacement.

Fixed and adaptive Runge--Kutta
-------------------------------

For ordinary states, a fixed-step explicit RK method evaluates

.. math::

   k_i &= f\left(t+c_i h,\;x_n+h\sum_{j<i}a_{ij}k_j\right),\\
   x_{n+1} &= x_n+h\sum_i b_i k_i.

``svIntegratorRungeKutta`` stores each derivative stage in one column of ``kStorage``.
It captures the step-entry state once, combines stage columns into candidates, and makes
each candidate visible in the live buffers before evaluating the dynamics. Adjacent
Euclidean states form contiguous update runs; special states dispatch through their policy.
An all-Euclidean single-object model can use its live buffer as the candidate buffer.

Adaptive RK reuses those stages to form lower- and higher-order candidates. It accepts the
higher-order candidate when the greatest error-to-tolerance ratio is at most one, otherwise
it retries with a smaller internal step. ``baseState`` holds the most recently accepted
internal step, while ``entryState`` preserves the start of the entire requested interval.
An exception restores the latter, even if earlier internal substeps succeeded, while propagating the error to the caller.

For whole-state error control, the ratio is

.. math::

   r_s =
   \frac{\lVert x_s^{\mathrm{high}}-x_s^{\mathrm{low}}\rVert}
        {a_s+r_s^{\mathrm{tol}}\lVert x_s^{\mathrm{high}}\rVert}.

For per-component control, the corresponding scalar ratios are evaluated separately and
their maximum is used. The norm above is the Euclidean/Frobenius norm of the stored state,
including for special-policy states; this is not a manifold-distance error metric.
Absolute tolerances have the units of the state, and relative tolerances are dimensionless.

Tolerance precedence is object-and-state override, then state-name override, then global
default, independently for relative and absolute tolerance. Resolved tolerances are cached
in descriptor order so an unchanged configuration needs no state-name lookup during error
evaluation. Direct writes to the legacy public default fields are also detected.
Nonfinite candidates and time steps that cannot make representable forward progress fail
explicitly.

Stochastic topology and stage storage
-------------------------------------

Stochastic models supply drift and diffusion for

.. math::

   dx = f(t,x)\,dt + \sum_q g_q(t,x)\,dW_q.

Choose an integrator whose stochastic interpretation and noise assumptions match the
model; the shared storage layer does not change those mathematical requirements.
Each state declares local noise sources. ``registerSharedNoiseSource()`` identifies local
endpoints driven by the same process, so they receive the same global Wiener increment.

There are three relevant orders:

* **Registration order** determines state and derivative buffer layout.
* **Canonical noise traversal** visits objects in integration order and states by name
  within each object to assign global noise slots consistently.
* **State-local noise order** determines how successive diffusion increments are applied
  to a state. Special updates need this order because the operations may not commute.

``StateVecStochasticIntegrator`` builds the mappings between those orders once.
A ``StochasticNoiseSlot`` identifies one global source; its ``StochasticNoiseBinding``
records identify the state-local tangents affected by that source. Diffusion-stage storage
is packed by global slot, with only the tangents belonging to that slot. It is not a dense
state-by-global-noise matrix.

``StochasticRKIntegratorBase`` owns the generator and preallocated Wiener/auxiliary output.
``FlatStochasticWorkspace`` owns method stage matrices, combined tangents, and scratch
vectors indexed by global source. Concrete methods supply coefficients and callback order.
Methods needing only Wiener increments do not request unused auxiliary samples. Consequently,
identical seeds need not reproduce trajectories from versions that consumed those samples.
Prescribed generators are available for reproducible recurrence tests.

Candidate assembly has a small internal protocol:

* a complete candidate can be accepted as the base for another update;
* sparse per-source trials share a drift baseline and restore the previous source's
  scratch changes; live buffers are also checked and restored if callbacks changed them;
* an all-Euclidean final candidate can accumulate ordered terms before one final scatter.

Those phases describe scratch-buffer validity, not model registration. They prevent a
method from accepting an incomplete candidate or mixing sparse and final assembly.
On an attempted-step failure, the numerical method restores its step-entry state values before propagating the error.
It does not rewind the noise generator, derivatives, messages, or other callback effects.

BSM and MuJoCo adapters
-----------------------

BSM spacecraft register hub and effector states during Reset. Effectors keep ``StateData``
handles and reacquire local views for their equations of motion. Device counts and state
dimensions become fixed with the first topology. Effectors with dimension-dependent
derivative scratch allocate it during setup and overwrite it during evaluation. Existing
post-integration operations, such as MRP shadow-set selection, remain model responsibilities.

``MJScene`` registers joint position states first and velocity states second, allowing
checked contiguous segments to transfer the corresponding ``qpos`` and ``qvel`` blocks.
Scalar joints use Euclidean states. Ball joints use quaternion position states; free joints
split translation and attitude into separate records. The native quaternion policy has a
four-component state and three-component drift, whereas the high-order policy uses a
four-component quaternion derivative. Both use three-component diffusion tangents.

Before evaluating MuJoCo dynamics, the scene copies the candidate states into MuJoCo data.
The evaluation updates drift or diffusion, which the integrator then captures. Scene
tasks and message publication keep their defined callback schedule; storage optimization
does not make intermediate-stage callbacks optional.

``MJSpec`` compiles a candidate model and checks its dimensions against the fixed state
layout before replacing model/data. Runtime values are copied into the replacement, and
body, joint, actuator, and equality wrappers update their cached IDs and addresses directly.
Wrapper initialization errors propagate without restoring previous bindings. Compile guards
keep model/data pointers stable while callbacks use them.

Additional continuous states use ``StatefulSysModel``. The scene passes a
``DynParamRegisterer`` during registration, which prefixes state names and supports shared
noise declarations. The helper is temporary, and the scene must outlive returned state
handles. Modules exchange ordinary model information through messages.

Python compatibility and borrowed objects
-----------------------------------------

The SWIG modules use canonical ``DynParamManager``, ``StateData``, ``DynamicObject``, and
integrator proxy classes. Import shims reuse those definitions across extension modules.
Compatibility names remain aliases to the
canonical class, preserving identity across import paths.

Python keeps the established ``registerState(nRow, nCol, stateName)`` signature, including
keyword arguments. Use ``registerStateSpec(stateName, spec)`` for a complete ``StateSpec``.
C++ uses the corresponding ``registerState`` overloads.

The ownership rules follow the objects that hold native storage:

* A standalone manager owns its states. States returned directly by its registration and
  lookup methods retain that manager's Python proxy.
* A manager embedded in a dynamics object is borrowed. Keep the dynamics object alive
  while using the manager or its states.
* States returned through a ``DynParamRegisterer`` borrow the scene's storage. Keep the
  scene alive while using those handles; the temporary registerer does not own them.
* MuJoCo body, joint, and site accessors retain their parent through a reference attached
  to the returned proxy. Local SWIG ``%pythonappend`` declarations provide this behavior
  without replacing methods at module import.
* ``setIntegrator()`` consumes a newly supplied integrator, including on rejection.
  Reinstalling the active integrator is a no-op. Borrowed integrators expire on replacement
  or owner destruction; Python rejects attempts to reinstall those expired proxies.

The bindings do not maintain a global owner registry. Native callers manage borrowed
lifetimes explicitly.

Registry internals and lifecycle mutation helpers are not exposed to Python.
The fixed-size sequence adapters allow supported indexed configuration updates without
exposing container-resize operations that would bypass topology checks.
