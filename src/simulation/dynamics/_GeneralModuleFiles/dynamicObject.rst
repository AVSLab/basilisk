``DynamicObject`` is the base for models whose continuous states are integrated,
including spacecraft and MuJoCo scenes. It owns a ``DynParamManager`` and an integrator,
coordinates synchronized objects, and provides drift, diffusion, pre-integration, and
post-integration extension hooks.

Subclass responsibilities
-------------------------

``Reset()`` declares and initializes states, calls ``finalizeStates()``, and finishes
model setup. Later resets reuse those states and update values directly. An initialization
exception may leave partial values; complete setup successfully before integrating.

``UpdateState()`` normally delegates propagation to ``integrateState()``. The subclass
defines ``equationsOfMotion()`` and, for stochastic models, ``equationsOfMotionDiffusion()``.
These callbacks read candidate states and write derivatives or noise tangents. They may
run several times within one requested interval.

``preIntegration()`` and ``postIntegration()`` perform model-specific work around the
integrator call. Configure topology and synchronization before the first integration step.
Callbacks must not change ownership, destroy participating objects, or replace the integrator.

Synchronization and ownership
-----------------------------

``syncDynamicsIntegration()`` adds another object to the primary object's integration
group before binding. The primary owns the group's integrator. ``setIntegrator()`` transfers
the dynamics list and destroys the old numerical method; borrowed pointers to it expire.
Both ``setIntegrator(method)`` and Python ``integrator = method`` transfer ownership of a
new method, including when it is rejected. Reinstalling the active method is a no-op.
Keep the dynamics object alive while using its borrowed manager or integrator.

Python retains synchronized secondaries. Native connections are detached when either
object is destroyed. A surviving primary rebuilds its workspace before the next step;
a surviving secondary resumes independent integration. Repeating an existing connection
is a no-op.

See :ref:`creatingDynObject` for a Reset example and :ref:`integratorArchitecture`
for the storage and workspace design.
