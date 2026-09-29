``StateVecIntegrator`` provides shared workspace preparation and dynamics callbacks
for deterministic and stochastic integrators. A ``DynamicObject`` owns it and invokes
integration after model initialization.

Implementing an integrator
--------------------------

Derive from an existing numerical family when possible. A new family implements:

* ``prepareIntegrationBinding()`` to resolve finalized state topology and allocate
  workspaces before the first step or after a synchronized secondary is destroyed;
* ``validateIntegrationBinding()`` to check that cached descriptors remain valid;
* ``integrateImpl(currentTime, timeStep)`` to advance one requested interval.

The private ``integrate()`` entry point prepares storage when needed and reuses it
while the dynamics group remains unchanged. Model code calls ``DynamicObject::integrateState()``. Numerical methods
call ``evaluateDerivatives()`` and ``evaluateDiffusions()`` to evaluate the dynamics
objects in synchronization order.

Ownership
---------

The dynamics list is borrowed. Configure it before workspace preparation. Keep its objects
alive while using the integrator. The base does not track object destruction during
callbacks. Replacing an integrator destroys it and invalidates its borrowed pointers.

See :ref:`integratorArchitecture` for the complete call path.
