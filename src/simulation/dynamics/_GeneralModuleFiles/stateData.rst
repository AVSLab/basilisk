
``StateData`` is a stable, registry-owned handle for one named continuous state.
It provides shaped access to the state value, its time derivative, and one diffusion
tangent per local noise source. It neither owns an independent matrix nor evaluates
the equations of motion. See :ref:`integratorArchitecture` for the surrounding system.

Using a handle
--------------

Keep the handle returned by ``DynParamManager::registerState()``. During a dynamics
callback, acquire views and write the derivative in place:

.. code-block:: cpp

    // A dimensionless scalar x obeys dx/dt = -decayRate * x.
    const double decayRate = 0.5; // [1/s]
    const auto x = this->xState->stateView();
    this->xState->derivativeView() = -decayRate * x;

``stateView()``, ``derivativeView()``, and ``diffusionView(index)`` are borrowed Eigen
maps with column-major storage. They must not outlive the owning manager and must be
reacquired after first finalization. Handles and finalized buffer addresses survive later
resets; their values update in place.
The copying getters ``getState()``, ``getStateDeriv()``, and ``getStateDiffusion()``
return owning snapshots. Setters require exact shape matches.

Shapes and special updates
--------------------------

``StateSpec`` declares state, derivative, and diffusion-tangent shapes separately.
They match for ordinary Euclidean states. A ``StateUpdatePolicy`` defines propagation
when the representation needs a special update, such as a quaternion driven by angular
velocity. Policies are immutable and owned by the registry, not by individual integrators.
Adaptive error control can measure the whole state or each scalar component.

Declare noise counts in ``StateSpec``. The deprecated ``setNumNoiseSources()`` remains
available for initial registration; it cannot change established topology.
