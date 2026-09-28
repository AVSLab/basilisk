
RKF45 integrator. It implements the method integrate() to advance one simulation time step, but can scale intermediate time steps according to the current relative error and the module's relative tolerance.

The module
:download:`PDF Description </../../src/simulation/dynamics/Integrators/_Documentation/Basilisk-Integrators20170724.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

The default ``absTol`` value is 1e-8, while the default ``relTol`` is 1e-4.

The integrator reuses state, derivative, and error buffers across internal trial
steps and calls to ``integrate()``. Rejected trials retain the last accepted
state and retry with the existing step-size controller. Error estimates retain
the configured global, state-specific, and object-specific tolerances, including
componentwise error control when requested by a state.

Buffers allocate on first use and may allocate again when registered states,
integrated dynamic objects, or state and derivative dimensions change between
calls. State registration and object membership must remain stable during an
``integrate()`` call. Custom state propagation and dynamics computations may
allocate independently of the integrator buffers.
