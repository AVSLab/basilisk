
RKF78 integrator. It implements the method integrate() to advance one simulation time step, but can scale intermediate time steps according to the current relative error and the module's relative tolerance.

The module
:download:`PDF Description </../../src/simulation/dynamics/Integrators/_Documentation/Basilisk-Integrators20170724.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

The default ``absTol`` value is 1e-8, while the default ``relTol`` is 1e-4.









