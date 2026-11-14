
RKF78 integrator. It implements the method integrate() to advance one simulation time step, but can scale intermediate time steps according to the current relative error and the module's relative tolerance.

The module
:download:`PDF Description </../../src/simulation/dynamics/Integrators/_Documentation/Basilisk-Integrators20170724.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

The default ``absTol`` value is 1e-8, while the default ``relTol`` is 1e-4.









Step Size and Tolerances
------------------------

The task rate sets the largest integration step. The integrator may shorten a step to meet
``relTol`` and ``absTol``, but it never takes a step longer than the task period. The
tolerances apply to each state in its own units, for example the position in m and the
velocity in m/s.

.. warning::

    The default ``relTol`` and ``absTol`` values may not be suitable for all orbit regimes.
    The user must ensure that appropriate relative and absolute tolerances are set for the
    simulation.
