
RKF78 integrator. It implements the method integrate() to advance one simulation time step, but can scale intermediate time steps according to the current relative error and the module's relative tolerance.

The module
:download:`PDF Description </../../src/simulation/dynamics/Integrators/_Documentation/Basilisk-Integrators20170724.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

The default ``absTol`` value is 1e-8, while the default ``relTol`` is 1e-4.









Step Size and Tolerances
------------------------

The integrator advances the state over one task period at each task update. It starts each
task period with a trial step equal to the whole period, and it can shorten the step to meet
``relTol`` and ``absTol`` but never takes a step longer than the task period, because other
modules read the spacecraft state at the task rate. The task period is therefore the largest
step, and the tolerances govern the accuracy only below it. The tolerances apply to each
state in its own units, for example the position in m and the velocity in m/s.

At the default ``relTol`` of 1e-4 the error estimate can accept a full task period as a
single step even where the dynamics change rapidly, such as through the periapsis of a highly
eccentric orbit. The resulting error is not reported.

For example, for a two-body orbit with an eccentricity of 0.95 and a periapsis radius of
6800 km, propagated for 2.71 periods with Basilisk 2.12.2 and compared with the solution of
Kepler's equation:

.. list-table:: Position error after 2.71 periods, eccentricity 0.95
   :header-rows: 1

   * - Task period
     - ``relTol``, ``absTol``
     - Position error
   * - 998 s (500 per orbit)
     - 1e-4, 1e-8 (default)
     - 2006 km
   * - 998 s
     - 1e-8, 1e-8
     - 974 m
   * - 998 s
     - 1e-12, 1e-6
     - 0.53 m
   * - 99.8 s (5000 per orbit)
     - 1e-4, 1e-8 (default)
     - 3.1 m
   * - 99.8 s
     - 1e-12, 1e-6
     - 1.2 m

With the longer task period the default tolerance accepts full-length steps through
periapsis, while with the shorter one the steps are already short enough that the tolerance
has little effect.

For orbits of high eccentricity, either tighten ``relTol`` or shorten the task period, and
check the accuracy against a reference before relying on the default settings.
