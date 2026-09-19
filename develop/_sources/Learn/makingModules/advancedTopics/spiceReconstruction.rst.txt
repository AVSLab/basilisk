.. _spiceReconstruction:

Optimizing performance when using SPICE
=======================================

By default, :ref:`spiceInterface` evaluates SPICE (``spkezr_c`` for a body's position,
``sxform_c`` for its orientation) at the *exact* simulation time on every ``UpdateState``
call. When SPICE has become a performance bottleneck, the module can instead **reconstruct** each
body's state from a coarse grid of cached SPICE samples ("knots"), trading a controllable amount
of accuracy for far fewer SPICE evaluations. This page describes when that helps and how to
configure it.

When SPICE is (and is not) a bottleneck
---------------------------------------
How often SPICE is queried depends entirely on how the module is scheduled:

* **Regular task, e.g. a classic** :ref:`spacecraft` **with**
  :ref:`gravityEffector`. ``spiceInterface`` runs once per task period and writes each planet
  message a single time per step. ``gravityEffector`` then reads that one message and performs
  its *own* internal linear extrapolation of the planet to each integrator sub-step (position via
  ``PositionVector + VelocityVector*dt`` and orientation via ``J20002Pfix + J20002Pfix_dot*dt``).

* **Dynamics task (e.g.** :ref:`MJScene` **).** A model added with ``AddModelToDynamicsTask`` has
  its ``UpdateState`` invoked once per *integrator stage*. A fixed RK4 step calls it four times;
  an adaptive integrator such as RKF45 calls it more. Every one of those calls issues a fresh
  ``spkezr_c``/``sxform_c`` per body.

Reconstruction breaks the tie between "how often the state is *requested*" and "how often SPICE
is *called*". It lays down a grid of sample points ("knots") spaced ``knotStep`` nanoseconds
apart in absolute time: knot 0 at the epoch, knot 1 at ``knotStep``, knot 2 at ``2*knotStep``, and so
on. SPICE is called only to fill a knot, and only the first time that knot is needed; the result
is cached. When the state is requested at some arbitrary time (say a mid-step stage time), the
module finds the knots surrounding that time, calls SPICE for any not already cached, and
computes the requested value by interpolating or extrapolating between them. It does **not**
call SPICE at the requested time itself.

The consequence is that the cost no longer depends on the integrator. Many different requests
fall between the same pair of knots: the four RK4 stages within one step, the extra stages of
an adaptive method, and the re-tried stages of a rejected step all land in the same knot interval
and reuse the same cached knots. So the number of SPICE calls is set purely by how many knots the
run spans, ``~ horizon / knotStep`` per channel per body, regardless of how many times per step
the integrator asks for the state.

The three per-channel states
----------------------------
Each planet has two independent channels, **position** and **orientation**, and each channel
is in one of three states:

#. **Exact** (the default): query SPICE at the exact time on every call.
#. **Reconstruct**: build the value from the cached knot grid.
#. **Disabled**: make no SPICE query and emit the trivial default (zero position and velocity,
   or the identity orientation and zero rate). Use this to remove a computation that is not needed
   (e.g. a body pinned at the frame origin needs no position; a body used purely as a point mass
   needs no orientation).

In all three states every field of the planet-state message is still populated: the numeric
arrays as described above, and ``J2000Current`` (message validity time), ``computeOrient``, and
``PlanetName`` exactly as in the exact path. Orientation reconstruction is additionally gated on
``computeOrient``, so a body with no orientation frame keeps the identity ``J20002Pfix``.

The reconstruction is controlled by four knobs, set per planet and per channel:

* ``knotStep``: spacing of the absolute-time knot grid, in nanoseconds;
* ``nKnots``: stencil size, i.e. the polynomial order;
* ``useVelocity``: also match the SPICE-provided velocity/rate at each knot (Hermite) rather
  than fitting positions only (Lagrange);
* ``interpolate``: bracket the current time with past *and* future knots (interpolation) or use
  only past knots (causal extrapolation).

Configuring it
--------------
Create the ``SpiceInterface`` the usual way, through :ref:`simIncludeGravBody`: the same factory
that creates the gravity bodies also builds and connects the ``SpiceInterface`` (and, on an
:ref:`MJScene`, attaches the gravity model with ``addBodiesTo``). The reconstruction methods are
then called on the returned object, keyed by planet name (case-insensitive)::

    from Basilisk.utilities import macros, simIncludeGravBody

    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createEarth().isCentralBody = True
    gravFactory.createMoon()
    gravFactory.createSun()
    spice = gravFactory.createSpiceInterface(time="2025 NOVEMBER 15 12:00:00.000")

    # Position: cubic Hermite interpolation on a 240 s grid. knotStep is in nanoseconds,
    # so use macros.sec2nano to convert (like task periods elsewhere in Basilisk).
    spice.setPlanetPositionReconstruction("earth", knotStep=macros.sec2nano(240.0))   # nKnots=2, useVelocity=True, interpolate=True

    # Orientation of a constant-rate spinner: extrapolate from a SINGLE knot (see below).
    spice.setPlanetOrientationReconstruction("moon", knotStep=macros.sec2nano(1.0e5), nKnots=1,
                                             useVelocity=True, interpolate=False)

    # Turn a channel off entirely (no SPICE query):
    spice.setPlanetPositionDisabled("sun")

    # Revert a planet to the exact per-call SPICE query:
    spice.setPlanetExactSpice("earth")

    # Measure the payoff: {positionQueries, orientationQueries} over the run.
    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")

When a channel is enabled, its defaults are the efficient ones: cubic Hermite interpolation
(``nKnots=2, useVelocity=True, interpolate=True``) on a 60 s grid. So
``setPlanetPositionReconstruction(name)`` with no further arguments is usually a good starting
point.

Choosing knobs by body type
----------------------------
* **Bodies with a constant rotation vector** (a fixed spin axis and constant rate). This covers
  nearly every catalogued small body and the planetary IAU frames, whose orientation model is a
  linear ``W = W0 + Wdot*t``. SPICE returns the attitude as a DCM, whose trajectory is *not* linear
  in time; the reconstruction extrapolates the **principal rotation vector** (PRV) instead, which
  *is* linear in time for a fixed axis at a constant rate. So first-order extrapolation from a
  **single knot** (``nKnots=1, useVelocity=True, interpolate=False``) is exact at any ``knotStep``:
  one ``sxform_c`` call serves the whole run. Do not pay for interpolation or extra knots here.
* **Smooth heliocentric position** (planets, the Moon relative to Earth). The trajectory is very
  smooth on the scale of a propagation, so cubic Hermite interpolation (``nKnots=2,
  useVelocity=True, interpolate=True``) on a coarse ``knotStep`` reconstructs it to well below the
  integrator's own error.
* **Tumblers** (non-principal-axis rotators, e.g. proximity operations at objects like Apophis).
  The orientation has genuine curvature, so linear extrapolation degrades as the knot spacing
  grows; use interpolation (``interpolate=True``) and, if needed, ``nKnots >= 2``.

Tuning: match the reconstruction error to the integrator error
--------------------------------------------------------------
The guiding principle is that **the reconstruction error should be the same order of magnitude as
the integrator's own truncation error.** Once the reconstruction error drops
below the integrator error it no longer affects the trajectory, so any finer knot grid is wasted
computation.

A practical workflow:

#. Fix the shape at the efficient default (``nKnots=2, useVelocity=True, interpolate=True``), or
   the single-knot extrapolation above for a constant rotator. If SPICE querying is no longer
   a performance bottleneck, stop here. Otherwise:
#. Treat ``knotStep`` as the primary lever. A reasonable starting value is **8-16x the task
   step**. Then *increase* ``knotStep`` until the final-state error just begins to rise above the
   exact-SPICE baseline (that rise is reconstruction error emerging from beneath the integrator
   floor), and back off one step. Equivalently, if ``knotStep`` is fixed, there is no benefit to
   shrinking it once the error curve has flattened onto the integrator floor.
#. Use ``getPlanetSpiceQueryCount`` to confirm the query count fell to roughly ``horizon /
   knotStep`` and quantify the saving.

Align ``knotStep`` with the task step when possible
---------------------------------------------------
The reconstruction is a *piecewise* polynomial: its knot stencil switches as time crosses each
knot, so the reconstructed state has a mild discontinuity in its curvature (a "C1 kink") at knot
boundaries. If such a kink falls *inside* an integrator step it can locally reduce the
integrator's order and, for adaptive steppers, provoke unnecessary step-size cuts. The
reconstructed *value* is correct for any ``knotStep``; this is purely an integrator-efficiency
concern.

For a **fixed-step** integrator you can neutralize it by choosing ``knotStep`` as an integer
multiple of the task step, ``knotStep = m * dt``. Every knot boundary then coincides with a step
boundary (where the integrator restarts its expansion anyway) instead of landing mid-step. For
the orientation channel, whose base knot switches at the half-interval, an **even** multiple aligns
that boundary too. For adaptive integrators exact alignment is not possible, but keeping
``knotStep`` large relative to the average step keeps kinks rare. This is the concrete reason the
``knotStep >> task step`` guidance above is a soundness recommendation, not merely a tuning tip.

Worked examples
---------------
Two example scenarios put this page into practice, one per channel:

* :ref:`scenarioSpiceReconstruction` reconstructs the **position** of the Moon during a cislunar
  lunar-flyby transfer, sweeping ``knotStep`` to trace the accuracy-vs-query-count tradeoff and
  showing the "free" region below the integrator error.
* :ref:`scenarioVestaOrientation` reconstructs the **orientation** of the constant-rate spinner
  (4) Vesta, showing that a single-knot velocity extrapolation reproduces the attitude exactly at
  a fraction of the SPICE cost.
