.. _orbitalMotionMigration:

Orbital Motion Compatibility
============================

The issue 1629 audit compares the orbital utilities, their three SWIG export
paths, module callers, and tests with the branch's starting revision
``3193231686``. Numerical defect corrections remain enabled. Public types,
representations, signatures, and previously accepted domain behavior are
preserved as described below.

The existing methods and conventions remain supported without deprecation
warnings or a removal schedule. Numerical improvements are available through
the original method names. Optional checked calls and alternate encodings are
additions; existing callers do not need to adopt them.

Preserved behavior
------------------

* Python ``rv2elem`` retains NumPy ``float64`` fields ``a``, ``e``, ``rmag``,
  ``alpha``, ``rPeriap``, and ``rApoap``. Expressions such as
  ``axis_history - elements.a`` continue to work when the history is a list.
  ``M2E`` and ``N2H`` retain NumPy scalar results when either argument is a
  NumPy scalar, including their zero-anomaly shortcuts. Zero-dimensional mean
  arrays retain their container type without changing the caller's input.
* Python's default ``rv2elem`` retains the planar parabolic representation
  ``a = -rPeriap, rApoap = -1``. Optionally use
  ``rv2elem(mu, r, v, canonical=True)`` for the C-compatible
  ``a = rApoap = 0, rPeriap > 0`` representation accepted by ``elem2rv``.
  Legacy output can still be passed to ``elem2rv_parab``.
  That adapter always interprets ``e = 1, a < 0`` as a planar parabola with
  periapsis ``-a``, even when auxiliary fields are omitted, unset, or zero.
  Python and SWIG element inputs remain unchanged on repeated calls.
* ``rv2elem_parab`` retains signed node, periapsis, and nonradial true-anomaly
  angles, its circular convention with periapsis at the current position,
  and ``a = rApoap = -rPeriap`` for planar parabolas. The main C ``rv2elem``
  retains its nonradial true-anomaly interval [0, 2*pi); the main Python
  function retains its signed hyperbolic anomaly interval.
* The Python ``*_parab`` adapters retain inward motion for positive
  ``sin(E)`` on rectilinear ellipses. When explicitly converting those input
  elements for use with ``elem2rv``, negate the radial eccentric anomaly. The canonical
  core uses outward-positive radial E. Rectilinear hyperbolas use the core
  ``elem2rv`` with outward-positive H; they are distinct from the adapter's
  original negative-a planar parabolic encoding. Eccentricity one alone never selects a parabola: the angular
  momentum, energy, and documented auxiliary fields distinguish the cases.
* ``KeplerianOrbit`` retains its native member layout, including reserved
  legacy storage. Its public signatures and units are unchanged.

Planet selection
----------------

Ordinary C/C++ calls with three arguments, ``jPerturb(r, degree, acceleration)``,
now select Earth safely through the header adapter. Four-argument calls still compile
and run, and honor the selected planet. All three SWIG bindings retain both
call forms. Native source calls also accept C compound literals and C++ braced
initializers as vector arguments. Checked planet selection is available through:

.. code-block:: c

    jPerturbForPlanet(r, degree, acceleration, CELESTIAL_MARS);

The legacy four-argument native path retains Earth fallback for unknown body
identifiers and leaves the output unchanged for Sun/Pluto. The corresponding
Python utility retains its unknown-name Earth fallback, ``None`` for Sun/Pluto,
and floating degree cutoffs. Python callers can opt into checked
selection using ``orbitalMotion.jPerturb(r, degree, planet, strict=True)``.
Checked paths reject unknown bodies and return explicit zero vectors for
Sun/Pluto's zero-zonal approximation. Saturn's previously incorrect Earth
coefficients are corrected in all paths.

The historical variadic native symbol remains available for existing
**explicit-planet** function pointers and binary callers. Such pointers require
one trailing integer planet identifier. A raw three-argument call to that
variadic symbol already had undefined behavior because it read an absent
argument; it cannot be made safe by preserving that binary calling convention.
Use ``jPerturbEarth`` for a fixed three-argument pointer. The header dispatch
makes ordinary three-argument source calls safe when rebuilt.

Checked domains
---------------

C and Python now provide the additive entry points ``hillFrameChecked``,
``hill2rvChecked``, ``rv2hillChecked``, ``clMeanOscMapChecked``, and
``clElem2eqElemChecked``. Supported inputs receive the improved calculations
through both old and checked names. Checked calls report domain failures as
NaN outputs with diagnostics in C, or ``ValueError`` in Python. The old names
retain historical rejected-domain formulas, sentinel outputs, and exceptions.
Neither form is deprecated; checked calls are optional.

Inclination validation is an intentional exception to this compatibility path:
all element-input APIs require finite inclination in [0, pi], including both
endpoints. Negative and wrapped inclinations are rejected immediately.
The constellation example's documented input remains radians; its main call
and test now convert 56 degrees explicitly. Optical-navigation test fixtures
likewise convert 10 degrees. Monte Carlo configurations use nonnegative radian
inclination bounds; this intentionally changes their former invalid samples. A valid
inclination can still coincide with a particular coordinate singularity:
prograde equinoctial elements remain singular at the retrograde equator.

``meanOEFeedback`` explicitly uses the checked conversions and stops rather
than publishing an invalid force after conversion failure. ``oeStateEphem``
uses the fitted conic without snapping near-unit eccentricity to a parabola;
its positive-periapsis records cannot encode rectilinear states or parabolic
mean anomaly. These module messages and configuration field names are unchanged.

Corrections retained without compatibility fallbacks
----------------------------------------------------

The audit retains corrections to unconverged Kepler roots, singular orbit
geometry, near-parabolic and rectilinear Cartesian reconstruction, equatorial
and whole-revolution mean/osculating mapping, zonal coefficients, zero-speed
drag, characteristic energy, and eclipse disk overlap. Matching an incorrect
numerical answer is not a supported compatibility mode. ``KeplerianOrbit.c3``
now returns ``-mu/a`` rather than its erroneous constant zero.

Explicit failure replaces undefined or unsupported calculations: nonphysical
conics, unrepresentable states, nonfinite Kepler inputs, nonpositive mass or
gravity parameters, and unsupported ``KeplerianOrbit`` conics. In
particular, the drag helper no longer reads outside a three-vector on invalid
position input. Callers supplying invalid inputs can observe different error
types or NaN diagnostics; they must not rely on the previous accidental
IndexError, uninitialized data, or unconverged output.

The removed AVS self-check names were test-harness selectors that executed no
native test; the four real self-check entry points remain. Eclipse corrections
and debris plotting compatibility preserve their callable interfaces. Forward-anomaly precision
near the parabolic limit remains outside this change.

Regression coverage
-------------------

Compatibility tests exercise NumPy scalar methods and original debris list
arithmetic; default and canonical parabolic encodings; radial adapter velocity
signs; input-storage invariance for Python and all SWIG element types; large
wrapped anomalies against direct trigonometric references; signed and circular
angle conventions; native/SWIG optional planet
calls; legacy Sun/Pluto and unknown-body sentinels; fractional degree cutoffs;
checked versus legacy domain failures; output aliasing; and accepted/rejected
inclination boundaries. Compatibility tests reject deprecation warnings for
the preserved APIs. Existing numerical, controller, ephemeris, and scenario
tests remain part of validation. The optional Vizard-driven OpNav Monte Carlo
example requires an external renderer and is not exercised by the normal suite;
its inclination bounds were updated by inspection.
