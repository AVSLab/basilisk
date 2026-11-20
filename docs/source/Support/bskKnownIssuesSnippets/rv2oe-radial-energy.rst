- In Basilisk ``2.12.2``, ``astroFunctions.RV2OE`` can return a zero semimajor
  axis for bound or unbound radial motion because radial orbits have unit
  eccentricity at every energy. It now uses specific orbital energy to
  compute the semimajor axis. Radial motion in any direction returns unit
  eccentricity even when the cross product is nonzero from roundoff.
  Angular momentum and energy are evaluated with at least double precision,
  including for integer and mixed-precision inputs.
  Both radial and nonradial states at escape energy within roundoff return
  ``a = 0`` and ``e = 1``.
