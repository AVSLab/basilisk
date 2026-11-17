- GitHub issue 1689: In tagged releases v2.6.0 through v2.12.2,
  :ref:`polyhedralGravityModel` returned positive gravitational potential,
  opposite to the specific potential-energy convention of the other gravity
  models. Spacecraft orbital-energy reporting also passed inertial positions
  directly to the body-fixed polyhedral potential calculation. Consequently,
  ``totOrbEnergy`` had the wrong potential sign and, for a rotated or rotating
  body, evaluated the potential at the wrong location. The fix returns negative
  specific potential energy and transforms relative positions into each body's
  frame at the energy evaluation epoch. On affected
  releases, compute the specific potential independently as the negative of
  ``poly.computePotentialEnergy()`` evaluated at the correct body-fixed position.
  Remove that extra negation after upgrading. Acceleration calculations retain
  their existing behavior. For a uniformly rotating non-axisymmetric body,
  check the Jacobi integral rather than expecting orbital energy alone to be
  conserved.
