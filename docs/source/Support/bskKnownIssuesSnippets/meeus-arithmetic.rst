- In Basilisk ``2.12.2``, ``astroFunctions.ephemeridesMeeus`` evaluates the
  quadratic time coefficients for Saturn, Uranus, and Neptune as linear terms.
  Uranus and Neptune also use angular elements referred to the equinox of date,
  which introduces a frame mismatch with the other planets away from J2000.0.
  Both now use the mean ecliptic and equinox of J2000.0. The quadratic terms
  are corrected, and ``computeCOE`` now uses the fractional coefficients of
  the fifth-order equation of center in computing true anomaly.
