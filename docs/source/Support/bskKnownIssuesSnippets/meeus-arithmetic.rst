- In Basilisk ``2.12.2``, ``astroFunctions.ephemeridesMeeus`` evaluates the
  quadratic time coefficients for Saturn, Uranus, and Neptune as linear terms.
  Separately, ``computeCOE`` uses floor division for fractional equation-of-center
  coefficients, which can bias true anomalies and the resulting planetary states.
  Both arithmetic errors are corrected. The existing polynomial ephemeris model
  and fifth-order eccentricity approximation are retained, as are the return
  interfaces; computed elements and states can differ from previous releases.
