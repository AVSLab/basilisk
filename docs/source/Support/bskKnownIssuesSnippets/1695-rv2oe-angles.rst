- In Basilisk ``2.12.2``, ``astroFunctions.RV2OE`` can return zero inclination for
  polar orbits, or zero/360-degree node, periapsis, or anomaly angles when a
  defining component or dot product is exactly zero (#1695). Roundoff in
  inverse-cosine arguments can also produce nonfinite angles. The function now
  tests the relevant vector magnitudes for undefined angles and clamps each
  inverse-cosine argument to ``[-1, 1]``. Its existing zero placeholders for
  singular angles are retained; these placeholders can discard orientation
  information for circular, equatorial, or radial states.
