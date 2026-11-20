- In Basilisk 2.12.x, ``readAtmTable`` skipped the surface row in the nominal Venus, Uranus,
  Titan, and Jupiter GRAM CSV files. :ref:`tabularAtmosphere` then returned zero density and
  temperature below the first retained row. It also returned zeros exactly at the highest
  altitude of any table. The fix retains the surface row and returns the final row at its
  altitude (issue #1698). On affected releases, load all numeric rows directly and use the
  final row's values when querying the exact upper endpoint.
