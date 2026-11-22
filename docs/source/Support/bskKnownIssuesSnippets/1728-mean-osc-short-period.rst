- Basilisk 2.12.2 omits a short-period longitude term in ``clMeanOscMap``
  (issue `1728 <https://github.com/AVSLab/basilisk/issues/1728>`__). This biases
  the mapped argument of periapsis for eccentric orbits and can cause
  kilometer-scale position errors. The C and Python mappings, including
  checked and legacy paths, now include the term in both directions.
  Upgrade and recompute affected reference results, or apply the correction
  described in :ref:`meanOscShortPeriodCorrection` to an older implementation.
  Circular inputs are unaffected by this omission; the theory's first-order
  accuracy and critical-inclination limitations remain.
