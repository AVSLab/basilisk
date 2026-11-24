- In Basilisk 1.7.2 through 2.12.2, :ref:`spacecraftReconfig` treated the documented
  normalized semimajor-axis target as a distance in meters. A zero target was unaffected.
  Backport the normalized target conversion and its one-orbit mean-anomaly drift correction
  when using a nonzero target on those releases.
