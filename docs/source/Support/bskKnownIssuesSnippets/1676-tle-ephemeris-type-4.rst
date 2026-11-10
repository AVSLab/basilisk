- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` initialized
  Basilisk spacecraft from TLEs with ephemeris type 4 (SGP4-XP) by running plain SGP4,
  which returns a plausible but wrong state because those sets carry BTERM and AGOM in
  the B* and second-derivative columns. Such sets now raise ``ValueError``. With an
  earlier version, check column 63 of line 1 and use only sets with type 0 or 2.
