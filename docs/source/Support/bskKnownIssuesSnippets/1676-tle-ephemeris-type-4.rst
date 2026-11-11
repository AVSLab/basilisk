- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` converted
  TLEs with ephemeris type 4 (SGP4-XP) using plain SGP4,
  which can produce incorrect states because those sets carry BTERM and AGOM in
  the B* and second-derivative columns. Unsupported sets are now skipped with a
  printed warning, while other readable sets are returned. If every attempted set
  fails, the first error is raised; a file containing only otherwise valid type-4
  sets raises ``ValueError``. With an affected version, check column 63 of line 1
  and exclude complete sets unless that field is blank, ``0`` or ``2``.
  Changing a type-4 field to ``0`` or ``2`` does not convert SGP4-XP data to SGP4.
