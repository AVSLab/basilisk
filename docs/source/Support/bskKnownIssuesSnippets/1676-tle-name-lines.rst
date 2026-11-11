- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` raised
  ``ValueError`` for a whole 3LE file when any name line started with ``1`` or ``2``
  (for example ``1KUNS-PF``), because such lines were counted as element lines. Issue
  #1676 is fixed by recognizing complete record layouts before counting element-line
  prefixes, preserving names such as ``1 TEST`` and ``2 TEST`` and their associated
  satellites, including files with trailing blank lines. These prefixes are also
  accepted in optional 2LE file titles. Recognizable truncated element headers are
  rejected as possible orphaned lines rather than consumed as names or file titles.
  This includes longer fragments with malformed catalog fields when later field
  structure still identifies them as element lines, and shortened headers when
  intact sequences of element fields identify columns shifted by missing characters.
  Prefix an intentional element-like title with ``0`` followed by a space to
  disambiguate it; for a satellite name, that prefix is retained in ``satName``.
  With an earlier version, rename satellites or file titles starting with ``1`` or
  ``2`` before reading the file.
