- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` raised
  ``ValueError`` for a whole 3LE file when any name line started with ``1`` or ``2``
  (for example ``1KUNS-PF``), because such lines were counted as element lines. Element
  lines are now recognized by their line number followed by a blank. With an earlier
  version, rename such satellites in the file before reading it.
