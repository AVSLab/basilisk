- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` raised
  ``ValueError`` for a whole file when any set had a blank international designator
  (columns 10-17 of line 1), as CelesTrak's analyst-object TLEs do. Such sets now load with
  ``launchDate`` left as ``None``. With an earlier version, read those objects from OMM
  with :ref:`ommHandling` instead.
