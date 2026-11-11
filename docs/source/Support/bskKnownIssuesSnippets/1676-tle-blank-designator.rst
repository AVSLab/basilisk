- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` raised
  ``ValueError`` for a whole file when any set had a blank international designator
  (columns 10-17 of line 1), as CelesTrak's analyst-object TLEs do. Such sets now load with
  ``launchDate`` left as ``None``; ``launchNo`` and ``pol`` retain their defaults of
  ``1`` and ``"A"``, which do not represent known launch metadata for these objects.
  Upgrade to a version containing this fix or backport the reader change to read
  these sets. Until then, omit the complete affected sets to read the rest of the file.
  The :ref:`ommHandling` reader was introduced during the 2.13 beta cycle and is
  unavailable in the affected releases.
