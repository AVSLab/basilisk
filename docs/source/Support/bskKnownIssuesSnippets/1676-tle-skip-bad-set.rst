- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` raised
  ``ValueError`` for the whole file when any one set could not be read (for example a bad
  checksum or a malformed field), so the valid sets in the file were lost too. Unreadable
  sets are now skipped with a printed warning; a file in which no set can be read still
  raises. A file whose counts of line-1 and line-2 lines differ still raises as a whole.
