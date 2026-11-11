- In versions 2.8.28 through 2.12.2, :ref:`tleHandling` ``satTle2elem()`` failed for
  the whole file when any one set could not be read (for example a bad checksum or
  a malformed field), so the valid sets in the file were lost too. Complete sets
  that raise validation or arithmetic errors, including epoch overflow and division
  by zero, are now skipped with a printed warning. If every attempted set fails,
  the first set's original error is raised. File-level format checks, including
  orphaned element lines, can still reject the whole file.
