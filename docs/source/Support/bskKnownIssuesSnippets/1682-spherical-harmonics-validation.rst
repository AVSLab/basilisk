- GitHub issue 1682: In tagged releases v2.2.1 through v2.12.2,
  :ref:`sphericalHarmonicsGravityModel` accepted a ``maxDeg`` larger than its
  manually configured ``cBar`` or ``sBar`` tables, as well as rows with missing
  orders. Field evaluation could read outside coefficient storage and crash.
  Changing the degree after initialization could also exceed the recurrence
  storage, and repeated initialization appended caches instead of rebuilding
  them, potentially producing incorrect acceleration. Initialization now
  requires at least ``maxDeg + 1`` rows in each table and at least ``l + 1``
  entries in each retained row ``l``. It rebuilds all recurrence caches and
  invalidates failed attempts. Field evaluation raises ``BasiliskError`` for
  missing initialization, insufficient initialized degree, or incomplete
  coefficient storage. On affected versions, use the file loader with a
  supported degree or validate manual table dimensions, and create a fresh
  model when changing the degree. With the fix, correct rejected inputs and
  initialize successfully before evaluating the field again.
