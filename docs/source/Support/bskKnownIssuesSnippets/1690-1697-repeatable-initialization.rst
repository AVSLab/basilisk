- In Basilisk ``2.12.2``, :ref:`waypointReference` retains its end-of-file flag after
  reinitialization, causing subsequent runs to hold an early waypoint instead of
  advancing through the file (#1690). Repeated resets also leak file streams, and
  destruction before the first file-open attempt can dereference an uninitialized
  pointer. Reset now clears the end-of-file flag and replaces an owned file stream;
  destruction safely releases that stream. For affected releases, use a fresh,
  successfully initialized module for each run to avoid stale playback state.
- In Basilisk ``2.12.2``, :ref:`msisAtmosphere` can change its epoch on repeated
  initialization when ``epochDoy`` is greater than 31, producing different density
  and temperature outputs for otherwise identical inputs (#1697). Changing the
  day or reverting to the default epoch can also retain the previous month or year.
  The module now rebuilds the complete default epoch before applying ``epochDoy``.
  For affected releases, supply an explicit ``epochInMsg``; this path already
  reconstructs the epoch on each reset and retains precedence over ``epochDoy``.
