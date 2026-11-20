- In Basilisk 2.12.2, :ref:`spiceInterface` could round seconds just below 60 in a linked
  ``epochInMsg`` up to 60 and terminate the process during initialization (issue #1686).
  On affected releases, leave the epoch message unlinked and supply the full fractional
  seconds in a valid ``UTCCalInit`` string. The fix preserves sub-microsecond input precision
  at the minute boundary and raises a catchable ``BasiliskError`` when SPICE rejects an epoch.
