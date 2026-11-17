- In Basilisk 2.12.2, :ref:`chebyPosEphem` falls back to record zero when no
  record covers the requested ephemeris time. With multiple records, times after
  the last interval or in a gap can therefore return position and velocity from
  the wrong endpoint (issue #1685). The fix selects the nearest configured
  interval endpoint, ignores unused slots, and retains array-order priority for
  overlaps and equal-distance ties. On affected versions, keep requested times
  within configured coverage or load only the intended record into slot zero
  before resetting and evaluating the module.
