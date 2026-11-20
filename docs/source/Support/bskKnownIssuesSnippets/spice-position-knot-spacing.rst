- In Basilisk 2.12.x, :ref:`spiceInterface` position reconstruction could return nonfinite
  position and velocity for knot intervals longer than :math:`2^{53}` nanoseconds (about
  104.25 days). The fix uses the configured interval in seconds for each knot query.
  On affected releases, use a shorter position knot interval or exact SPICE position queries.
