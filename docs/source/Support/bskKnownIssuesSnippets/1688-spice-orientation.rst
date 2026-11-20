- In Basilisk 2.12.x, multi-knot :ref:`spiceInterface` orientation fits could err by up to
  180 degrees when the body turned more than half a revolution across the fit (issue #1688).
  Knot spacing above about 104.25 days could also produce nonfinite orientation and rate.
  On affected releases, use shorter intervals that keep the angular travel across the entire
  stencil well below 180 degrees, or use exact SPICE orientation with coverage at each requested time.
  The fix supports long orientation intervals and uses an exact query when even the local pair is ambiguous.
