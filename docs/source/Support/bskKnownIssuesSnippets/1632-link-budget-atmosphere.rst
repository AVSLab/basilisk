- GitHub issue 1632: :ref:`linkBudget` underestimated atmospheric attenuation on Earth-space slant paths by
  repeatedly applying a ground-to-layer refraction ratio to the running elevation cosine. Each layer now uses
  the ground ray invariant and its own elevation when accumulating loss. Atmospheric layer generation also
  produced nearly constant one-meter steps and overflowed near 72.4 km for a sea-level station, silently
  stopping below the intended 100 km limit. A finite exponential grid now covers the complete altitude interval
  and samples atmospheric properties at layer midpoints, using 922 layers instead of 70,979 at sea level.
