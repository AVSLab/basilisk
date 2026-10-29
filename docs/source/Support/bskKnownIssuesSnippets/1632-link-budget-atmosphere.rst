- GitHub issue 1632: :ref:`linkBudget` underestimated atmospheric attenuation on Earth-space slant paths by
  repeatedly applying a ground-to-layer refraction ratio to the running elevation cosine. Each layer now uses
  the ground ray invariant and its own elevation when accumulating loss. Atmospheric layer generation also
  produced nearly constant one-meter steps and overflowed near 72.4 km for a sea-level station, silently
  stopping below the intended 100 km limit. A finite exponential grid now covers the complete altitude interval
  and samples atmospheric properties at layer midpoints. Cells are split at the atmosphere's temperature and
  pressure formula boundaries to reduce absolute attenuation errors near absorption lines. The resulting
  sea-level grid uses 931 cells instead of 70,979. Absolute attenuation regression tests use real oxygen and
  water-vapor absorption tables and a converged fine-grid reference. A committed reference-generation utility
  reproduces the 1 m versus 0.5 m convergence comparison. Quantitative tests cover the existing per-layer sine
  clamp of 0.0872, including low elevations and both sides of its threshold; clamp behavior is unchanged.
