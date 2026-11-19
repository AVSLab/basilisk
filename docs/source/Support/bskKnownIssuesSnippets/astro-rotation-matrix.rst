- In Basilisk ``2.12.2``, ``astroFunctions.rotationMatrix`` overwrites an
  off-diagonal element and leaves a diagonal element at one. This can distort
  rotated vectors and produces a nonidentity matrix at zero angle. The helper
  now fills the correct diagonal element. The existing ``TisserandPlot`` caller
  uses only the unaffected first column, so its curves are unchanged.
