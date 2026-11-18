- ``pixelLineBiasUKF`` in Basilisk 2.12.2 and earlier releases stores six post-fit
  residuals in a three-element array, overwriting the navigation output timestamp,
  ``maxTimeJump``, and the cached circle timestamp after a measurement update
  (#1707). The navigation timestamp also remains at the reset epoch when no
  measurement is processed. The fix sizes and clears the entire residual array
  and tags the navigation output with the filter state's time of validity.
  Upgrading is required to prevent the overwrite; using the filter output's
  ``timeTag`` only works around the incorrect navigation timestamp.
