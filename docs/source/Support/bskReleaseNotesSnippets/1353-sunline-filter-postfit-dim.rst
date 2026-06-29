- Fixed the post-fit residual dimension in :ref:`sunlineSEKF` and :ref:`okeefeEKF`. The residual term
  ``Hx = measMat * x`` used the full ``SKF_N_STATES`` width instead of the reduced state width each filter
  actually carries (``EKF_N_STATES_SWITCH`` for the SEKF, ``SKF_N_STATES_HALF`` for okeefe), so the multiply
  strode the measurement matrix across the wrong row width and over-read the state-error vector, corrupting
  the reported ``postFitRes`` during the convergence transient. The state and covariance estimates were
  unaffected (issue #1353).
