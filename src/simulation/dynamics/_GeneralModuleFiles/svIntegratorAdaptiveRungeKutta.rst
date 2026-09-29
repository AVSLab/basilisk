``svIntegratorAdaptiveRungeKutta<numberStages>`` extends the fixed RK machinery
with an embedded lower/higher-order pair and internal step-size control.
The inherited ``bArray`` forms the lower-order candidate; ``bStarArray`` forms
the higher-order candidate accepted when the scaled error is at most one.

The integrator covers one requested interval through accepted internal substeps.
Its inherited base buffer tracks the latest accepted substep, while ``entryState``
preserves the beginning of the requested interval for exception rollback.
Nonfinite results and a step size too small to advance representable time raise
errors rather than continuing indefinitely.

Relative and absolute tolerances resolve independently in this order:

#. An override for a particular dynamics object and state name.
#. An override for the state name across synchronized objects.
#. The global default.

Resolved ``ToleranceSpan`` entries follow descriptor order. Configuration revisions
and snapshots of the legacy public defaults determine when they must be refreshed.
Whole-state control uses the norm of the stored-state error; per-component control
uses scalar errors. A special update policy does not supply a separate error metric.
See :ref:`integratorArchitecture` for the scaling equations and buffer relationships.
