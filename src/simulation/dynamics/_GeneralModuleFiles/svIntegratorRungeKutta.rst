``svIntegratorRungeKutta<numberStages>`` implements fixed-step explicit
Runge--Kutta integration from a Butcher tableau. Concrete RK methods supply the
stage count and coefficients. The constructor rejects nonfinite coefficients
and matrices that are not strictly lower triangular.

The class uses ``FlatStateBinding`` for topology and owns a flat base state,
candidate scratch, combined drift, and one contiguous derivative column per stage.
Each stage makes its candidate visible before evaluating the dynamics. Euclidean
states use vector arithmetic; special states dispatch through their registered
``StateUpdatePolicy``. A single all-Euclidean object's live buffer can serve directly
as the candidate destination.

On failure, step-entry state is restored if the participating dynamics objects are
still alive. Derivative buffers and external callback effects are not checkpointed.
See :ref:`integratorArchitecture` for equations and lifecycle rules.
