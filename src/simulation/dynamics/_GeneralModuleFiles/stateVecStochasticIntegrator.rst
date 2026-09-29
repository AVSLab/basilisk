``StateVecStochasticIntegrator`` supplies the flat storage and candidate operations
shared by native stochastic methods. Models continue to register local noise counts
and write diffusion tangents through ``StateData``; they do not manage packed buffers.

During binding, shared local endpoints are assigned to global noise slots. Each slot
has a contiguous range of ``StochasticNoiseBinding`` records and packed tangent values.
Mappings back to state-local order preserve the order of special-policy increments.
The class also owns step-entry rollback, accepted intermediate state, candidate scratch,
and a drift baseline for sparse per-source trials.

Candidate phases describe which scratch operations are valid. A complete candidate
can become an intermediate base; sparse candidates reuse a common drift baseline;
all-Euclidean final candidates may accumulate terms before one scatter. Live states
are restored from the sparse baseline if callbacks changed them, including states
outside the previous noise slot.

``StochasticRKIntegratorBase`` adds generator ownership and output buffers.
``FlatStochasticWorkspace`` provides reusable method stages. See
:ref:`integratorArchitecture` for the three noise orderings and rollback limits.
