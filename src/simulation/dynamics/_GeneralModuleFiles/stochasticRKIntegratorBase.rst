``StochasticRKIntegratorBase`` connects the shared stochastic topology to a
``GaussianNoiseGenerator`` and the concrete method's workspace. The public
``setRNGSeed()`` and ``setNoiseGenerator()`` configuration methods remain independent
of the stage recurrence.

The binding hook allocates topology-sized Wiener and auxiliary output, then invokes
``bindStochasticMethodStorage()``. Incomplete preparation is discarded on failure.
Retirement calls the method's release hook and frees shared numerical storage while
retaining generator configuration.

Concrete methods implement ``integrateImpl()`` and request only the noise output
they need. Wiener-only methods avoid unused auxiliary draws; other methods can request
an auxiliary prefix. Built-in generators fill preallocated buffers. Compatibility
adapters permit older custom generators, which may still allocate or produce unused
auxiliary values.

A local shared pointer keeps a custom generator alive during its callback, including
if that callback replaces the configured generator. The exact built-in generator can
use a cached direct pointer. State rollback does not rewind either generator.
See :ref:`integratorArchitecture` for the complete stochastic call path.
