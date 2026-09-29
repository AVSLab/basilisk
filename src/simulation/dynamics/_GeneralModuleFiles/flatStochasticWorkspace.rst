``FlatStochasticWorkspace`` holds stage data used by several native stochastic
Runge--Kutta families. It borrows finalized object, noise-binding, and noise-slot
descriptors from the integrator. Those descriptors must outlive the bound workspace.

``bind()`` allocates the requested drift columns, diffusion columns, combination
buffers, and scratch vectors before committing the borrowed references. ``reset()``
releases both storage and references. Neither operation belongs in a stage loop.

Drift rows follow derivative-buffer order. Diffusion rows are packed by global noise
source and contain only registered tangents. Each scratch vector has one entry per
global source. Capture methods copy live values into stage columns; combination
methods write weighted sums into reusable buffers without resizing. Methods that
write one diffusion slot leave the other slots unchanged, so the recurrence must
populate every slot it later reads.

See :ref:`integratorArchitecture` for how this workspace fits between noise topology,
candidate assembly, and concrete stochastic recurrences.
