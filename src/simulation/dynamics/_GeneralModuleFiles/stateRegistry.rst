``StateRegistry`` implements the state storage owned by :ref:`dynParamManager`.
It owns named handles, update policies, layouts, value buffers, and noise connections.

Registration creates temporary matrices. First finalization computes offsets,
allocates contiguous buffers, and redirects handles into those buffers. The layout
then remains fixed. Later registration retrieves existing states by name and checks
that their declarations match; values are updated in place without staging or rollback.

Construction and registration are controlled by the manager. C++ callers inspect
and resolve storage through ``DynParamManager::getStateRegistry()``.

``StateLayout`` offsets address the registry's own arrays of doubles. They differ
from the integrator-global offsets in ``FlatStateBinding``. ``StateBufferSegment``
identifies its registry, buffer kind, and scalar range. The registry must remain alive
while a segment or a pointer resolved from it is in use.

``getSegment(kind, firstSlot, elementCount)`` addresses state, derivative, and
diffusion storage through the same interface. ``segmentView(segment)`` returns a
borrowed column matrix whose writes update the live buffer. Calling it on a const
registry returns a read-only view. Typed segment constructors and pointer accessors
remain available when the buffer kind is known.

A segment starts at a registration slot and ends at a complete record boundary.
For diffusion, one record contains all local noise tangents for that state.
States without noise contribute no diffusion scalars. For example, if state A has
two scalar tangents and state B has one, their diffusion segment contains
``[A/source0, A/source1, B/source0]``. Shared sources still occupy separate local
contributions. Stochastic workspaces gather them into global source order during
capture.

Acquire segment views after finalization. Their addresses remain stable across
later resets, and the registry must outlive them. Module callbacks can continue
to use ``StateData::stateView()``, ``derivativeView()``, and ``diffusionView(index)``
for shaped access to individual records.

See :ref:`integratorArchitecture` for the complete storage and integration call path.
