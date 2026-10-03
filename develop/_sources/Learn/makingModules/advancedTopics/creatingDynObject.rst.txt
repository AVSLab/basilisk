.. _creatingDynObject:

Creating ``DynamicObject`` Basilisk Modules
===========================================

Basilisk modules that inherit from the class :ref:`dynamicObject` are still regular Basilisk modules
that have the typical ``SelfInit()``, ``Reset()`` and ``UpdateState()`` methods.  However, they also contain
a state engine called ``dynManager`` of the class ``DynParamManager`` and own a
``StateVecIntegrator``.  :ref:`spacecraft` is an example of
a Basilisk module that is also inheriting from the ``DynamicObject`` class.

For the ownership, storage, and integrator-binding contracts behind this
interface, see :ref:`integratorArchitecture`.

In the spacecraft ``UpdateState()`` method the ``DynamicObject::integrateState()`` method is called.
This call integrates all the registered spacecraft states, as well as all the connect state
and dynamic effectors, to the next time step using the connected integrator type.  See
:ref:`scenarioIntegrators` for an example of setting the integrator on a dynamic object.

.. note::

    Integrators connected to ``DynamicObject`` instances don't have to be the same.
    It is possible to use an RK2 integrator on one spacecraft and the RK4 integrator on another.

Declare and initialize ODE states in the subclass's ``Reset()`` implementation, then
call ``finalizeStates()`` before integration. A typical Reset is:

.. code-block:: cpp

    void MyDynamicObject::Reset(uint64_t currentSimNanos)
    {
        this->registerStates();
        this->dynManager.finalizeStates();
        this->linkStates();
        this->initializeAfterStateLink(currentSimNanos);
    }

The first finalization allocates contiguous storage. Reacquire Eigen views after that
allocation; the returned ``StateData`` handles themselves remain valid. Later resets
reuse states by name and update their values in place. Their shapes, policies, and noise
connections must remain compatible with the fixed layout. Repeated finalization has no
effect. Initialization errors propagate to the caller without automatic value rollback.

The ``DynamicObject`` class contains two virtual methods ``preIntegration()`` and ``postIntegration()``.
The ``DynamicObject`` subclass must define what steps are to be completed before the integration step,
and what post-integrations must be completed.  For example, with :ref:`spacecraft` the pre-integration
process determines the current time step to be integrated and stores some values used.  In the post-integration
step the MRP spacecraft attitude states are checked to not have a norm larger than 1 and the conservative DV
component is determined.

Basilisk modules that are a subclass of ``DynamicObject`` are not restricted to mechanical integration
scenarios as with the spacecraft example.  See the discussion in :ref:`bskPrinciples-9` on how multiple
Basilisk modules that inherit from the ``DynamicObject`` class can be linked.  If linked,
then the associated modules' ordinary differential equations (ODEs) are integrated
simultaneously.

Integrator ownership
--------------------

A ``DynamicObject`` owns its active integrator through a C++ ``std::unique_ptr``.
In Python, use ``setIntegrator()`` or assign the ``integrator`` attribute:

.. code-block:: python

    scObject.setIntegrator(svIntegrators.svIntegratorRK4(scObject))

Both entry points transfer ownership to the dynamics object, so the Python
integrator variable may go out of scope before the simulation runs. Existing
scripts that call ``integrator.this.disown()`` or set ``integrator.thisown = False``
before installation remain supported; manual disowning is no longer necessary.
Replacing the integrator destroys the previous one and preserves the primary object's
list of synchronized dynamics objects. Passing the active integrator again is
a no-op. A newly supplied integrator that is rejected is destroyed; its Python
proxy must not be reused.

The ``integrator`` attribute and ``getIntegrator()`` return borrowed access to
the active integrator. That access is valid only until the integrator is
replaced or its owning dynamics object is destroyed. Python synchronization
connections retain their supplied secondary objects; see
:ref:`bskSynchronizedDynamicsLifetime` for the lifetime and configuration contract.

Custom C++ dynamics classes should use the
``setIntegrator(std::unique_ptr<StateVecIntegrator>)`` overload to transfer
ownership explicitly. Include ``<memory>`` for ``std::make_unique`` and
``<utility>`` for ``std::move``:

.. code-block:: cpp

    this->setIntegrator(std::make_unique<svIntegratorRK4>(this));

    auto replacement = std::make_unique<svIntegratorRK4>(this);
    this->setIntegrator(std::move(replacement));
    // replacement is now empty; this dynamics object owns the integrator.

The owning argument is consumed even when validation rejects the replacement;
the rejected integrator is destroyed and the active integrator remains installed.
Replacing a synchronized secondary's integrator logs a warning and discards the
replacement. Change the primary's integrator instead to preserve the group.

The raw-pointer overload remains available for Python and existing C++ callers,
including ``setIntegrator(new svIntegratorRK4(this))`` and the no-op when passing
``getIntegrator()`` back to the same object. Never construct a new ``unique_ptr``
from a borrowed integrator pointer. The C++ owning overload is hidden from Python.

The owning C++ ``integrator`` member is private. Replace direct assignments with
``setIntegrator()`` and use ``getIntegrator()`` when a borrowed raw pointer is
required. Do not manually delete the owned integrator. Compiled extensions must
be rebuilt against extension ABI version 3.

Create synchronization links through ``syncDynamicsIntegration()`` before the
first integration step. The ``DynamicObject`` base destructor removes these links
when either object is destroyed. It does not delete borrowed C++ dynamics objects.
Use ``getIntegrationOwner()`` to inspect a connection and ``getDynamics()`` to
inspect an integrator's group. Once synchronization is configured, use
``setIntegrator()`` for replacements so that the new integrator receives the
existing group.
