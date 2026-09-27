.. _bskPrinciples-9:

Advanced: Using ``DynamicObject`` Basilisk Modules
==================================================
Basilisk modules such as :ref:`spacecraft` inherit from the ``DynamicObject`` class.
They have the regular Basilisk ``Reset()`` and ``UpdateState()`` methods, as well as
an internal state manager and integrator for solving ordinary differential equations (ODEs).

The ``DynamicObject`` class has the ability to integrate not just the ODEs of the one Basilisk module,
but it is possible to synchronize the integration across multiple subclasses of ``DynamicObject``
instances.  Consider an example where the integration of two spacecraft instances  ``scObject`` and ``scObject2``
must be synchronized because effectors are used that create forces and torques onto both spacecraft.
From python, this can be done elegantly using::

    scObject.syncDynamicsIntegration(scObject2)

This steps ties the integration of ``scObject2`` to the integration of ``scObject``.  Thus, even if
``scObject`` is setup in a Basilisk task running 10Hz using the RK4 integrator, and ``scObject2`` is
in a 1Hz task and specifies an RK2 integrator, the ODE integration of the primary object overrides
the setup of the sync'd ``DynamicObjects``.  As a result both objects would be integrated using
the RK4 integrator at 10Hz.  The ``UpdateState()`` method of ``scObjects2`` would still be called
at 1Hz as this method is called at the task update period.

.. note::

    The ``syncDynamicsIntegration()`` method is not limited to syncing the ODE integration across
    two ``DynamicObject`` instances.  Rather, the primary ``DynamicObject`` contains a standard
    vector of points to sync'd ``DynamicObject`` instances.  Thus, the ODE integration of
    an arbitrary number of ``DynamicObject`` integrations can be thus synchronized.

The integration type is determined by the integrator assigned to the primary ``DynamicObject`` to
which the other ``DynamicObject`` integrations is synchronized.  By default this is the ``RK4``
integrator.  It doesn't matter what integrator is assigned to the secondary ``DynamicObject`` instances,
the integrator of the primary object is used.

.. _bskSynchronizedDynamicsLifetime:

Lifetime of Synchronized Objects
--------------------------------

In Python, ``primary.syncDynamicsIntegration(secondary)`` keeps a reference to
``secondary`` on ``primary``. A Python-owned secondary can therefore leave local
scope without being destroyed while the primary still uses it. Replacing the
primary's integrator through ``setIntegrator()`` preserves both the connection
and this reference. Synchronization does not add a module to a simulation task
or initialize it; continue to schedule and initialize both modules as usual.

Call this method on the owning Python primary object. A borrowed primary proxy,
such as the scene returned by ``MJBody.getScene()``, raises ``BasiliskError`` before
changing any connection. A temporary proxy cannot hold the secondary for the
native primary's lifetime. Use the original ``MJScene`` object to configure the
connection instead; repeated calls through that owning object remain supported.

When the primary is destroyed, surviving secondaries are detached and their
``isDynamicsSynced`` properties become false. They can use their own integrators again
or be connected to another primary. C++ callers retain ownership of their
dynamics objects: destroying a secondary removes it from the primary's
integration list. Only synchronization links are detached; other effector and
message connections retain their usual lifetime requirements.

Use one primary for each group and connect every secondary directly to it:

.. code-block:: python

    primary.syncDynamicsIntegration(secondary1)
    primary.syncDynamicsIntegration(secondary2)

Repeating the same connection is a no-op. A null argument, self-synchronization,
a secondary assigned to two primaries, or a nested group raises ``BasiliskError``.
For example, after the calls above, ``secondary1.syncDynamicsIntegration(other)``
is invalid; connect ``other`` to ``primary`` instead.

Configure new connections before either object's integrator has taken its first
step. To regroup an object that has already integrated independently, install a
new integrator before connecting it. Destroy connected objects outside integration
calls, and keep the
group on one worker as described in :ref:`bskPrinciples-13`. A borrowed Python
proxy for a C++-owned dynamics object does not extend that external owner's
lifetime.
