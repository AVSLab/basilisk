.. _bskPrinciples-9:

Advanced: Using ``DynamicObject`` Basilisk Modules
==================================================
Basilisk modules such as :ref:`spacecraft` and :ref:`spacecraftSystem` are members of
the ``DynamicObject`` class.  This means they still have the regular Basilisk ``Reset()`` and
``UpdateState()`` methods, but they also have a state machine and integrator build in as these
modules have to integrate internal ordinate differential equations (ODEs).

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