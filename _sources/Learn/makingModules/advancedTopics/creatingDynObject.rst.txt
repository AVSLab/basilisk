.. _creatingDynObject:

Creating ``DynamicObject`` Basilisk Modules
===========================================

Basilisk modules that inherit from the class :ref:`dynamicObject` are still regular Basilisk modules
that have the typical ``SelfInit()``, ``Reset()`` and ``UpdateState()`` methods.  However, they also contain
a state engine called ``dynManager`` of the class ``DynParamManager``, as well as an integrator
pointer called ``integrator`` of class ``StateVecIntegrator``.  :ref:`spacecraft` is an example of
a Basilisk module that is also inheriting from the ``DynamicObject`` class.

In the spacecraft ``UpdateState()`` method the ``DynamicObject::integrateState()`` method is called.
This call integrates all the registered spacecraft states, as well as all the connect state
and dynamic effectors, to the next time step using the connected integrator type.  See
:ref:`scenarioIntegrators` for an example how how to set the integrator on a dynamic object.

.. note::

    Integrators connected to ``DynamicObject`` instances don't have to be the same.
    It is possible to use an RK2 integrator on one spacecraft and the RK4 integrator on another.

The ``initializeDynamics()`` virtual method must be defined in the ``DynamicObject`` subclass.
It typically performs the required setup steps, including registering the ODE states that are
to be integrated.

The ``DynamicObject`` class contains two virtual methods ``preIntegration()`` and ``postIntegration()``.
The ``DynamicObject`` subclass must define what steps are to be completed before the integration step,
and what post-integrations must be completed.  For example, with :ref:`spacecraft` the pre-integration
process determines the current time step to be integrated and stores some values used.  In the post-integration
step the MRP spacecraft attitude states are checked to not have a norm larger than 1 and the conservative DV
component is determined.

Basilisk modules that are a subclass of ``DynamicObject`` are not restricted to mechanical integration
scenarios as with the spacecraft example.  See the discussion in :ref:`bskPrinciples-9` on how multiple
Basilisk modules that inherit from the ``DynamicObject`` class can be linked.  If linked,
then the associated module ordinate differential equations (ODEs) are integrated
simultaneously.

