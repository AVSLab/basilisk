.. _pyModules:

Making Python Modules
=====================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/making-pyModules.py>`.

Python modules are a good alternative to C and C++ modules for quick prototyping.
They are defined entirely in a Python script, which means that there is no need
for a header (``.h``), definition (``.cpp``), or SWIG interface file (``.i``). However, they 
are much slower than C or C++ modules, which will significantly slow down your simulation.

Python modules are implemented by subclassing ``SysModel`` from ``Basilisk.architecture.sysModel``. 
Then, one can implement the ``__init__``,
``Reset``, and ``UpdateState`` methods in the same way that one would
implement these methods in C++. Remember to always call ``__init__`` of
the parent class ``SysModel`` if you are implementing your own ``__init__``.

The ``ModelTag`` value of these python BSK modules will be a unique positive number,
same as with C/C++ BSK modules.

All Python modules have a logger stored in ``bskLogger`` (although it will
not be available until the module has been added to a simulation). Additionally,
you may declare any other variables, methods, messages, etc. within your Python module.

The script below expands on the code shown in :ref:`bskPrinciples-2` to include
a Python module.

.. literalinclude:: ../../codeSamples/making-pyModules.py
   :language: python
   :linenos:
   :lines: 18-

Running the above code prints:

.. code-block::

    (.venv) source/codeSamples % python making-pyModules.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Reset in TestPythonModule
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    InitializeSimulation() completed...
    BSK_INFORMATION: C Module ID 3 ran Update at 0.000000s
    BSK_INFORMATION: Python Module ID 4 ran Update at 0.0s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: C Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C Module ID 3 ran Update at 5.000000s
    BSK_INFORMATION: Python Module ID 4 ran Update at 5.0s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 5.000000s
    BSK_INFORMATION: C Module ID 1 ran Update at 5.000000s
    Recorded mod2.dataOutMsg.dataVector:  [[2. 1. 0.]
    [5. 2. 0.]]

Note how the Python module made use of ``bskLogger``, the ``Reset``
and ``UpdateState`` were called, how the priority of the Python
module was respected, and how messaging happened between a C++
and Python module.

The scenario :ref:`scenarioAttitudePointingPy` further shows how to define Python modules.

Deprecated way of creating Python modules
-----------------------------------------
.. warning:: 

    This section discusses the deprecated way of setting up Python modules
    in Basilisk versions earlier than 2.2.
    Users should refer to the previous section when setting up new simulation
    scripts using Python modules.

Apart from the way to shown above, there exist an older way to create 
Python modules which has now been deprecated. This section briefly
discusses this older method, its disadvantages, and how to update to the
new system. Note that this deprecated method is pending for removal;
users are advised to update to the new system.

Before the new system, Python modules had to be added to separate
Python processes, which could not have C/C++ modules. Moreover, this
Python processes always had to have a lower priority than C++/C
processes, which effectively meant that all Python modules would run
after the C++/C modules. This severely limited users' control of the
execution order of their simulation.

Moreover, the syntax for creating these modules was slightly different
than for C++ modules:

- The class inherited from ``PythonModelClass`` instead of ``SysModel``
- The ``ModelTag`` had to be passed to the constructor of the class
- One had to overload ``reset`` and ``updateState``, instead of ``Reset`` and ``UpdateState``

In order to update python simulation scripts that use the deprecated system to the
new system, one needs to:

- Replace ``CreateNewPythonProcess`` by ``CreateNewProcess``, as the new Python modules
  can be added to regular processes.
- Make the Python module class inherit from ``SysModel``, and not from ``PythonModelClass``
  Note that you must import ``SysModel`` from ``Basilisk.architecture.sysModel``.
- Instead of passing the module tag and priority in the constructor, set the tag by
  setting the ``ModuleTag`` attribute (similar to C++ modules), and set the priority on the ``addTask`` method.
- Rename ``selfInit``, ``reset``, and ``updateState`` to ``SelftInit``, ``Reset``, and ``UpdateState``.

With this depreciated manner of creating a python Basilisk module the ``ModelTag`` value
is an unique negative number, while the C/C++ modules had unique positive numbers.
This means that updating your simulation script
might change the ID of your modules compared to previous versions.

It is possible that you may not even need a separate process
for your Python modules, so consider adding the Python modules directly
to other existing processes, always with a lower priority if you want
to retain the older behaviour.

The scenario :ref:`scenarioAttitudePointingPyDEPRECATED` shows both the 
deprecated way of creating a Python module.
