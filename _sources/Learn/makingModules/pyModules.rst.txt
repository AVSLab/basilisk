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
