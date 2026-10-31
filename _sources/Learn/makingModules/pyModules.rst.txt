.. _pyModules:

Making Python Modules
=====================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/making-pyModules.py>`.

Python modules are a good alternative to compiled C, C++, and Rust modules for quick prototyping.
They are defined entirely in a Python script, which means that there is no need
for a header (``.h``), definition (``.cpp``), or SWIG interface file (``.i``). However, they
are much slower than C, C++, or Rust modules, which will significantly slow down your simulation.

Python modules are implemented by subclassing ``SysModel`` from ``Basilisk.architecture.sysModel``.
Then, one can implement the ``__init__``,
``Reset``, and ``UpdateState`` methods in the same way that one would
implement these methods in C++. Remember to always call ``__init__`` of
the parent class ``SysModel`` if you are implementing your own ``__init__``.

Simulation Time
---------------

Basilisk passes the current simulation time to ``Reset()`` and
``UpdateState()`` as the integer ``CurrentSimNanos`` in nanoseconds.  Convert
an absolute timestamp to seconds with ``macros.NANO2SEC``:

.. code-block:: python

    from Basilisk.utilities import macros

    absolute_time_sec = float(CurrentSimNanos) * macros.NANO2SEC  # [s]

Python integers preserve the complete timestamp, while the conversion to a
floating-point value may lose individual nanoseconds during long simulations.
The result nevertheless remains finite beyond :math:`2^{53}` nanoseconds,
approximately 104 days.

For a relative time, subtract the integer timestamps before converting.  This
retains the precision of a small elapsed interval even when the absolute
simulation time is large:

.. code-block:: python

    elapsed_time_sec = (CurrentSimNanos - self.previous_time_nanos) * macros.NANO2SEC  # [s]

Store ``previous_time_nanos`` as a Python integer and update it only after the
elapsed-time calculation.

Unused Lifecycle Parameters
---------------------------

Basilisk still supplies every lifecycle argument when it calls a Python
module.  Keep an unused argument in the method signature and prefix its name
with an underscore to show that it is intentionally unused:

.. code-block:: python

    def Reset(self, _current_sim_nanos):
        """Reset the module state."""
        self.dummy = 0.0  # [-]

This convention also prevents unused-argument reports from common Python
linters.  Remove the underscore if the implementation later uses the value.

The ``moduleID`` value of these Python BSK modules will be a unique positive number,
same as with C, C++, and Rust BSK modules.

.. note::

    To include a pure Python BSK module in the generated Basilisk Python package
    and wheel, place it in a module folder and give the folder and Python file
    the same lower camel case name.  For example, a flight software module named
    ``someModule`` should live in
    ``src/fswAlgorithms/<moduleCategory>/someModule/someModule.py``.  The same
    pattern is supported under ``src/simulation``.  The ``src/architecture``
    folder contains support code and message infrastructure, not BSK modules.
    Define the BSK module class inside that file using upper camel case, such
    as ``class SomeModule(sysModel.SysModel):``.  After configuring and building
    Basilisk, users can import it from the corresponding Basilisk package, such
    as ``from Basilisk.fswAlgorithms import someModule`` or
    ``from Basilisk.simulation import someModule``, and instantiate it with
    ``someModule.SomeModule()``.

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
