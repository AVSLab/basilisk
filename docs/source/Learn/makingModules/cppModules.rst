.. _cppModules:

Making C++ Modules
==================


This section covers how to write a C++ Basilisk Module.  A sample C++ Basilisk module is found in :ref:`cppModuleTemplate`.  If starting a new module, this sample module is a convenient starting point to copy the folder and rename the files and methods accordingly.  After that the desired functionality can be added to the module.


Simulation Time
---------------

Basilisk passes the current simulation time to ``Reset()`` and
``UpdateState()`` as the unsigned integer ``CurrentSimNanos`` in nanoseconds.
Convert an absolute timestamp to seconds with an explicit cast and
``NANO2SEC``:

.. code-block:: cpp

   #include "architecture/utilities/macroDefinitions.h"

   double absoluteTimeSec = static_cast<double>(CurrentSimNanos) * NANO2SEC;  // [s]

This conversion remains finite during simulations longer than
:math:`2^{53}` nanoseconds, approximately 104 days, although a ``double`` can
no longer preserve every individual nanosecond at that scale.  Do not use
``nanoToSec(CurrentSimNanos)`` for an absolute timestamp because
``nanoToSec()`` deliberately returns ``NAN`` above that exact-integer limit.

For a relative time, subtract the integer timestamps before converting.  The
``diffNanoToSec()`` helper handles either timestamp order and preserves the
precision of a bounded interval:

.. code-block:: cpp

   double elapsedTimeSec = diffNanoToSec(CurrentSimNanos, previousTimeNanos);  // [s]

Store ``previousTimeNanos`` as a ``uint64_t`` and update it only after the
elapsed-time calculation.  ``diffNanoToSec()`` returns ``NAN`` if the interval
itself is greater than :math:`2^{53}` nanoseconds.  Rebase the stored reference
timestamp when exact nanosecond resolution is required across a long-running
simulation.


Unused Lifecycle Parameters
---------------------------

Basilisk base classes define the lifecycle method signatures.  When an
implementation never uses one of these parameters, retain the parameter name
and mark it with the C++17 ``[[maybe_unused]]`` attribute:

.. code-block:: cpp

   void MyModule::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
   {
       // Reset the module state.
   }

Keeping the name allows Doxygen ``@param`` documentation to remain associated
with the argument.  Remove the attribute if the implementation later uses the
parameter.


.. toctree::
   :maxdepth: 2

   cppModules/cppModules-1
   cppModules/cppModules-2
   cppModules/cppModules-3
   cppModules/cppModules-4
   cppModules/cppModules-5
