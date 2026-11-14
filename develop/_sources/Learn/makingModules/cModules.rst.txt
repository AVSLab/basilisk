.. _cModules:

Making C Modules
================
This section covers how to write a C Basilisk Module.  A sample C Basilisk module is found in :ref:`cModuleTemplate`.


Simulation Time
---------------

Basilisk passes the current simulation time to a C module as the unsigned
integer ``callTime`` in nanoseconds.  Convert an absolute timestamp to seconds
with an explicit cast and ``NANO2SEC``:

.. code-block:: c

   #include "architecture/utilities/macroDefinitions.h"

   double absoluteTimeSec = (double) callTime * NANO2SEC;  /* [s] */

This conversion remains finite during simulations longer than
:math:`2^{53}` nanoseconds, approximately 104 days, although a ``double`` can
no longer preserve every individual nanosecond at that scale.  Do not use
``nanoToSec(callTime)`` for an absolute timestamp because ``nanoToSec()``
deliberately returns ``NAN`` above that exact-integer limit.

For a relative time, subtract the integer timestamps before converting.  The
``diffNanoToSec()`` helper handles either timestamp order and preserves the
precision of a bounded interval:

.. code-block:: c

   double elapsedTimeSec = diffNanoToSec(callTime, previousTimeNanos);  /* [s] */

Store ``previousTimeNanos`` as a ``uint64_t`` and update it only after the
elapsed-time calculation.  ``diffNanoToSec()`` returns ``NAN`` if the interval
itself is greater than :math:`2^{53}` nanoseconds.  Rebase the stored reference
timestamp when exact nanosecond resolution is required across a long-running
simulation.


Unused Lifecycle Parameters
---------------------------

Basilisk lifecycle functions have framework-defined parameter lists.  Keep an
unused parameter in the function signature and cast it to ``void`` in the
function body to document that it is intentionally unused and suppress
compiler warnings:

.. code-block:: c

   void Reset_myModule(MyModuleConfig *configData, uint64_t callTime, int64_t moduleID)
   {
       (void) configData;
       (void) callTime;
       (void) moduleID;
   }

Remove the cast if the implementation later uses the parameter.


.. toctree::
   :maxdepth: 2

   cModules/cModules-1
   cModules/cModules-2
   cModules/cModules-3
   cModules/cModules-4
   cModules/cModules-5
