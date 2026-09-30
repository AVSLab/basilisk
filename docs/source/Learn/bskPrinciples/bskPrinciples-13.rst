.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
   Distributed under the ISC license; see LICENSE.

.. _bskPrinciples-13:

Advanced: Multithreaded Simulations
===================================

This tutorial builds on :ref:`bskPrinciples-1`, :ref:`bskPrinciples-2`, and
:ref:`bskPrinciples-5`. Once a simulation has several independent groups of
modules, Basilisk can execute those groups concurrently on worker threads.
For example, a constellation simulation can assign each independent spacecraft
and its associated modules to a separate Basilisk process.

Processes and worker threads
----------------------------

A Basilisk process is a group of tasks within the same application. The simulation
assigns each process to one C++ worker thread, and that worker executes the
process's tasks and modules in their scheduled order. A worker can execute more
than one process. Creating additional tasks inside a single process does not
distribute those tasks across threads.

The default configuration uses one worker. To request more workers, call
``scSim.TotalSim.resetThreads(num_threads)`` before initializing the simulation.
The count must be positive. During initialization, Basilisk assigns processes
without an explicit thread assignment in round-robin order across the workers.

Choosing independent work
-------------------------

Processes on different workers must be able to advance independently. Keep
modules that share mutable state, coupled dynamics, or changing messages on the
same worker. Each module instance should be scheduled on only one worker.

.. warning::

    Basilisk's standard message reads and writes do not synchronize access
    between worker threads. A process priority does not establish execution order
    across different workers. For example, placing spacecraft dynamics on one
    worker and its message-dependent flight software on another does not provide
    a safe producer-consumer schedule. Keep those processes on the same worker.

Adding workers is useful when there is enough independent computation to offset
thread coordination costs. More workers than independent processes leaves workers
idle. Measure runtime for the intended simulation before selecting a thread count;
the short example below demonstrates configuration, rather than speedup.

Two independent spacecraft
--------------------------

.. sidebar:: Source Code

    Download the runnable example:
    :download:`bsk-13.py </../../docs/source/codeSamples/bsk-13.py>`.

This example creates two spacecraft with constant velocities and no applied
forces. Each spacecraft has its own process, task, and state recorder. The
recorder runs in the same task as its spacecraft so that reading the state
message follows the spacecraft update on the same worker.

.. literalinclude:: ../../codeSamples/bsk-13.py
   :language: python
   :linenos:
   :start-at: import numpy as np

The two processes are automatically assigned to the two workers. The normal
``InitializeSimulation()``, ``ConfigureStopTime()``, and ``ExecuteSimulation()``
calls are unchanged. Both spacecraft start at the origin and propagate for one
second. Their final positions should therefore be:

.. code-block:: text

    spacecraft0: x = 1.000 m
    spacecraft1: x = 2.000 m

The script checks these positions against constant-velocity motion. Calling
``run(num_threads=1)`` runs the same processes on one worker and produces the same
results. For a larger example, see :ref:`scenario_BasicOrbitMultiSat_MT`.

Assigning processes explicitly
------------------------------

Use ``addProcessToThread()`` when particular processes must share a worker.
Thread indices start at zero and must be less than the configured thread count.
For existing dynamics and flight-software processes, the following assignments
keep both on worker zero:

.. code-block:: python

    scSim.TotalSim.resetThreads(2)
    scSim.TotalSim.addProcessToThread(dynProcess.processData, 0)
    scSim.TotalSim.addProcessToThread(fswProcess.processData, 0)
    scSim.InitializeSimulation()

Make explicit assignments after configuring the pool and before the first
initialization. Processes that are not assigned explicitly still use automatic
assignment. Automatic assignment follows the simulation's process order. For
explicit assignments, processes are added to a worker in the order of these calls;
place the producer before its consumer when both are due at the same simulation
time. Task and module priorities still apply within each process. Priorities do not
coordinate different workers.

.. _bskThreadOwnership:

Reinitialization, ownership, and shutdown
-----------------------------------------

The simulation owns its worker threads. Calling ``InitializeSimulation()`` again
rebuilds the worker pool before resetting the simulation. This clears explicit
process-to-thread assignments and returns to automatic assignment. Recreate the
simulation configuration when repeating a run that requires an explicit mapping.
To continue an existing run without resetting it, set a later stop time and call
``ExecuteSimulation()`` again.

Resetting or destroying the pool requests shutdown and joins each worker before
releasing its state. ``scSim.TotalSim.deleteThreads()`` explicitly releases the
pool and is safe to call repeatedly, including after a module throws an exception.
To reuse a deleted pool, call ``resetThreads(num_threads)`` and then
``InitializeSimulation()``. Call these lifecycle methods from the simulation's
controlling thread, after the current simulation call returns, rather than from
a module callback.

Processes, tasks and modules remain borrowed by the C++ scheduler;
``SimulationBaseClass`` retains their Python references. Normal simulation scripts
configure workers through the methods described above. The internal ``threadList``
and ``threadContext`` fields are no longer exposed to Python.

Custom C++ scheduling code
--------------------------

``SimModel::threadList`` now contains ``std::unique_ptr<SimThreadExecution>`` and
``SimThreadExecution::threadContext`` is a ``std::thread`` value. Use ``.get()``
when borrowing an execution record, and do not manually delete either owned
object. Compiled extensions must be rebuilt against extension ABI version 3.

Use ``SimThreadExecution::requestStop()`` to request shutdown and wake an idle
worker. ``killThread()`` remains available as a compatibility alias with the same
behavior; a separate ``unlockThread()`` call is no longer needed. Repeated calls
through either method release the worker semaphore only on the first stop request.

A stop request does not join the worker. Keep its execution record alive until
the worker finishes and has been joined. The execution record's destructor
requests shutdown and joins automatically. ``SimModel::deleteThreads()`` first
requests shutdown of every worker, then destroys and joins the records, preserving
the rule that all workers are signaled before any join begins. Call pool lifecycle
methods from the controlling thread as described above.
