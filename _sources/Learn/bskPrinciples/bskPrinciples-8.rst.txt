.. _bskPrinciples-8:

Advanced: Enabling and Disabling Tasks
======================================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-8.py>`.

Next we study how the Basilisk tasks can be both disabled and enabled.  Why would we do this?  You might setup a set of BSK modules to mimic a sun-pointing behavior in a task.  Next, you setup another set of BSK modules to create a science pointing mode.  Being able to enable and disable tasks means that you can creates these tasks once, but then control which flight software modules are actually executed.

.. image:: ../../_images/static/qs-bsk-8.svg
   :align: center

The sample script below sets up a single process which contains 2 tasks called ``cTask`` and ``cppTask``.  The :ref:`cModuleTemplate` and :ref:`cppModuleTemplate` are added to these tasks respectively.


.. literalinclude:: ../../codeSamples/bsk-8.py
   :language: python
   :linenos:
   :lines: 18-

After performing the typical module initialization the script executes a single simulation step.  The terminal output below shows that both the C and C++ modules have been executed, meaning both ``cTask`` and ``cppTask`` are enabled.

To disable all tasks within a process, the command ``disableAllTasks()`` can be called on the process variable.  A single simulation step is executed with print statements before and after to illustrate not no tasks are being executed, as expected.

Next, the :ref:`SimulationBaseClass` command ``enableTask(name)`` is used to turn on the ``cTask``. The string argument is the name of the task being enabled.  After executing another simulation step the terminal output illustrates that the C module is again executed.  This is repeated for enabling ``cppTask``.

To disable a single task, this is done with the :ref:`SimulationBaseClass` method ``disableTask(name)``.  The string argument is the name of the task being disabled.  The expected terminal output for this script is illustrated below.

.. code-block::

    (.venv) source/codeSamples % python bsk-8.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: C Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    all tasks disabled
    BSK executed a single simulation step
    BSK_INFORMATION: C Module ID 1 ran Update at 2.000000s
    BSK executed a single simulation step
    BSK_INFORMATION: C Module ID 1 ran Update at 3.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 3.000000s
    BSK executed a single simulation step
    BSK_INFORMATION: C Module ID 1 ran Update at 4.000000s
    BSK executed a single simulation step


