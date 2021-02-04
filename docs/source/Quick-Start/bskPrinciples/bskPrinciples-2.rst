Adding Basilisk Modules
=======================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../codeSamples/bsk-2.py>`.

Adding C-Modules
----------------
Understanding now what a Basilisk process (Task Group) and a task list is, we can now move forward to adding Basilisk modules to a task.  :ref:`fswModuleTemplate` will be the basic module that we use in this discussion.  It is a very simple module used to illustrate how to make a C-based Basilisk module.  It is convenient for this discusion as it is setup as a tutorial module and prints out the module ID value when the module is executed.  This will make it simple in the simulation below that the desired module execution order is achieved.

.. image:: ../../_images/static/qs-bsk-addModules.svg
   :align: center
   :scale: 50 %

The following simulation creates a single process called ``dynamicsProcess`` and single 0.2Hz task called ``dynamicsTask``. As illustrated above, 3 copies of :ref:`fswModuleTemplate` are created where they are called ``Module1``, ``Module2`` and ``Module3``.  However, note that in this example we seek to executes modules 2 and 3 first and 1 last.

.. literalinclude:: ../../codeSamples/bsk-2.py
   :language: python
   :linenos:
   :lines: 18-

The resulting demonstration code is shown above.  Note that the C-based :ref:`fswModuleTemplate` must be imported from ``Basilisk.fswAlgorithm`` at the top of the file.

Next we create a copy of the C-based Basilisk module.  Let's discuss this process considering a generic module name ``someCModule``.  As C code doesn't know about class objects, in Basilisk we have to recreate some of the basic features of an object oriented language like C++.  Instead of having class variables the data of the C module is stored in the ``someCModuleConfig`` data structure.  For :ref:`fswModuleTemplate` you find this module configuration structure listed at the bottom of that web page, or you can see the structure definition in the ``someCModule.h`` header file.  Thus, to create a python copy of this module configuration structure use::

    moduleData = someCModule.someCModuleConfig()

Next, for Basilisk to be able to execute this module, we need to wrap this module data structure with a C++ interface.  These steps make it possible to have multiple C module copies be created that don't conflict with each other.  The module wrapped is done with::

    moduleWrap = scSim.setModelDataWrap(moduleData)

Each Basilisk module should have a unique name.  This is set using::

    moduleWrap.ModelTag = "someName"

In the code above you see these steps repeated 3 times to create three distinct copies of :ref:`fswModuleTemplate`.  The next step is to add these modules to the task list to execute them in the desired order.  The general commmand to add a C-based module to a task is::

    scSim.AddModelToTask("taskName", moduleWrap, moduleData, priority)

The first argument is the name of the task to which you are adding the module.  The 2nd and 3rd arguments are the module wrapper and module data variables.  The last argument is the optional integer priority argument.  If this is not provided, then the priority defaults to -1 and the modules are executed in the order they are added, but after modules with priority have been executed.  This is the same behavior as what we saw with processes and tasks earlier.

Looking at the above simulation code, note that ``Module1`` is added to the task list without any priority specified.  In contrasts, ``Module2`` and `Module3`` have the priorities 10 and 5 assigned.  The higher the module priority, the earlier it is evaluated.

.. note::

    Basilisk assigns unique positive ID numbers to C/C++ modules upon their creation.  Thus, in the above simulation code the modules 1, 2 and 3 will have the corresponding ID numbers 1, 2 and 3 because that is the order in which they are created.

If you execute this python code you should see the following terminal output:

.. code-block::

    source/codeSamples % python3 bsk-2.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    InitializeSimulation() completed...
    BSK_INFORMATION: Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 3 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 2 ran Update at 5.000000s
    BSK_INFORMATION: Module ID 3 ran Update at 5.000000s
    BSK_INFORMATION: Module ID 1 ran Update at 5.000000s


:ref:`fswModuleTemplate` logs in the ``Reset()`` method that a variable has been set to 0.  As we have 3 copies of this module, notice that this reset statement is seen 3 times.  This reset step occurs when we run ``scSim.InitializeSimulation()``.

After the initialization Basilisk starts the time loop evaluating the modules at the specified rate.  The ``Update()`` routine in :ref:`fswModuleTemplate` prints out the module I and the simulation time where the module is called.  Note that thanks to the module evaluation priorties we set the desired module execution order is achieved.

Adding C++ Modules
------------------
If the BSK module being added is a C++ module, then the above steps are simplified to the following.  Let the module be called ``someCppModule``.  As this is a C++ class, we don't need to create a data structure and wrap it as we do with a C module.  Rather, we can get a instance of the C++ module using::

    mod = someCppModule.SomeCppModule()
    mod.ModelTag = "moduleName"

To add this to a task use::

    scSim.AddModelToTask("taskName", mod, None, priority)

The 3rd argument is ``None`` as the module data is already contained in the module class.  To add a module without specifiying the priority you can use this shorter version as well::

    scSim.AddModelToTask("taskName", mod)


.. ** Tutorial Review Video **

    .. raw:: html

        <iframe width="560" height="315" src="https://www.youtube.com/embed/6YmZyu0f-qI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
