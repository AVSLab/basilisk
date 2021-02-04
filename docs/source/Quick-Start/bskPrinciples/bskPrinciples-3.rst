Connecting Messages
===================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-3.py>`.

Having learned how to add C or C++ Basilisk modules to a task and setting priorities, next we will look at how to connect the module messages.  Again we use both :ref:`fswModuleTemplate` and :ref:`cppModuleTemplate` as the stand-in modules to illustrate setting message connections.  Note that the input and output message connections of these modules are of the same type.  The following simulation script again uses a single process and task.  The modules are created and their input and output messages are connected as illustrated below.

.. image:: ../../_images/static/qs-bsk-3.svg
   :align: center

The source code is shown below.  As we are going to be using the Basilisk messaging system now, it is important to import the ``messing`` package from ``Basilisk.architecture``.  Without this the python code will not know how to subscribe to any message type, or how to create a stand-alone message.

.. literalinclude:: ../../codeSamples/bsk-3.py
   :language: python
   :linenos:
   :lines: 18-

The method connect an input message (variable name ending with ``InMsg``) to an output message (varible name ending with ``OutMsg``) is the ``.subscribeTo()`` method.  While C modules contain message objects with a C interface, and C++ modules contain C++ message objects, the ``.subscribeTo()`` method is setup such that the user doesn't have to worry about this distinction.  Rather, this method connects C to C, C to C++, C++ to C++ and C++ to C message connections.

Thus, given a module input message ``someModule.xxxInMsg`` and a module output message ``anotherModule.xxxOutMsg``, these are connected using::

    someModule.xxxInMsg.subscribeTo(anotherModule.xxxOutMsg)

The input and output message names are arbitrary.  However, the messages being connected must be of the same type.
In the above simulation code we use this protocol to connect the output message of the C module 1 to the input message of C++ module 2.  Next the output of C++ module 2 is connected to the input of C module 1 to create a sample closed-loop messaging setup.

.. warning::

    You can only subscribe an input message to an output message that already exists!  Don't try to subscribe to the message before it has been created.  In this simulation the subscriptions are all occurring after the modules are created.

If you execute this python code you should see the following terminal output:

.. code-block::

    source/codeSamples % python3 bsk-3.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: C Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: C Module ID 1 ran Update at 5.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 5.000000s

Note that here the 2 modules are added without setting a priority.  Thus, they are executed in the order that they were added to the Basilisk task.



.. ** Tutorial Review Video **

    .. raw:: html

        <iframe width="560" height="315" src="https://www.youtube.com/embed/6YmZyu0f-qI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
